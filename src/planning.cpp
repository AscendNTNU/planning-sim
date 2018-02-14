#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/Pose2D.h"

#include <ascend_msgs/DetectedRobotsGlobalPositions.h>

#include <actionlib/client/simple_action_client.h>
#include <stdio.h>
#include "AI/AIController.h"
#include "AI/structs.h"

#include <actionlib/client/simple_action_client.h>
#include <ascend_msgs/ControlFSMAction.h>

using ClientType = actionlib::SimpleActionClient<ascend_msgs::ControlFSMAction>;
using GoalState = actionlib::SimpleClientGoalState;

float elapsed_time = 0;

World world = World(0);
AIController ai_controller = AIController();

void time_chatterCallback(std_msgs::Float32 msg) {
    elapsed_time = (float)msg.data;
}

void drone_chatterCallback(geometry_msgs::Pose2D msg) {
    observation_t droneObs = observation_Empty;
    droneObs.drone_x = msg.x;
    droneObs.drone_y = msg.y;
    ai_controller.observation.updateDrone(droneObs, elapsed_time);
}

void tracker_chatterCallback(ascend_msgs::DetectedRobotsGlobalPositions::ConstPtr msg){
    observation_t robotObs = observation_Empty;
    for(int i = 0; i < 10; i++) {
    	if(i < (int)msg->count){
            robotObs.robot_x[i] = msg->global_robot_position[i].x;
            robotObs.robot_y[i] = msg->global_robot_position[i].y;
            robotObs.robot_q[i] = msg->direction[i];
            robotObs.robot_visible[i] = true;
        }
        else{
            robotObs.robot_visible[i] = false;
        }
    }
    elapsed_time = 10;
    ai_controller.observation.updateRobot(robotObs, elapsed_time);   
}

ascend_msgs::ControlFSMGoal action_plank2ROS(action_t action) {
    // Instansiate the action message
    ascend_msgs::ControlFSMGoal drone_action;
    // Tell server who we are
    drone_action.caller_id = ascend_msgs::ControlFSMGoal::CALLER_AI;
    // Set action type
    switch (action.type) {
        case land_On_Top_Of:
            drone_action.cmd = ascend_msgs::ControlFSMGoal::LAND_ON_TOP_OF;
            break;
        case land_In_Front_Of:
            drone_action.cmd = ascend_msgs::ControlFSMGoal::LAND_AT_POINT;
            break;
        case search:
            drone_action.cmd = ascend_msgs::ControlFSMGoal::SEARCH;
            break;
        default:
            ROS_INFO("Action type: %i was not recognized.", (int)action.type);
            // This should never happen.
            drone_action.cmd = ascend_msgs::ControlFSMGoal::NO_COMMAND;
            break;
    }

    drone_action.target_id = action.target;
    drone_action.x = action.where_To_Act.x;
    drone_action.y = action.where_To_Act.y;
    drone_action.z = action.where_To_Act.z;
    // Is used by the sim to show reward in gui
    drone_action.reward = action.reward;

    return drone_action;
}


int main(int argc, char **argv) {
    
    ros::init(argc, argv, "planning");
    ros::NodeHandle node;
    ros::Subscriber time_sub = node.subscribe("time_chatter", 1000, time_chatterCallback);
    ros::Subscriber drone_sub = node.subscribe("drone_chatter", 1000, drone_chatterCallback);
    ros::Subscriber tracker_sub = node.subscribe("globalGroundRobotPosition", 1000, tracker_chatterCallback);
    
    ClientType client("control_action_server", true);
    client.waitForServer(); //Waits until server is ready
    ascend_msgs::ControlFSMGoal drone_action;

    bool ready_for_new_action = true;
    action_t action = empty_action;

    // For only printing current action when it is changed
    // --------------------------------
    int current_action_type = -1;
    std::__cxx11::basic_string<char> current_action_state = "None";
    // --------------------------------

    //Give in a center drone position for testing when we don't get in drone position
    observation_t droneObs = observation_Empty;
    droneObs.drone_x = 10;
    droneObs.drone_y = 10;
    ai_controller.observation.updateDrone(droneObs, elapsed_time);

    ros::Rate rate(0.2);

    while (ros::ok()) {
        ros::spinOnce();

        if(ready_for_new_action) {
            action = ai_controller.stateHandler();

            if (action.type == no_Command) {
                rate.sleep();
                printf("Action type is no command\n");
                continue;
            } else {
                ready_for_new_action = false;
                drone_action = action_plank2ROS(action);
                client.sendGoal(drone_action);
            }
        }

        GoalState action_state = client.getState();

        if (action.type != current_action_type || action_state.toString() != current_action_state){
            std::cout << std::endl << "Action: " << action << std::endl;
            std::cout << "-----------" << action_state.toString() << "-------" << std::endl;
            current_action_type = action.type;
            current_action_state = action_state.toString();
        }

        // When no break is present, it falls through to next case
        switch(action_state.state_){
            case GoalState::PENDING:
                // Control node is processing the action
            case GoalState::ACTIVE:
                ready_for_new_action = false;
                break;
            case GoalState::RECALLED:
                // We, the planning node emediately canceled
            case GoalState::PREEMPTED:
                // We, the planning node cancel the goal after a while
            case GoalState::REJECTED:
                // Control rejected the action
            case GoalState::ABORTED:
                // Control node aborted the goal
                    // Fly higher to see more?
                    // Lift off ground so we dont get disqualified?
            case GoalState::SUCCEEDED:
                // The goal was successfull!
            case GoalState::LOST:
                // Control node has no goal
            default:
                ready_for_new_action = true;
                break;
        }

        rate.sleep();
    }
}
