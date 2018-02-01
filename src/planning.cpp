#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/Pose2D.h"
#include "planning_ros_sim/groundRobotList.h"
#include "planning_ros_sim/groundRobot.h"
#include "planning_ros_sim/droneCmd.h"
#include <actionlib/client/simple_action_client.h>
#include <stdio.h>
#include "AI/AIController.h"
#include "AI/structs.h"

#include <actionlib/client/simple_action_client.h>
#include <ascend_msgs/ControlFSMAction.h>
using ClientType = actionlib::SimpleActionClient<ascend_msgs::ControlFSMAction>;
using GoalState = actionlib::SimpleClientGoalState;

planning_ros_sim::groundRobotList GroundRobots;
geometry_msgs::Pose2D Drone;

float elapsed_time = 0;

World world = World(0);
AIController ai_controller = AIController();

void time_chatterCallback(std_msgs::Float32 msg) {
    elapsed_time = (float)msg.data;
}

void groundRobot_chatterCallback(const planning_ros_sim::groundRobotList &msg) {
    observation_t robotObs = observation_Empty;
    for(int i = 0; i < 10; i++) {
            robotObs.robot_x[i] = msg.groundRobot[i].x;
            robotObs.robot_y[i] = msg.groundRobot[i].y;
            robotObs.robot_q[i] = msg.groundRobot[i].theta;
            // robotObs.robot_visible[i] = msg.groundRobot[i].visible;
            robotObs.robot_visible[i] = true;
    }
    ai_controller.observation.updateRobot(robotObs, elapsed_time);
}

void drone_chatterCallback(geometry_msgs::Pose2D msg) {
    observation_t droneObs = observation_Empty;
    droneObs.drone_x = msg.x;
    droneObs.drone_y = msg.y;
    ai_controller.observation.updateDrone(droneObs, elapsed_time);
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

// 
bool robotsAtTurnTime(float elapsed_time) {
    if (2.5 > fmod(elapsed_time, 20) || fmod(elapsed_time, 20) < 17.5) {
        return true;
    }
    return false;
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "planning");
    ros::NodeHandle nh;

    ros::Subscriber time_sub = nh.subscribe("time_chatter", 1000, time_chatterCallback);
    ros::Subscriber ground_robot_sub = nh.subscribe("groundrobot_chatter", 1000, groundRobot_chatterCallback);
    ros::Subscriber drone_sub = nh.subscribe("drone_chatter", 1000, drone_chatterCallback);

    ClientType client("control_action_server", true);
    client.waitForServer(); //Waits until server is ready

    ascend_msgs::ControlFSMGoal drone_action;
    action_t action = empty_action;
    bool ready_for_new_action = true;

    ros::Rate rate(30.0);
    while (ros::ok()) {
        ros::spinOnce();

        if(ready_for_new_action) {
            printf("Getting new action");
            action = ai_controller.stateHandler();
            printf("Got new action");

            if (robotsAtTurnTime(elapsed_time) || action.type == no_Command) {
                printf("Continue loop");
                continue;
            }
            drone_action = action_plank2ROS(action);
            client.sendGoal(drone_action);
            printf("Sendt goal");
        }

        GoalState action_state = client.getState();
        std::cout << "Action type: " << action.type << std::endl;
        std::cout << "-----------" << action_state.toString() << "-------" << std::endl;
        // When no break is present, it falls through to next case
        switch(action_state.state_){
            case GoalState::PENDING:
                // Control node is processing the action
            case GoalState::ACTIVE:
                printf("busy\n");
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
                printf("free\n");
                ready_for_new_action = true;
                break;
        }

        rate.sleep();
    }
}
