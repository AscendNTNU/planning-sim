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
    }
    ai_controller.observation.updateRobot(robotObs, elapsed_time);
}

void drone_chatterCallback(geometry_msgs::Pose2D msg) {
    observation_t droneObs = observation_Empty;
    droneObs.drone_x = msg.x;
    droneObs.drone_y = msg.y;
    ai_controller.observation.updateDrone(droneObs, elapsed_time);
}

// When is GO_TO_XYZ used?
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
        case land_In_Front_Of: // We need to send a new action if we observe that the robot position drifts
            drone_action.cmd = ascend_msgs::ControlFSMGoal::LAND_AT_POINT;
            break;
        case land_At_Point:
            drone_action.cmd = ascend_msgs::ControlFSMGoal::LAND_AT_POINT;
            break;
        case track:
            drone_action.cmd = ascend_msgs::ControlFSMGoal::TRACK;
            break;
        case search:
            drone_action.cmd = ascend_msgs::ControlFSMGoal::SEARCH;
            break;
        default:
            ROS_INFO("Action type: %i", (int)action.type);
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
    ros::NodeHandle nh;

    ros::Subscriber time_sub = nh.subscribe("time_chatter", 1000, time_chatterCallback);
    ros::Subscriber ground_robot_sub = nh.subscribe("groundrobot_chatter", 1000, groundRobot_chatterCallback);
    ros::Subscriber drone_sub = nh.subscribe("drone_chatter", 1000, drone_chatterCallback);

    ClientType client("perception_control", true);
    client.waitForServer(); //Waits until server is ready

    ascend_msgs::ControlFSMGoal drone_action;
    
    action_t action = empty_action;

    while (ros::ok()) {
        // do we still need this?
        ros::Duration(0.4).sleep();
        ros::spinOnce();

        action = ai_controller.stateHandler();
        drone_action = action_plank2ROS(action);
        client.sendGoal(drone_action);

        // Can be passed ros::Duration(20) to timeout after 20 seconds
        bool client_result = client.waitForResult(); 
        //Check status
        if(client_result && 2.5 < fmod(elapsed_time, 20) && fmod(elapsed_time, 20) < 17.5) {
            auto state = client.getState();
            if(state == state.SUCCEEDED) {
                //Pointer to result
                auto result_p = client.getResult();
            } else if(state == state.ABORTED) {
                //Ups, something went wrong, Control aborted action
                continue;
            }
        }
    }
}
