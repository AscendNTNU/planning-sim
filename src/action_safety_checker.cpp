#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/Pose2D.h"
#include "AI/structs.h"
#include <stdio.h>
#include <actionlib/server/simple_action_server.h>
#include <ascend_msgs/ControlFSMAction.h>

//Typedefs
using ActionServerType = actionlib::SimpleActionServer<ascend_msgs::ControlFSMAction>;
using GoalType = ascend_msgs::ControlFSMGoal;

action_t action_ROS2Plank(GoalType goal) {
    action_t action = empty_action;

    // Set action type
    switch (goal.cmd) {
        case ascend_msgs::ControlFSMGoal::LAND_ON_TOP_OF:
            action.type = land_on_top_of;
            break;
        case ascend_msgs::ControlFSMGoal::LAND_AT_POINT:
            action.type = land_at_point;
            break;
        case ascend_msgs::ControlFSMGoal::SEARCH:
            action.type = search;
            break;
        default:
            ROS_INFO("Action type: %i was not recognized.", (int)goal.cmd);
            action.type = no_command;
            break;
    }

    action.target = goal.target_id;
    action.where_To_Act.x = goal.x;
    action.where_To_Act.y = goal.y;
    action.where_To_Act.z = goal.z;
    // Is used by the sim to show reward in gui
    action.reward = goal.reward;

    return action;
}

//Accepts new goal from client when recieved
void newGoalCB(ActionServerType* server) {
    //Check there really is a new one available
    if(!server->isNewGoalAvailable()) return;
    //Accept the new goal
    ascend_msgs::ControlFSMGoal goal = *server->acceptNewGoal();
    //Check that the client hasn't cancelled the request already calls
    if(server->isPreemptRequested()) {
        //Goal will be preemted by client
        server->setPreempted();
    return;
    }
    std::cout << action_ROS2Plank(goal) << std::endl;
}

//Terminates current goal when requested.
void preemptCB(ActionServerType* server) {
    //Abort whatever you are doing first!
    // Should we send no_command to sim?
    ROS_WARN("Action preemted by planning node before safety node answered!");
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "action_safety_checker");
    ros::NodeHandle nh;

    // Define action server
    ActionServerType server(nh, "action_safety_checker_server", false);
    server.registerGoalCallback(boost::bind(newGoalCB, &server));
    server.registerPreemptCallback(boost::bind(preemptCB, &server));
    server.start();

    ros::Rate rate(30.0);
    while (ros::ok()) {
        ros::spinOnce();

        // Ask user if goal is OK.
        if (server.isActive()) {
            std::cin.clear();
            std::string inputString;
            std::cout << "Accept the goal? (yes/no)\n";
            std::getline(std::cin, inputString);

            if(inputString.compare("yes") == 0 && server.isActive()) {
                server.setSucceeded();
            } else {
                server.setAborted();
            }
        }
        rate.sleep();
    }
    return 0;
}
