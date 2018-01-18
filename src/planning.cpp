#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/Pose2D.h"
#include "planning_ros_sim/groundRobotList.h"
#include "planning_ros_sim/groundRobot.h"
#include "planning_ros_sim/droneCmd.h"
#include "planning_ros_sim/track.h"
#include <actionlib/client/simple_action_client.h>
#include <stdio.h>
#include "AI/AIController.h"
#include "AI/structs.h"

planning_ros_sim::groundRobotList GroundRobots;
geometry_msgs::Pose2D Drone;

float elapsed_time = 0;
bool action_done = true;
World world = World(0);
AIController ai_controller = AIController();

void time_chatterCallback(std_msgs::Float32 msg) {
    elapsed_time = (float)msg.data;
}

void tracker_chatterCallback(planning_ros_sim::track msg){
    observation_t robotObs = observation_Empty;
    for(int i = 0; i < msg.num_targets; i++) {
            robotObs.robot_x[i] = msg.position_x[i];
            robotObs.robot_y[i] = msg.position_y[i];
            robotObs.robot_q[i] = 0;
            robotObs.robot_visible[i] = true;
    }
    elapsed_time = 20 - msg.time_until_180;
    ai_controller.observation.updateRobot(robotObs, elapsed_time);   
}

void command_done_chatterCallback(std_msgs::Bool msg) {
    action_done = (bool)msg.data;
}

planning_ros_sim::droneCmd to_ROS_Command(action_t action) {
    planning_ros_sim::droneCmd command;
    command.x = action.where_To_Act.x;
    command.y = action.where_To_Act.y;
    command.z = 0;
    command.cmd = (int)action.type;
    command.target_id = action.target;
    command.reward = action.reward;

    return command;
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "planning");
    ros::NodeHandle time_node;
    ros::NodeHandle command_node;


    ros::Subscriber tracker_sub = time_node.subscribe("tracker_chatter", 1000, tracker_chatterCallback);
    
    ros::Publisher command_pub = command_node.advertise<planning_ros_sim::droneCmd>("drone_cmd_chatter", 1000);
    
    planning_ros_sim::droneCmd drone_action;
    
    world.startTimer();

    action_t action = empty_action;

    observation_t droneObs = observation_Empty;
    droneObs.drone_x = 10.0;
    droneObs.drone_y = 10.0;
    ai_controller.observation.updateDrone(droneObs, elapsed_time);


    while (ros::ok()) {
        ros::Duration(0.4).sleep();
        ros::spinOnce();

        if(action_done && 2.5 < fmod(elapsed_time, 20) && fmod(elapsed_time, 20) < 17.5 ) {

            action = ai_controller.stateHandler();

            if(action.type != no_Command){
                printf("sending command\n");
                drone_action = to_ROS_Command(action);
                command_pub.publish(drone_action);
            }
        }
    }
}
