#include "ros/ros.h"
#include <math.h> 
#include <stdio.h>

#include <ascend_msgs/DetectedRobotsGlobalPositions.h>
#include "planning_ros_sim/track.h"

#define PI 3.14159265

ascend_msgs::DetectedRobotsGlobalPositions data;

void tracker_chatterCallback(planning_ros_sim::track msg){
    for(int i = 0; i < 10; i++) {
    	if(i < msg.num_targets){
            data.global_robot_position[i].x = msg.position_x[i];
            data.global_robot_position[i].y = msg.position_y[i];
            data.direction[i] = atan(msg.velocity_y[i]/msg.velocity_x[i]);
        }
        else{
            break;
        }
    }
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "ai_conv");
    ros::NodeHandle node;

    ros::Subscriber data_sub = node.subscribe("/target_tracker/tracks", 1000, tracker_chatterCallback);
    
    ros::Publisher data_pub = node.advertise<ascend_msgs::DetectedRobotsGlobalPositions>("globalGroundRobotPosition", 1000);
    
    while (ros::ok()) {
        ros::Duration(0.1).sleep();
        ros::spinOnce();

        data_pub.publish(data);

    }
}
