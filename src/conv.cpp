#include "ros/ros.h"
#include <math.h> 
#include <stdio.h>

#include <ascend_msgs/DetectedRobotsGlobalPositions.h>
#include "planning_ros_sim/track.h"

#define PI 3.14159265

ascend_msgs::DetectedRobotsGlobalPositions groundrobot_msg;

    


void tracker_chatterCallback(planning_ros_sim::track msg){
    
    if(msg.num_targets == 0){
        return;
    }

    ascend_msgs::DetectedRobotsGlobalPositions groundrobot_msg;
    
    for(int i = 0; i < 10; i++) {
        if(i < msg.num_targets){
            geometry_msgs::Point32 robot_position;
            std_msgs::Float32 direction;
    
            robot_position.x = msg.position_x[i];
            robot_position.y = msg.position_y[i];
   
            direction.data = atan(msg.velocity_y[i]/msg.velocity_x[i]);
   
            groundrobot_msg.global_robot_position.push_back(robot_position);
            groundrobot_msg.direction.push_back(obs_state.target_q[n]);
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
