#include <array>
#include <vector>

#include "ros/ros.h"

#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Bool.h"

#include "ascend_msgs/DetectedRobotsGlobalPositions.h"

#include "AI/Robot.h"
#include "AI/World.h"

World world = World(0);

std::array<Plank, 10> planks;
std::vector<Robot> robots;
float start_time = 0;
//Can only handle 10 robots in one message.
void tracker_chatterCallback(ascend_msgs::DetectedRobotsGlobalPositions::ConstPtr msg){

    for(int i = 0; i < 10; i++) {
        if(i < (int)msg->count){

            Robot robot;
            point_t position;
            position.x = msg->global_robot_position[i].x;
            position.y = msg->global_robot_position[i].y;
            float q = msg->direction[i];
            float time = msg->header.stamp.sec-start_time;
            bool visible = true;
            robot.update(i, position, q , time, visible);
            robots.push_back(robot);
        }
        else{
            break;
        }
    }
}

int main(int argc, char **argv){

    for(auto it = planks.begin(); it != planks.end(); it++){
        *it = Plank();
    }
    // Initialize ros-messages
    ros::init(argc, argv, "fuser");
    ros::NodeHandle node;
    // geometry_msgs::Pose2D drone_msg;
    std_msgs::Float32 time_msg; 

    // ros::Publisher ground_robots_pub = node.advertise<ascend_msgs::AIWorldObservation>("AIWorldObservation", 1);
    ros::Subscriber tracker_sub = node.subscribe("globalGroundRobotPosition", 1, tracker_chatterCallback);
    ros::Rate rate(30.0);

    while (ros::ok()) {
        ros::spinOnce();
        
        ascend_msgs::DetectedRobotsGlobalPositions groundrobot_msg;
        // groundrobot_msg.count = 10;
        // for (int n = 0; n<10; n++){
        //     geometry_msgs::Point32 robot_position;
        //     std_msgs::Float32 direction;
        //     robot_position.x = obs_state.target_x[n];
        //     robot_position.y = obs_state.target_y[n];
        //     direction.data = obs_state.target_q[n];
        //     groundrobot_msg.global_robot_position.push_back(robot_position);
        //     groundrobot_msg.direction.push_back(obs_state.target_q[n]);
        // }

        // ground_robots_pub.publish(groundrobot_msg);
        // drone_pub.publish(drone_msg);

        // time_msg.data = state.elapsed_time;
        // elapsed_time_pub.publish(time_msg);

        rate.sleep();
    }
    return 0;
}
