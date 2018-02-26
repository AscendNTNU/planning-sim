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

std::array<Robot, 10> robots;
std::vector<Robot> new_robots;
float start_time = 0;

float TIMEOUT_OBSERVATION = 5;

//Can only handle 10 robots in one message.
void tracker_chatterCallback(ascend_msgs::DetectedRobotsGlobalPositions::ConstPtr msg){
    for(int i = 0; i < (int)msg->count; i++) {
        Robot robot;

        point_t position;
        position.x = msg->global_robot_position[i].x;
        position.y = msg->global_robot_position[i].y;

        float q = msg->direction[i];
        float time = msg->header.stamp.sec-start_time;
        bool visible = true;

        robot.update(i, position, q , time, visible);
        new_robots.push_back(robot);
    }
}

double distanceBetweenRobots(Robot r1, Robot r2) {
    return sqrt(pow(r1.getPosition().x - r2.getPosition().x,2)+pow(r1.getPosition().y - r2.getPosition().y,2));
}

int nearestNeighbor(Robot robot) {
    double min_distance = 1000000;
    int index = -1;
    for(auto it = robots.begin(); it != robots.end(); it++){
        Robot robot = *it;
        int counter = 0;
        if(distanceBetweenRobots(robot, robots[counter]) < min_distance) {
            min_distance = distanceBetweenRobots(robot, robots[counter]);
            index = counter;
            counter++;
        }
    }
    return index;
}

void updateRobot(Robot new_robot){
    int nearest_robot_index = nearestNeighbor(new_robot);
    robots[nearest_robot_index].update(new_robot);
}

int main(int argc, char **argv){
    int counter = 0;
    for(auto it = robots.begin(); it != robots.end(); it++){
        *it = Robot(counter);
        counter++;
    }
    // Initialize ros-messages
    ros::init(argc, argv, "fuser");
    ros::NodeHandle node;
    // geometry_msgs::Pose2D drone_msg;
    std_msgs::Float32 time_msg; 

    // ros::Publisher ground_robots_pub = node.advertise<ascend_msgs::AIWorldObservation>("AIWorldObservation", 1);
    ros::Subscriber tracker_sub = node.subscribe("globalGroundRobotPosition", 100, tracker_chatterCallback);
    ros::Rate rate(30.0);

    while (ros::ok()) {
        ros::spinOnce();

        for(auto it = new_robots.begin(); it != new_robots.end(); it++){
            updateRobot(*it);
        }
        
        for(auto it = robots.begin(); it != robots.end(); it++){
            if(current_time - it->getTimeLastSeen() > TIMEOUT_OBSERVATION){
                it->setVisible(FALSE);
            }
        }


        ascend_msgs::DetectedRobotsGlobalPositions groundrobot_msg;


        rate.sleep();
    }
    return 0;
}
