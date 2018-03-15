#pragma once
#include <array>
#include <vector>
#include <stdio.h>

#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Time.h"

#include "ascend_msgs/DetectedRobotsGlobalPositions.h"
#include "ascend_msgs/AIWorldObservation.h"

#include "AI/Robot.h"
#include "AI/World.h"

std::array<Robot, 10> robots_in_memory;
std::vector<std::vector<Robot>> observed_robots;
std::array<Robot, 4> obstacle_robots_in_memory;
std::vector<std::vector<Robot>> observed_obstacle_robots;

void initializeRobotsInMemory();
void groundRobotCallback(ascend_msgs::DetectedRobotsGlobalPositions::ConstPtr msg);
void startTimeCallback(std_msgs::Time::ConstPtr msg);
void dronePositionCallback(geometry_msgs::PoseStamped::ConstPtr msg);
void aiSimCallback(ascend_msgs::AIWorldObservation::ConstPtr obs);
double distanceBetweenRobots(Robot r1, Robot r2);
int nearestNeighbor(Robot robot);
void updateRobot(Robot new_robot);
float calcCurrentTime(float seconds);

double distanceBetweenRobots(Robot r1, Robot r2) {
    return sqrt(pow(r1.getPosition().x - r2.getPosition().x,2)+pow(r1.getPosition().y - r2.getPosition().y,2));
}

int nearestNeighbor(Robot robot, std::set<int> used_index) {
    double min_distance = 2;
    int index = -1;
    int not_visible_index = -1;
    int counter = 0;

    for(auto it = robots_in_memory.begin(); it != robots_in_memory.end(); it++){

        if(used_index.find(counter) != used_index.end()){
            counter++;
            continue;
        }

        if(it->getVisible()){

            Robot robot_in_memory = it->getRobotPositionAtTime(robot.getTimeLastSeen());

            if(distanceBetweenRobots(robot, robot_in_memory) < min_distance) {
                min_distance = distanceBetweenRobots(robot, robot_in_memory);
                index = counter;
            }
        }

        else{
            not_visible_index = counter;
        }

        counter++;
    }

    if(index == -1){
        return not_visible_index;
    }

    return index;
}
