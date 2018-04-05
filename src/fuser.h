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

extern std::vector<Robot> robots_in_memory;
extern std::vector<std::vector<Robot>> observed_robots;
extern std::vector<Robot> obstacle_robots_in_memory;
extern std::vector<std::vector<Robot>> observed_obstacle_robots;

void groundRobotCallback(ascend_msgs::DetectedRobotsGlobalPositions::ConstPtr msg);
void startTimeCallback(std_msgs::Time::ConstPtr msg);
void dronePositionCallback(geometry_msgs::PoseStamped::ConstPtr msg);
void aiSimCallback(ascend_msgs::AIWorldObservation::ConstPtr obs);

float calcCurrentTime(float seconds);

void initializeRobotsInMemory();
int nearestNeighbor(Robot robot, std::vector<Robot> memory, float current_time);
void updateRobots(std::vector<Robot> robots_in_single_message,std::vector<Robot> &memory, float current_time);

double distanceBetweenRobots(Robot r1, Robot r2) {
    return sqrt(pow(r1.getPosition().x - r2.getPosition().x,2)+pow(r1.getPosition().y - r2.getPosition().y,2));
}