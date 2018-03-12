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
std::vector<Robot> observed_robots;
std::array<Robot, 4> obstacle_robots_in_memory;
std::vector<Robot> observed_obstacle_robots;

void initializeRobotsInMemory();
void groundRobotCallback(ascend_msgs::DetectedRobotsGlobalPositions::ConstPtr msg);
void startTimeCallback(std_msgs::Time::ConstPtr msg);
void dronePositionCallback(geometry_msgs::PoseStamped::ConstPtr msg);
void aiSimCallback(ascend_msgs::AIWorldObservation::ConstPtr obs);
double distanceBetweenRobots(Robot r1, Robot r2);
int nearestNeighbor(Robot robot);
void updateRobot(Robot new_robot);
float calcCurrentTime(float seconds);
