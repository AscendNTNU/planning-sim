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

void initializeFuser();
void updateRobots(std::vector<Robot> robots_in_single_message,std::vector<Robot> &memory, float current_time);

double distanceBetweenRobots(Robot r1, Robot r2) {
    return sqrt(pow(r1.getPosition().x - r2.getPosition().x,2)+pow(r1.getPosition().y - r2.getPosition().y,2));
}

int nearestNeighbor(Robot robot, std::vector<Robot> memory, std::set<int> free_indices) {

    double min_distance = 3;
    int index = -1;
    int not_visible_index = -1;
    int counter = 0;

    //Loop through our robot memory
    for(auto it = memory.begin(); it != memory.end(); it++){

        //If we already have updated this robot in memory, skip
        if(free_indices.find(counter) == free_indices.end()){
            counter++;
            continue;
        }

        //If the robot in memory is visible, check the distance between the newly observed
        //robot and where the robot in memory should be at the corresponding time.
        if(it->getVisible()){

            Robot robot_in_memory = *it;

            //If this distance is small, update the robot in memory.
            if(distanceBetweenRobots(robot, robot_in_memory) < min_distance) {
                min_distance = distanceBetweenRobots(robot, robot_in_memory);
                index = counter;
            }
        }

        // Save an index of a not visible robot incase no visible robots match the new
        // observation.
        else{
            not_visible_index = counter;
        }

        counter++;
    }

    //If we found no matching robot, return the index to a not visible robot in memory to
    // replace with the new observation.
    if(index == -1){
        return not_visible_index;
    }

    return index;
}
