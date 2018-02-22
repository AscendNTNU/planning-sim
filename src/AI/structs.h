#pragma once
#include <cmath>
#include <queue>
#include <iostream>
static const int DRONE_SPEED = 1;
static const float ROBOT_SPEED = 0.33;
static const float MATH_PI = 3.141592653589793238;
static const float SIMILARITY_THRESHOLD = 1;

/**
@brief Struct describing a point on the course.
@param x X coordinate
@param y Y coordinate
@param z Z coordinate
@param travel_Time Travel time to the point
*/
struct point_t{
	float x;
	float y;
	float z;
	float travel_Time;
};

/**
@brief Struct describing a point on a plank.
@param point point being described
@param is_ahead true if the point is ahead or on top of the robot
@param time_till_first_arrival time in seconds until the robot would arrive at the point
*/
struct plank_point_t{
    point_t point;
    bool is_ahead;  
    float time_till_first_arrival;
    float time_since_start_turn;
};

static point_t point_Zero = {
	.x = 0.0,
	.y = 0.0,
	.z = 0.0,
	.travel_Time = 0.0
};

inline std::ostream& operator<<(std::ostream &strm, const point_t &point) {
    strm << "[" << point.x << ", " << point.y << "]";
    return strm;
};

/**
@brief Struct describing the size of the course.
@param x X coordinate
@param y Y coordinate
*/
struct bounds_t{
	int x_Max;
	int y_Max;
};

/**
@brief Struct describing the current state.
@param elapsed_time Time since start of the game
@param drone_x X coordinate of drone
@param drone_y Y coordinate of drone
@param drone_cmd_done If drone is doing an action or not
@param num_Targets Number of targets in the game
@param robot_x X coordinate of a robot
@param robot_y Y coordinate of a robot
@param robot_q Angle of a robot in radians
@param obstacle_x X coordinate of an obstacle
@param obstacle_y Y coordinate of an obstacle
@param obstacle_q Angle of an obstacle in radians
*/
struct observation_t
{
    float elapsed_time;
    float drone_x;
    float drone_y;
    bool  drone_cmd_done;
    int num_Targets;

    float robot_x[10];
    float robot_y[10];
    float robot_q[10];
    float robot_visible[10];

    float obstacle_x[4];
    float obstacle_y[4];
    float obstacle_q[4];
};

static observation_t observation_Empty = {
    .elapsed_time = 0,
    .drone_x = 0,
    .drone_y = 0,
    .drone_cmd_done = false,
    .num_Targets = 0,
};

enum action_Type_t
{
    no_Command = 0,   // continue doing whatever you are doing
    land_On_Top_Of,   // trigger one 45 deg turn of robot (i)
    land_In_Front_Of, // trigger one 180 deg turn of robot (i),
    land_At_Point,    // land at a given point
    track,            // follow robot (i) at a constant height
    search            // ascend to 3 meters and go to (x, y)
};

/*
std::string actionTypeToString(action_Type_t type) {
    switch(type) {
        case no_Command:
            return "no command";
            break;
        case land_On_Top_Of:
            return "land on top of";
            break;
        case land_In_Front_Of:
            return "land in front of";
            break;
        case land_At_Point:
            return "land at point";
            break;
        case search:
            return "search";
    }
}
*/

struct action_t{
    int target;
    action_Type_t type;
    float reward;
    float when_To_Act;
    point_t where_To_Act;
};

static action_t empty_action = {
    .target = 0,
	.type = no_Command,
	.reward = -200000,
	.when_To_Act = 0,
	.where_To_Act = point_Zero
};

inline std::ostream& operator<<(std::ostream &strm, const action_t &action) {
    strm << "--- Action ---" << std::endl
    << "Target: " << action.target << std::endl
    << "Type: " << action.type << std::endl
    << "Reward: " << action.reward << std::endl
    << "Where to act: " << action.where_To_Act << std::endl;
    return strm;
};

struct tree_action_t{
    std::queue<action_t> actions;
    float reward;
};

static float getDistanceBetweenPoints(point_t point1, point_t point2){

    float x_Distance = point1.x - point2.x;
    float y_Distance = point1.y - point2.y;

    return sqrt(pow(x_Distance,2) + pow(y_Distance,2));
}
