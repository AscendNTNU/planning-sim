#pragma once
#include <cmath>
#include <queue>
#include <iostream>
static const int DRONE_SPEED = 1;
static const double ROBOT_SPEED = 0.33;
static const double MATH_PI = 3.141592653589793238;
static const double SIMILARITY_THRESHOLD = 1.5;

static const double MAXDIST_DRONE_TO_POINT = 0.2;
static const double MAXDIST_ROBOT_TO_POINT = 1.8; 
static const double MAXDIST_ACTIONPOINTS = 200; // essentially how much the AI will change its mind (high number = frequent action reconsiderations)

/**
@brief Struct describing a point on the course.
@param x X coordinate
@param y Y coordinate
@param z Z coordinate
@param travel_Time Travel time to the point
*/
struct point_t{
	double x;
	double y;
	double z;
	double travel_Time;
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
    double time_till_first_arrival;
    double time_since_start_turn;
};

static point_t point_zero = {
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
	int x_max;
	int y_max;
};

/**
@brief Struct describing the current state.
@param elapsed_time Time since start of the game
@param drone_x X coordinate of drone
@param drone_y Y coordinate of drone
@param drone_cmd_done If drone is doing an action or not
@param num_targets Number of targets in the game
@param robot_x X coordinate of a robot
@param robot_y Y coordinate of a robot
@param robot_q Angle of a robot in radians
@param obstacle_x X coordinate of an obstacle
@param obstacle_y Y coordinate of an obstacle
@param obstacle_q Angle of an obstacle in radians
*/
struct observation_t
{
    double elapsed_time;
    double drone_x;
    double drone_y;
    double drone_z;
    bool  drone_cmd_done;
    int num_targets;

    double robot_x[10];
    double robot_y[10];
    double robot_q[10];
    double robot_visible[10];

    double obstacle_x[4];
    double obstacle_y[4];
    double obstacle_q[4];
};

static observation_t observation_Empty = {
    .elapsed_time = 0,
    .drone_x = 0,
    .drone_y = 0,
    .drone_z = 0,
    .drone_cmd_done = false,
    .num_targets = 0,
};

enum action_Type_t
{
    no_command = 0,   // continue doing whatever you are doing
    land_on_top_of,   // trigger one 45 deg turn of robot (i)
    land_in_front_of, // trigger one 180 deg turn of robot (i),
    land_at_point,    // land at a given point
    take_off,         // take off
    track,            // follow robot (i) at a constant height
    search            // ascend to 3 meters and go to (x, y)
};

inline std::string actionTypeToString(action_Type_t type) {
    switch(type) {
        case no_command:
            return "no command";
        case land_on_top_of:
            return "land on top of";
        case land_in_front_of:
            return "land in front of";
        case land_at_point:
            return "land at point";
        case search:
            return "search";
        case take_off:
            return "take_off";
    }
}


struct action_t{
    int target;
    action_Type_t type;
    double reward;
    double when_To_Act;
    point_t where_To_Act;
};

static action_t empty_action = {
    .target = -1,
	.type = no_command,
	.reward = -200000,
	.when_To_Act = 0,
	.where_To_Act = point_zero
};

inline std::ostream& operator<<(std::ostream &strm, const action_t &action) {
    strm << "--- Action ---" << std::endl
    << "Target: " << action.target << std::endl
    << "Type: " << actionTypeToString(action.type) << std::endl
    << "Reward: " << action.reward << std::endl
    << "Where to act: " << action.where_To_Act << std::endl;
    return strm;
};

struct tree_action_t{
    std::queue<action_t> actions;
    double reward;
};

static double getDistanceBetweenPoints(point_t point1, point_t point2){

    double x_Distance = point1.x - point2.x;
    double y_Distance = point1.y - point2.y;

    return sqrt(pow(x_Distance,2) + pow(y_Distance,2));
}
