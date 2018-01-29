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
    //<< "When to act: " << action.when_To_Act << std::endl
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

static bool pointsWithinThreshold(point_t point_one, point_t point_two, float threshold){
    float distance = getDistanceBetweenPoints(point_one, point_two);
    if(distance <= threshold){
        return true;
    }
    return  false;
}

static float getGridValue(float X, float Y){
    float value = (-9.995004e+02)+(9.976812e+01)*X+(-1.004701e+02)*Y
        +(-5.785388e+01)*pow(X,2)+(1.161562e+01)*X*Y+(5.477725e+01)*pow(Y,2)
        +(1.260229e+01)*pow(X,3)+(1.299816e+01)*pow(X,2)*Y+(-1.438667e+01)*X*pow(Y,2)+(-1.158062e+01)*pow(Y,3)
        +(-1.404096e+00)*pow(X,4)+(-3.106303e+00)*pow(X,3)*Y+(4.263504e-01)*pow(X,2)*pow(Y,2)
        +(2.851553e+00)*X*pow(Y,3)+(1.301842e+00)*pow(Y,4)
        +(9.053408e-02)*pow(X,5)+(2.901147e-01)*pow(X,4)*Y+(1.327346e-01)*pow(X,3)*pow(Y,2)
        +(-1.761180e-01)*pow(X,2)*pow(Y,3)+(-2.603853e-01)*X*pow(Y,4)+(-8.415694e-02)*pow(Y,5)
        +(-3.615309e-03)*pow(X,6)+(-1.235169e-02)*pow(X,5)*Y+(-1.602868e-02)*pow(X,4)*pow(Y,2)
        +(3.840976e-03)*pow(X,3)*pow(Y,3)+(1.239923e-02)*pow(X,2)*pow(Y,4)
        +(1.283802e-02)*X*pow(Y,5)+(3.201336e-03)*pow(Y,6)
        +(8.890888e-05)*pow(X,7)+(1.960570e-04)*pow(X,6)*Y+(7.353331e-04)*pow(X,5)*pow(Y,2)
        +(-9.145182e-05)*pow(X,4)*pow(Y,3)+(8.794847e-10)*pow(X,3)*pow(Y,4)
        +(-6.113303e-04)*pow(X,2)*pow(Y,5)+(-2.451141e-04)*X*pow(Y,6)+(-7.627948e-05)*pow(Y,7)
        +(-1.058445e-06)*pow(X,8)+(4.059809e-11)*pow(X,7)*Y+(-1.167195e-05)*pow(X,6)*pow(Y,2)
        +(-4.630460e-12)*pow(X,5)*pow(Y,3)+(-1.355465e-11)*pow(X,4)*pow(Y,4)
        +(-5.731993e-12)*pow(X,3)*pow(Y,5)+(1.167198e-05)*pow(X,2)*pow(Y,6)
        +(3.539047e-11)*X*pow(Y,7)+(1.058675e-06)*pow(Y,8);

    return value;
}