#pragma once

#include <queue>
#include <iostream>
static const int DRONE_SPEED = 1;
static const float ROBOT_SPEED = 0.33;
static const float MATH_PI = 3.141592653589793238;

struct point_t{
	float x;
	float y;
	float z;
	float travel_Time;
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

struct bounds_t{
	int x_Max;
	int y_Max;
};

enum drone_State_t
{
    landing_On_Top,
    landing_In_Front, // Equal to landing_At_Point?
    landing_At_Point,
    waiting,
    tracking,
	choosing_Action,
	choosing_Target,
	starting,
	waiting_For_Action,
	no_Target_Found,
	terminate
};

struct observation_t
{
    float elapsed_time;
    float drone_x;
    float drone_y;
    bool  drone_cmd_done;

    float robot_x[10];
    float robot_y[10];
    float robot_q[10];

    float obstacle_x[4];
    float obstacle_y[4];
    float obstacle_q[4];
};


enum action_Type_t
{
    no_Command = 0,   // continue doing whatever you are doing
    land_On_Top_Of,     // trigger one 45 deg turn of robot (i)
    land_In_Front_Of,   // trigger one 180 deg turn of robot (i),
    land_At_Point,	 // land at a given point
    track,           // follow robot (i) at a constant height
    search           // ascend to 3 meters and go to (x, y)
};

struct action_t{
    int target;
    action_Type_t type;
    float reward;
    float when_To_Act;
    point_t where_To_Act;
};

static action_t action_Empty = {
    .target = -1,
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
    << "When to act: " << action.when_To_Act << std::endl
    << "Where to act: " << action.where_To_Act << std::endl;
    return strm;
};

struct tree_action_t{
    std::queue<action_t> actions;
    float reward;
};
