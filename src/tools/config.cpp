#include "config.h"
// #include <ascend_msgs/ReloadConfig.h>

using planning::Config;

std::set<std::string> Config::missing_param_set_;

//===============  Global config params  =================

std::string Config::CONTROL_FSM_ACTION_SERVER;
std::string Config::AI_SIM_OBSERVATION_TOPIC;
float Config::TOTAL_COMPETITION_TIME;
float Config::ROS_RATE_PERCEPTION_CONTROL;
float Config::ROS_RATE_PLANNING;
float Config::ROBOT_REVERSE_INTERVAL;
float Config::ROBOT_TURN_TIME;
float Config::ROBOT_SPEED;
float Config::LAND_TIME;
float Config::MAX_GROUND_TIME;
float Config::DRONE_SPEED;
float Config::SIMILARITY_THRESHOLD;
float Config::MAXDIST_DRONE_TO_POINT;
float Config::MAXDIST_ROBOT_TO_POINT; 
float Config::MAXDIST_ACTIONPOINTS;
float Config::INITIAL_PLANK_REWARD;
float Config::ACTION_EDGE_BUFFER;
float Config::RELIABLE_DATA_HEIGHT;
float Config::POINT_OUTSIDE_OF_PLANK_TOLERANCE;
float Config::ROBOT_TIME_DRIFT;
bool Config::PARTIAL_OBSERVABILITY;
int Config::NUMBER_OF_ROBOTS;
int Config::NUMBER_OF_OBSTACLES;
int Config::NUM_PLANK_POINTS;
int Config::PLANK_POINT_SHRINKAGE;
int Config::GRID_BOUNDS_X;
int Config::GRID_BOUNDS_Y;
int Config::NUM_PLANK_ITERATIONS;

void Config::loadParams() {
    // if(!ros::isInitialized()) {
    //     throw ROSNotInitializedException();
    // }

    ros::NodeHandle n("~");
    auto getDoubleParam = [&](const std::string& name, double& var, double min, double max) {
        double temp;
        if(!n.getParam(name, temp) || temp < min || temp > max) {
            std::string warn_msg = "Load param failed: ";
            warn_msg += name;
            warn_msg += ", using ";
            warn_msg += std::to_string(var);
            missing_param_set_.insert(name);
            ROS_WARN("%s", warn_msg.c_str());
        } else {
            var = temp;
        }
    };

    auto getFloatParam = [&](const std::string& name, float& var, float min, float max) {
        float temp;
        if(!n.getParam(name, temp) || temp < min || temp > max) {
            std::string warn_msg = "Load param failed: ";
            warn_msg += name;
            warn_msg += ", using ";
            warn_msg += std::to_string(var);
            missing_param_set_.insert(name);
            ROS_WARN("%s", warn_msg.c_str());
        } else {
            var = temp;
        }
    };

    auto getStringParam = [&](const std::string& name, std::string& var) {
        if(!n.getParam(name, var)) {
            std::string warn_msg = "Load param failed: ";
            warn_msg += name;
            warn_msg += ", using ";
            warn_msg += var;
            missing_param_set_.insert(name);
            ROS_WARN("%s", warn_msg.c_str());
        }   
    };

    auto getIntParam = [&](const std::string& name, int& var, int min, int max) {
        int temp;
        if(!n.getParam(name, temp) || temp < min || temp > max) {
            std::string warn_msg = "Load param failed: ";
            warn_msg += name;
            warn_msg += ", using ";
            warn_msg += std::to_string(var);
            missing_param_set_.insert(name);
            ROS_WARN("%s", warn_msg.c_str());
        } else {
            var = temp;
        }
    };

    auto getBoolParam = [&](const std::string& name, bool& var) {
        if(!n.getParam(name, var)) {
            std::string warn_msg = "Load param failed: ";
            warn_msg += name;
            warn_msg += ", using ";
            warn_msg += std::to_string(var);
            missing_param_set_.insert(name);
            ROS_WARN("%s", warn_msg.c_str());
        }
    };

    getStringParam("control_fsm_action_server",  CONTROL_FSM_ACTION_SERVER);
    getStringParam("ai_sim_observation_topic", AI_SIM_OBSERVATION_TOPIC);
    getFloatParam("total_competition_time", TOTAL_COMPETITION_TIME, 0.01, 610.0);
    getFloatParam("ros_rate_perception_control", ROS_RATE_PERCEPTION_CONTROL, 1.0, 400.0);
    getFloatParam("ros_rate_planning", ROS_RATE_PLANNING, 1.0, 400.0);
    getFloatParam("robot_reverse_interval", ROBOT_REVERSE_INTERVAL, 0.01, 40.0);
    getFloatParam("robot_turn_time", ROBOT_TURN_TIME, 0.01, 5.0);
    getFloatParam("robot_speed", ROBOT_SPEED, 0.01, 10.0);
    getFloatParam("land_time", LAND_TIME, 0.01, 10);
    getFloatParam("max_ground_time", MAX_GROUND_TIME, 0.1, 30.0);
    getFloatParam("drone_speed", DRONE_SPEED, 0.01, 10.0);
    getFloatParam("similarity_threshold", SIMILARITY_THRESHOLD, 0.01, 10.0);
    getFloatParam("maxdist_drone_to_point", MAXDIST_DRONE_TO_POINT, 0.01, 10.0);
    getFloatParam("maxdist_robot_to_point", MAXDIST_ROBOT_TO_POINT, 0.01, 10.0);
    getFloatParam("maxdist_actionpoints", MAXDIST_ACTIONPOINTS, 0.01, 30.0);
    getFloatParam("initial_plank_reward", INITIAL_PLANK_REWARD, -10000000.0, 10000000.0);
    getFloatParam("action_edge_buffer", ACTION_EDGE_BUFFER, 0.0, 10.0);
    getFloatParam("reliable_data_height", RELIABLE_DATA_HEIGHT, 0.0, 3.0);
    getFloatParam("point_outside_of_plank_tolerance", POINT_OUTSIDE_OF_PLANK_TOLERANCE, 0.01, 5.0);
    getFloatParam("robot_time_drift", ROBOT_TIME_DRIFT, 0.0, 10.0);
    getBoolParam("partial_observability", PARTIAL_OBSERVABILITY);
    getIntParam("number_of_robots", NUMBER_OF_ROBOTS, 1, 10);
    getIntParam("number_of_obstacles", NUMBER_OF_OBSTACLES, 1, 4);
    getIntParam("num_plank_points", NUM_PLANK_POINTS, 1, 20);
    getIntParam("plank_point_shrinkage", PLANK_POINT_SHRINKAGE, 0, 5);
    getIntParam("grid_bounds_x", GRID_BOUNDS_X, 1, 30);
    getIntParam("grid_bounds_y", GRID_BOUNDS_Y, 1, 30);
    getIntParam("num_plank_iterations", NUM_PLANK_ITERATIONS, 1, 20);
}