#include "config.h"
// #include <ascend_msgs/ReloadConfig.h>

using planning::Config;

std::set<std::string> Config::missing_param_set_;

//===============  Global config params  =================

//--------------------  Topic nodes  ---------------------

std::string Config::CONTROL_FSM_ACTION_SERVER;
std::string Config::AI_SIM_OBSERVATION_TOPIC;
double TOTAL_COMPETITION_TIME;
double Config::ROS_RATE_PERCEPTION_CONTROL;
double Config::ROS_RATE_PLANNING;

//------------------  World variables  -------------------

double Config::TOTAL_COMPETITION_TIME;


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
    getDoubleParam("total_competition_time", TOTAL_COMPETITION_TIME, 0.0, 610.0);
    getDoubleParam("ros_rate_perception_control", ROS_RATE_PERCEPTION_CONTROL, 1.0, 400.0);
}