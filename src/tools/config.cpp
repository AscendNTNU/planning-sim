#include "config.h"
// #include <ascend_msgs/ReloadConfig.h>

using planning::Config;

std::set<std::string> Config::missing_param_set_;

//Global config params

std::string Config::time_chatter;
std::string Config::groundrobot_chatter;
std::string Config::drone_chatter;
std::string Config::control_action_server;

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

    getStringParam("time_chatter_topic", time_chatter);
    getStringParam("ground_robot_topic",  groundrobot_chatter);
    getStringParam("drone_topic",  drone_chatter);
    getStringParam("control_action_server",  control_action_server);
}
