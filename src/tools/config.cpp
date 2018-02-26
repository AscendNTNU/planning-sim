#include "control/tools/config.hpp"
#include <ascend_msgs/ReloadConfig.h>

using planning::Config;

std::set<std::string> Config::missing_param_set_;
std::unique_ptr<Config> Config::shared_instance_p_ = nullptr;

//Global config params
// std::string Config::mavros_local_vel_topic = "mavros/local_position/velocity";

void Config::loadParams() {
    if(!ros::isInitialized()) {
        throw ROSNotInitializedException();
    }
    if(shared_instance_p_ == nullptr) {
        shared_instance_p_ = std::unique_ptr<Config>(new Config);
    } 
    ros::NodeHandle n("~");
    auto getDoubleParam = [&](const std::string& name, double& var, double min, double max) {
        double temp;
        if(!n.getParam(name, temp) || temp < min || temp > max) {
            std::string warn_msg = "Load param failed: ";
            warn_msg += name;
            warn_msg += ", using ";
            warn_msg += std::to_string(var);
            control::handleWarnMsg(warn_msg);
            missing_param_set_.insert(name);
        } else {
            var = temp;
            control::handleInfoMsg("Param " + name + " loaded: " + std::to_string(var));
        }
    };

    auto getStringParam = [&](const std::string& name, std::string& var) {
        if(!n.getParam(name, var)) {
            std::string warn_msg = "Load param failed: ";
            warn_msg += name;
            warn_msg += ", using ";
            warn_msg += var;
            control::handleWarnMsg(warn_msg);
            missing_param_set_.insert(name);
        } else {
            control::handleInfoMsg("Param " + name + " loaded: " + var);
        }
    };

    auto getIntParam = [&](const std::string& name, int& var, int min, int max) {
        int temp;
        if(!n.getParam(name, temp) || temp < min || temp > max) {
            std::string warn_msg = "Load param failed: ";
            warn_msg += name;
            warn_msg += ", using ";
            warn_msg += std::to_string(var);
            control::handleWarnMsg(warn_msg);
            missing_param_set_.insert(name);
        } else {
            var = temp;
            control::handleInfoMsg("Param " + name + " loaded: " + std::to_string(var));
        }
    };

    auto getBoolParam = [&](const std::string& name, bool& var) {
        if(!n.getParam(name, var)) {
            std::string warn_msg = "Load param failed: ";
            warn_msg += name;
            warn_msg += ", using ";
            warn_msg += std::to_string(var);
            control::handleWarnMsg(warn_msg);
            missing_param_set_.insert(name);
        } else {
            control::handleInfoMsg("Param " + name + " loaded: " + std::to_string(var));
        }
    };

    getStringParam("time_chatter", time_chatter);
    getStringParam("groundrobot_chatter",  groundrobot_chatter);
    getStringParam("drone_chatter",  drone_chatter);
    getStringParam("control_action_server",  control_action_server);
}

using Request = ascend_msgs::ReloadConfig::Request;
using Response = ascend_msgs::ReloadConfig::Response;
bool reloadConfigCB(Request&, Response& resp) {
    control::Config::loadParams();
    control::handleInfoMsg("Config reloaded by service!");

    //Missing param set should be empty!
    for(auto& s : control::Config::getMissingParamSet()) {
        resp.missing_params.emplace_back(s);
    }
    return true;
}

control::Config::Config() {
    if(!ros::isInitialized()) {
        throw control::ROSNotInitializedException();
    }
    reload_config_service = nh_.advertiseService("/control_fsm_reload_config", reloadConfigCB);
}