#pragma once

#include <iostream>
#include <ros/ros.h>
#include <set>

/** @page fsm_params FSM Params
 *  @brief Parameter page
 *
 *  Contains all fsm params
 */


namespace planning {
class Config;
class Config {
private:
    ///Node handler
    ros::NodeHandle nh_;
    ///Reload service
    ros::ServiceServer reload_config_service;
    static std::set<std::string> missing_param_set_;
    ///Shared instance ptr
    static std::unique_ptr<Config> shared_instance_p_;
    ///Constructor
    Config();


public:

    static std::string time_chatter;
    static std::string groundrobot_chatter;
    static std::string drone_chatter;
    static std::string control_action_server;

    static void loadParams();    
    ///Returns set of unloaded params
    static const std::set<std::string>& getMissingParamSet() { return missing_param_set_; }
};
}