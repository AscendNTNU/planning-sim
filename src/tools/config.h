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
            static std::set<std::string> missing_param_set_;
            ///Constructor
            Config() = default;

        public:

            static std::string TIME_CHATTER;
            static std::string GROUNDROBOT_CHATTER;
            static std::string DRONE_CHATTER;
            static std::string CONTROL_ACTION_CHATTER;
            static double TOTAL_COMPETITION_TIME;

            static void loadParams();    
            ///Returns set of unloaded params
            static const std::set<std::string>& getMissingParamSet() { 
                return missing_param_set_; 
            }
    };
}