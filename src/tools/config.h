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
            static std::string CONTROL_FSM_ACTION_SERVER;
            static std::string AI_SIM_OBSERVATION_TOPIC;
            static double TOTAL_COMPETITION_TIME;
            
            static double ROS_RATE_PERCEPTION_CONTROL;
            static double ROS_RATE_PLANNING;
            static float ROBOT_REVERSE_INTERVAL;
            static double ROBOT_TURN_TIME;
            static double ROBOT_SPEED;
            static double LAND_TIME;
            static float MAX_GROUND_TIME;
            static double DRONE_SPEED;
            static float SIMILARITY_THRESHOLD;
            static float MAXDIST_DRONE_TO_POINT;
            static float MAXDIST_ROBOT_TO_POINT; 
            static float MAXDIST_ACTIONPOINTS;
            static bool PARTIAL_OBSERVABILITY;
            static float INITIAL_PLANK_REWARD;
            static int NUMBER_OF_ROBOTS;
            static int NUMBER_OF_OBSTACLES;
            static int NUM_PLANK_POINTS;
            static int PLANK_POINT_SHRINKAGE;
            static int GRID_BOUNDS_X;
            static int GRID_BOUNDS_Y;
            static float ACTION_EDGE_BUFFER;
            static float RELIABLE_DATA_HEIGHT;
            static int NUM_PLANK_ITERATIONS;
            static float POINT_OUTSIDE_OF_PLANK_TOLERANCE;
            static float ROBOT_TIME_DRIFT;

            static void loadParams();    
            ///Returns set of unloaded params
            static const std::set<std::string>& getMissingParamSet() { 
                return missing_param_set_; 
            }
    };
}