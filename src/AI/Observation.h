#pragma once
#include "Drone.h"
#include "Robot.h"
#include <array>

class Observation{
private:
    float time_Stamp;
    std::array<Robot,10> robots;
    std::array<Robot,4> obstacles;
    Drone drone;
    float state_value;

public:

    Observation();

    Drone getDrone();

    Robot getRobot(int index);
    std::array<Robot,10> getRobots();

    Robot getObstacle(int index);
    std::array<Robot,4> getObstacles();

    float getTimeStamp();

    float getStateValue();

    bool update(observation_t observation, float elapsed_time);
    bool updateDrone(observation_t observation, float elapsed_time);
    bool updateRobot(observation_t observation, float elapsed_time);
};
