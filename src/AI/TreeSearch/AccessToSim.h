#pragma once

#include "../Observation.h"
#include "sim.h"

class AccessToSim {
    private:
        sim_State state;
        bool hasCollision;
    public:
        AccessToSim(Observation observation);

        bool simulateActionSequence(action_t action, float time_limit);
        bool simulateFlyToAction(action_t action, float time_limit);
        bool simulateRobotAction(action_t action, float time_limit);
        bool simulateWaitForRobot(action_t action, float time_limit);
        bool simulateAction(action_t action, float time_limit);

        Observation stepNoCommand();
        Observation getObservation();
        observation_t getObservationStruct();

        Drone getDrone();
        Robot getRobot(int i);

        std::array<Robot, 10> getRobots();
        std::array<Robot, 4> getObstacles();

        bool getCollision();

        sim_Command convertToSimAction(action_t action_struct);
};