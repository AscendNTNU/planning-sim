#pragma once

#include "../Observation.h"
#include "sim.h"

class AccessToSim {
    private:
        sim_State state;
        bool hasCollision;
    public:
        AccessToSim(Observation observation);
        Observation simulateAction(sim_Command cmd);
        Observation step(sim_Command cmd);
        Observation stepNoCommand();
        Observation getState();
        Observation getObservation();
        observation_t getObservationStruct();
        Drone getDrone();
        std::array<Robot, 10> getRobots();
        std::array<Robot, 4> getObstacles();
        bool getCollision();
};
