#include "AI.h"
#include "sim.h"

class AccessToSim {
    private:
        sim_State state;
        bool hasCollision;
    public:
        AccessToSim(Observation observation);
        AccessToSim* step(sim_Command cmd);
        AccessToSim* stepNoCommand();
        sim_State getState();
        Observation getObservation();
        observation_t getObservationStruct();
        Drone getDrone();
        std::array<Robot, 10> getRobots();
        std::array<Robot, 4> getObstacles();
        bool getCollision();
};
