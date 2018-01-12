#include "AI.h"
#include "sim.h"

class AccessToSim {
    private:
        sim_State state;
        bool hasCollision;
    public:
        AccessToSim(Observation observation);
        AccessToSim* step();
        sim_State getState();
        Observation getObservation();
        bool getCollision();
};
