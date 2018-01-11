#include "AI.h"
#include "sim.h"

class AccessToSim {
    private:
        sim_State state = {};
        sim_Observed_State observed_state;
        sim_Observed_State prev_obv_state;
        sim_Command cmd;
    public:
        AccessToSim(Observation observation);
        void step();
};
