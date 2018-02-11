#pragma once

#include "StateInterface.h"

///Runs preflight checks and transition to idle when ready
class MissionCompleteState : public StateInterface {
public:
    MissionCompleteState();
    void stateBegin(PlanningFSM& fsm) override;
    void loopState(PlanningFSM& fsm) override;
    void stateEnd(PlanningFSM& fsm) override;
    std::string getStateName() const { return "NoInput";}
};
