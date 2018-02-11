#pragma once

#include "StateInterface.h"

///Runs preflight checks and transition to idle when ready
class BeginState : public StateInterface {
public:
    BeginState();
    void stateBegin(PlanningFSM& fsm) override;
    void loopState(PlanningFSM& fsm) override;
    void stateEnd(PlanningFSM& fsm) override;
    std::string getStateName() const { return "NoInput";}
};
