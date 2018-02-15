#pragma once

#include "StateInterface.h"


class IdleState : public StateInterface {
public:
    //IdleState();
    void stateBegin(PlanningFSM& fsm) override;
    void loopState(PlanningFSM& fsm) override;
    void stateEnd(PlanningFSM& fsm) override;
    std::string getStateName() const { return "IdleState";}
};