#pragma once

#include "state_interface.h"

///Runs preflight checks and transition to idle when ready
class NoInputState : public StateInterface {
public:
    NoInputState();
    void stateBegin(ControlFSM& fsm) override;
    void loopState(ControlFSM& fsm) override;
    void stateEnd(ControlFSM& fsm) override;
    std::string getStateName() const { return "NoInput";}
};
