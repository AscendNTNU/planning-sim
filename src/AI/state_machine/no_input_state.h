#pragma once

#include "state_interface.h"

///Runs preflight checks and transition to idle when ready
class NoInputState : public StateInterface {
public:
    NoInputState();
    void stateBegin(AIFSM& fsm) override;
    std::string getStateName() const override { return "No Input"; }
    void handleManual(AIFSM &fsm) override;
};
