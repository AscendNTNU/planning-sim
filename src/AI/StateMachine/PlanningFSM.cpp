#include <assert.h>

#include "PlanningFSM.h"

BeginState PlanningFSM::BEGIN_STATE;
PrestartState PlanningFSM::PRESTART_STATE;
NoInputState PlanningFSM::NO_INPUT_STATE;
IdleState PlanningFSM::IDLE_STATE;
PositioningState PlanningFSM::POSITIONING_STATE;
SearchState PlanningFSM::SEARCH_STATE;
LandOnTopState PlanningFSM::LAND_ON_TOP_STATE;
LandInFrontState PlanningFSM::LAND_IN_FRONT_STATE;
MissionCompleteState PlanningFSM::MISSION_COMPLETE_STATE;

//Change the current running state - be carefull to only change into an allowed state
//Due to poor design, transitionTo has no strong nothrow guarantees - not exception safe!!
//Will lead to undefined behaviour if exception is thrown
void PlanningFSM::transitionTo(StateInterface& state, StateInterface* caller_p) {
    //Only current running state is allowed to change state
    if(getState() == caller_p) {
        //Run stateEnd on current running state before transitioning
        getState()->stateEnd(*this);
        //Set the current state pointer
        state_vault_.current_state_p_ = &state;
        //Pass event to new current state
        getState()->stateBegin(*this);
        //Notify state has changed
        on_state_changed_();
    } else {
        std::cout << "transitionTo error" << std::endl;
        return; //TODO
    }
}

//Runs state specific code on current state
void PlanningFSM::loopCurrentState(void) {
    try {
        assert(getState() != nullptr);
        getState()->loopState(*this);
    } catch(const std::exception& e) {
        //If exceptions aren't handled by states - notify and try to go to blind hover
        //Will lead to undefined behaviour- but still safer than nothing!
        std::cout << "loopCurrentState error" << std::endl;
    }
}

PlanningFSM::PlanningFSM() {

    this->current_action_ = empty_action;
    this->planned_action_ = empty_action;

    //Set starting state
    state_vault_.current_state_p_ = &BEGIN_STATE;

    //Initialize all states
    this->initStates();
}


void PlanningFSM::initStates() {
    //Only init once
    if(states_is_ready_) return; // FSM has been initalized
    for(auto it = StateInterface::cbegin(); it != StateInterface::cend(); it++) {
        (*it)->stateInit(*this);
    }
    states_is_ready_ = true;
}


action_t PlanningFSM::getCurrentAction(){
    return this->current_action_;
}

void PlanningFSM::updateObservation(Observation obs) {
    this->observation = obs;
}

Observation PlanningFSM::getObservation() {
    return this->observation;
}
