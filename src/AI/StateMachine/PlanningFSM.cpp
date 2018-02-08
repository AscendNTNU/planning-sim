#include "PlanninFsm.h"
#include <ros/ros.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

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
        getState()->stateEnd(*this, event);
        //Set the current state pointer
        state_vault_.current_state_p_ = &state;
        //Pass event to new current state
        getState()->stateBegin(*this, event);
        //Notify state has changed
        on_state_changed_();
    } else {
        control::handleErrorMsg("Transition request made by not active state");
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
        control::handleCriticalMsg(e.what());
        RequestEvent abort_event(RequestType::ABORT);
        transitionTo(BLIND_HOVER_STATE, getState(), abort_event);
    }
}

PlanningFSM::PlanningFSM() {
    //Set starting state
    state_vault_.current_state_p_ = &BEGIN_STATE;

    //Initialize all states
    this->initStates();

    //Subscribe to neccesary topics
    std::string& stateTopic = control::Config::mavros_state_changed_topic;
    subscribers_.mavros_state_changed_sub = node_handler_.subscribe(stateTopic, 1, &PlanningFSM::mavrosStateChangedCB, this);
}
