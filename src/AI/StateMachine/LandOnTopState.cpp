#include "LandOnTopState.h"
#include "PlanningFSM.h"

LandOnTopState::LandOnTopState() {

}

void LandOnTopState::stateBegin(PlanningFSM &fsm) {
    
}

void LandOnTopState::loopState(PlanningFSM& fsm) {
	fsm.current_action_  = fsm.planned_action_;
	fsm.transitionTo(PlanningFSM::IDLE_STATE, this);
}

void LandOnTopState::stateEnd(PlanningFSM& fsm) {

}
