#include "LandInFrontState.h"
#include "PlanningFSM.h"

LandInFrontState::LandInFrontState() {

}

void LandInFrontState::stateBegin(PlanningFSM &fsm) {   
}

void LandInFrontState::loopState(PlanningFSM& fsm) {
	fsm.current_action_  = fsm.planned_action_;
	fsm.transitionTo(PlanningFSM::IDLE_STATE, this);
}

void LandInFrontState::stateEnd(PlanningFSM& fsm) {
}
