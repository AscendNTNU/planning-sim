#include "PrestartState.h"
#include "PlanningFSM.h"

PrestartState::PrestartState() {

}

void PrestartState::stateBegin(PlanningFSM &fsm) {
    
}

void PrestartState::loopState(PlanningFSM& fsm) {
	if(fsm.observation.getTimeStamp() > 0){
		fsm.transitionTo(PlanningFSM::NO_INPUT_STATE, this);
	}
}

void PrestartState::stateEnd(PlanningFSM& fsm) {

}
