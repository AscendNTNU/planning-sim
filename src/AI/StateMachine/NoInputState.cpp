#include "NoInputState.h"
#include "PlanningFSM.h"

NoInputState::NoInputState(): StateInterface::StateInterface() {}

void NoInputState::stateBegin(PlanningFSM &fsm) {
    
}

void NoInputState::loopState(PlanningFSM& fsm) {
	if(fsm.observation.getRobot(0).getPosition().x != 0){
    	fsm.transitionTo(PlanningFSM::IDLE_STATE, this);
    }
    return;
}

void NoInputState::stateEnd(PlanningFSM& fsm) {

}
