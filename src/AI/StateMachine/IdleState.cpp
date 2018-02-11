#include "IdleState.h"
#include "../structs.h"
#include "PlanningFSM.h"

void IdleState::stateBegin(PlanningFSM &fsm) {
	fsm.current_action_ = empty_action;
	fsm.planned_action_ = empty_action;
}

void IdleState::loopState(PlanningFSM& fsm) {
    action_t action = fsm.ai_.getBestGeneralAction(fsm.observation);
    
    if(action.type == no_Command){
        return;
    }
	
	fsm.planned_action_ = action;
    fsm.transitionTo(PlanningFSM::POSITIONING_STATE, this);
	return;
}

void IdleState::stateEnd(PlanningFSM& fsm) {}
