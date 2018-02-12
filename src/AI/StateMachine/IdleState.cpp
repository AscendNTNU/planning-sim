#include "IdleState.h"
#include "../structs.h"
#include "PlanningFSM.h"

void IdleState::stateBegin(PlanningFSM &fsm) {

}

void IdleState::loopState(PlanningFSM& fsm) {
	fsm.current_action_ = empty_action;
	fsm.planned_action_ = empty_action;
    action_t action = fsm.ai_.getBestGeneralAction(fsm.observation);
    
    if(action.type != no_Command){
		fsm.planned_action_ = action;
	    fsm.transitionTo(PlanningFSM::POSITIONING_STATE, this);
	}
	return;
}

void IdleState::stateEnd(PlanningFSM& fsm) {}
