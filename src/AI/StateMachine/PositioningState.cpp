#include "PositioningState.h"
#include "PlanningFSM.h"

bool is_nearby(point_t current_Where_To_Act, point_t target) {
      double x1 = current_Where_To_Act.x;
      double y1 = current_Where_To_Act.y;
      double x2 = target.x;
      double y2 = target.y;
      
      double dist = pow(pow(x2-x1,2) + pow(y2-y1,2), .5);
      return dist < SIMILARITY_THRESHOLD;
}

float similarity(action_t action1 ,action_t action2) {
    if(is_nearby(action1.where_To_Act, action2.where_To_Act)) {
        return 1;
    }
    return 0;
}


PositioningState::PositioningState() {

}

void PositioningState::stateBegin(PlanningFSM &fsm) {
	if(fsm.planned_action_.type==search){
		fsm.current_action_ = fsm.planned_action_;
		fsm.transitionTo(PlanningFSM::IDLE_STATE, this);
		return;
	}
}

void PositioningState::loopState(PlanningFSM& fsm) {
	

    int target_id = fsm.planned_action_.target;
    Robot target = fsm.observation.getRobot(target_id);

    //If target is not moving, wait til it starts moving
    if(!target.isMoving()){
		fsm.current_action_ = empty_action;
        return;
    }

    //If target is close enough for us to perform our planned action, do it!
	if(is_nearby(fsm.planned_action_.where_To_Act, target.getPosition())){
		if(fsm.planned_action_.type == land_On_Top_Of){
			fsm.transitionTo(PlanningFSM::LAND_ON_TOP_STATE, this);
		}
		else{
			fsm.transitionTo(PlanningFSM::LAND_IN_FRONT_STATE, this);
		}
		return;
	}

	//Get current best action on same target
	action_t updated_action = fsm.ai_.getBestAction(target, fsm.observation);

    if(!similarity(updated_action, fsm.planned_action_) &&
    		updated_action.type != no_Command) {
    	fsm.transitionTo(PlanningFSM::IDLE_STATE, this);
    	return;
    }

    else if(updated_action.type != no_Command){
	    fsm.planned_action_ = updated_action;
    }

	action_t fly_action = fsm.planned_action_;
	fly_action.type = search;
	fsm.current_action_ = fly_action;
}

void PositioningState::stateEnd(PlanningFSM& fsm) {

}