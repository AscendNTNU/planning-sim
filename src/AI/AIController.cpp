#include "AIController.h"

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

AIController::AIController(){
	this->ai_ = AI();
	this->state_ = no_input_data;
    this->observation = Observation();
}

action_t AIController::stateHandler(){
    action_t action = empty_action;
    switch(this->state_){
        case no_input_data:
            this->noInputDataState();
            break;

        case idle:
            this->idleState();
            break;

        case fly_to:
            action = this->flyToState();
            break;
        
        case waiting:
            this->waitingState();
            break;
        
        case perform_action:
            action = this->performActionState();
            break;
    }
    return action;
 }

void AIController::noInputDataState(){
    printf("No Input data state\n");
	if(this->observation.getRobot(0).getPosition().x != 0){
    	this->state_ = idle;
    }
    return;
 }

void AIController::idleState(){
    printf("Idle state\n");

    std::queue<action_t> current_action_queue_ = ai_.getBestGeneralActionQueue(this->observation);

    if (current_action_queue_.size() == 0) {
        this->state_ = idle;
    }
    else {
        this->current_action_ = current_action_queue_.front();
        this->state_ = fly_to;
    }
    
	return;
}

action_t AIController::flyToState(){
    printf("fly to state\n");	
	if(this->current_action_.type==search){
		this->state_ = idle;
		return this->current_action_;
	}

	action_t fly_action = this->current_action_;
	fly_action.type = search;
	this->state_ = waiting;
	return fly_action;
}

void AIController::waitingState(){
    printf("waiting state\n");
    int target_id = this->current_action_.target;
    Robot target = this->observation.getRobot(target_id);

    if(!target.isMoving()){
        this->state_ = waiting;
        return;
    }

	if(is_nearby(this->current_action_.where_To_Act, target.getPosition())){
		this->state_ = perform_action;
		return;
	}

    
	action_t updated_action = this->ai_.getBestAction(target, this->observation);

    /*
    if(!similarity(updated_action, this->current_action_)) {
    	this->state_ = idle;
    	return;
    }
    */

    bool intersection = false;
    point_t robotpos;
    for (int i = 0; i < 10; i++) { // currently 10 plank points
        robotpos = target.getCurrentPlank().getPoint(i).point;
        if (pointsWithinThreshold(robotpos, this->current_action_.where_To_Act, SIMILARITY_THRESHOLD)) {
            intersection = true;
        }
    }

    if (!intersection) {
        this->state_ = idle;
        return;
    }

    this->current_action_ = updated_action;
    this->state_ = fly_to;
    return;
}

action_t AIController::performActionState(){
    printf("action state\n");
	this->state_ = idle;
	return this->current_action_; 
}