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
    std::cout << "z-pos from stateHandler: " << this->observation.getDrone().getPosition().z << std::endl;
    action_t action = empty_action;
    switch(this->state_){
        case no_input_data:
            this->noInputDataState();
            break;

        case idle:
            this->idleState();
            break;

        case positioning:
            action = this->positioningState();
            break;

        case land_in_front:
            action = this->landInFrontState();
            break;

        case land_on_top:
            action = this->landOnTopState();
            break;

        case mission_complete:
            action = this->missionCompleteState();
            break;

        case no_visible_robots:
            action = this->noVisibleRobotsState();
            break;
    }

    return action;
}

void AIController::transitionTo(ai_state_t state) {
    std::cout << "transitioning to: ";
    this->state_ = state;
    this->prev_transition_timestamp = this->observation.getTimeStamp();
}


void AIController::noInputDataState(){
    printf("No Input data state\n");
	if(this->observation.getRobot(0).getPosition().x != 0){
        this->transitionTo(idle);
    }
    return;
}

void AIController::idleState(){
    printf("Idle state\n");
    this->planned_action_ = ai_.getBestGeneralAction(this->observation);
    
    if(this->planned_action_.type == no_command){
        // This is triggered when we do not have any targets
        int target_id = this->planned_action_.target;
        bool target_visibility = this->observation.getRobot(target_id).getVisibility();
        if (this->observation.anyRobotsVisible() == false) {
            this->transitionTo(no_visible_robots);
        }
        // else targets current plank is better than acting
        return;
    }

    if (this->observation.getTimeStamp() >= 11*60) {
        this->transitionTo(mission_complete);
        return;
    }
	
    this->transitionTo(positioning);
	return;
}

action_t AIController::positioningState() {
    printf("Positioning state\n");

    int target_id = this->planned_action_.target;
    Robot target = this->observation.getRobot(target_id);

    if(!target.isMoving()){
        return empty_action;
    }

    // Is the drone and the robot at the rendezvous point
    if( is_nearby(this->observation.getDrone().getPosition(), this->planned_action_.where_To_Act) &&
        is_nearby(this->planned_action_.where_To_Act, target.getPosition()))
    {

        if (this->planned_action_.type == land_on_top_of) {
            this->transitionTo(land_on_top);
        } else if (this->planned_action_.type == land_in_front_of) {
            this->transitionTo(land_in_front);
        } else {
            printf("Action is not defined in positionState.");
        }
        return empty_action;
    } else { // The drone or robot is too far from rendezvous point
        action_t updated_action = this->ai_.getBestAction(target);

        if(updated_action.type == no_command){
            // This is triggered when we do not have any targets
            if (this->observation.anyRobotsVisible() == false) {
                this->transitionTo(no_visible_robots);
                return empty_action;
            } else { // The current robot plank is at its best
                return updated_action;
            }
        }

        if(!similarity(updated_action, this->planned_action_)) {
            this->transitionTo(idle);
            return empty_action;
        }

        this->planned_action_ = updated_action;

        action_t fly_action = this->planned_action_;
        fly_action.type = search;

        return fly_action;
    }
}

action_t AIController::landOnTopState(){
    printf("Land on top state\n");
    this->transitionTo(idle);
    return this->planned_action_;
}

action_t AIController::landInFrontState(){ // lande tidligere, mindre drone hitbox i sim, stå kortere på bakken (går ikke inn i)
    printf("Land in front state\n");

    if(this->observation.getDrone().getPosition().z >= 0.1){ // hvis du flyr
        this->planned_action_.type = land_at_point;
        return this->planned_action_;
    }

    // antar dronen har landet, sjekker hvor lenge drone står på bakken

    else if (this->observation.getTimeStamp() - prev_transition_timestamp > 2.0) {
            this->planned_action_.type = take_off;
            this->transitionTo(idle);
            return this->planned_action_;

    }

    std::cout << "I have landed" << std::endl;
    std::cout << "timediff: " << (this->observation.getTimeStamp() - prev_transition_timestamp) << std::endl;

    //bumper: lytt til bumpers (for å se når vi treffer target), hvis vi venter lengre enn konst slutt å stå på bakken
    //sammenlikn predicted target intersect tidspunkt med faktisk intersect

    return empty_action;
}

action_t AIController::missionCompleteState(){
    printf("Mission complete state\n");

    point_t drone_pos = this->observation.getDrone().getPosition();
    action_t land_action;

    land_action.type = land_at_point;
    land_action.where_To_Act = drone_pos;

    return land_action;
}

action_t AIController::noVisibleRobotsState(){
    printf("No visible robots state\n");

    action_t search_Action = empty_action;

    point_t next_search_point = point_Zero;

    point_t pos = this->observation.getDrone().getPosition();
    float x = pos.x;
    float y = pos.y;

    // These should be global values
    bounds_t bounds = world.getBounds();

    point_t track_center = point_Zero;
    track_center.x = bounds.x_Max / 2.0;
    track_center.y = bounds.y_Max / 2.0;
    float padding = 5.0;

    // The drone flies in a triangle path in a clockwise order
    if (this->observation.getDrone().getDistanceToPoint(track_center) < 3) {
        next_search_point.x = padding;
        next_search_point.y = bounds.y_Max - padding;
    } else if (x > track_center.x && y > track_center.y) { // fra 1. til 2. kvadr
        next_search_point.x = track_center.x;
        next_search_point.y = track_center.y;
    } else if (x > track_center.x && y < track_center.y) { // 2. til mid
        next_search_point.x = track_center.x;
        next_search_point.y = track_center.y;
    } else if (x <= track_center.x && y <= track_center.y) { // 3. til mid
        next_search_point.x = track_center.x;
        next_search_point.y = track_center.y;
    } else if (x < track_center.x && y > track_center.y) { // 4. til 1.
        next_search_point.x = bounds.x_Max - padding;
        next_search_point.y = bounds.y_Max - padding;
    }

    search_Action.type = search;
    search_Action.where_To_Act = next_search_point;
    search_Action.reward = 0.0;
    this->transitionTo(idle);
    return search_Action;
}
