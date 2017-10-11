#include "AI.h"

AI::AI(){
    this->state = new State();
}

std::stack<action_t> AI::getBestGeneralActionStack(){
    Robot* target = chooseTarget(10);
    return getBestActionStack(target);
}

std::stack<action_t> AI::getBestActionStack(Robot* target){

    std::stack<action_t> action_Stack;

    if(target->current_Plank->getReward() == -200000){
        action_Stack.push(action_Empty);
        return action_Stack;
    }

    action_t best_Action = chooseAction(target);

    if(best_Action.reward == -200000){
        action_Stack.push(action_Empty);
        return action_Stack;
    }

    action_t search_Action = best_Action;
    search_Action.type = search;

    action_Stack.push(best_Action);
    action_Stack.push(search_Action);
    return action_Stack;
}

Robot* AI::chooseTarget(int num_Robots){
    float max_reward = -200000;
    float reward = 0;
	bool robotChosen = false;
    Robot* target = NULL;

    for(int i = 0; i < num_Robots; i++){
        Robot* robot = this->state->robots[i];
		if(robot->current_Plank->willExitGreen()) {
			continue;
		}
        reward = robot->current_Plank->getReward();
        if(reward > max_reward){
            max_reward = reward;
            target = robot;
			robotChosen = true;
        }
    }
	if(!robotChosen) {
        Robot* dummy = new Robot(-1);
        target = dummy;
	}
    return target;
}

action_t AI::chooseAction(Robot* target){

    point_t interception = this->state->drone->getInterceptPoint(target);
    point_t step_Point = {
        .x = interception.x, 
        .y = interception.y
    };

    float time_after_interception = 0;

    float n = 10;
    float step_size = target->current_Plank->getLength()/n;
    float angle = target->current_Plank->getAngle();
    float step_x = step_size*cos(angle);
    float step_y = step_size*sin(angle);

    // Temporary max rewarded action
    action_t best_Action = action_Empty;
    
    best_Action.where_To_Act.travel_Time = interception.travel_Time;
    
    action_t step_Action;
    bool backwards = false;
    int i = 1;
    int counter = 0;
    while (1) {
        counter++;
        if (target->current_Plank->pointIsOutsideOfPlank(step_Point)) {
            if (backwards) {
                best_Action.target = target->getIndex();
                return best_Action;
            } else {
                i = n+1;
                backwards = true;
                angle += MATH_PI;
            }
        }
		else {
			step_Action = getBestActionAtPosition(target, step_Point, time_after_interception);
			if (step_Action.reward > best_Action.reward) {
				best_Action = step_Action;
				best_Action.when_To_Act = time_after_interception;// + interception.travel_Time; Denne skal kanskje være globalt tispunkt etter start?
			}
		}
        
		
		if (backwards) {
            step_Point.x = step_Point.x-step_x;
            step_Point.y = step_Point.y-step_y;
            i -= 1;
        } else {
            step_Point.x = step_Point.x+step_x; 
            step_Point.y = step_Point.y+step_y;
            i += 1;
        }
        time_after_interception = time_after_interception + (step_size)/target->getSpeed();

    }
    best_Action.target = target->getIndex();
    return best_Action;
}

action_t AI::getBestActionAtPosition(Robot* target, point_t position, float time_after_interception) {

    int num_Iterations = 5; // Number of iterations when summing along a plank
    action_t action;
    action.where_To_Act = position;
    float time_After_Turn_Start = fmod(this->state->getTimeStamp() + position.travel_Time + time_after_interception, 20); //This can be calculated in a better way

    Plank* plank_On_Top = new Plank(position, fmod(target->getOrientation() + (MATH_PI/4), 2*MATH_PI), 
                                    time_After_Turn_Start, num_Iterations);
    Plank* plank_In_Front = new Plank(position, fmod(target->getOrientation() + MATH_PI, 2*MATH_PI), 
                                    time_After_Turn_Start, num_Iterations);

    return actionWithMaxReward(plank_On_Top->getReward(), plank_In_Front->getReward(), action);
}

action_t AI::actionWithMaxReward(float reward_On_Top, float reward_In_Front, action_t action){

    if(reward_On_Top > reward_In_Front){
        action.type = land_On_Top_Of;
        action.reward = reward_On_Top;
    } else if (reward_On_Top < reward_In_Front){
        action.type = land_In_Front_Of;
        action.reward = reward_In_Front;
    } else if (reward_On_Top == reward_In_Front){
        // Return in front because it is easier?
        action.type = land_In_Front_Of;
        action.reward = reward_In_Front;
    } else {
        // Will it ever get here?
        action.type = no_Command;
        action.reward = 0;
    }
    return action;
}

bool AI::update(observation_t observation,float elapsed_time) {
    return this->state->updateState(observation, elapsed_time);
}

bool AI::updateRobot(observation_t observation,float elapsed_time){
    return this->state->updateRobotState(observation, elapsed_time);
}

bool AI::updateDrone(observation_t observation,float elapsed_time){
    return this->state->updateDroneState(observation, elapsed_time);
}