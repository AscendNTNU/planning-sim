#include "AI.h"
#include <typeinfo>
AI::AI(){
    this->state = State();
}

std::stack<action_t> AI::getBestGeneralActionStack(int num_Robots){
    Robot target = chooseTarget(10);
    return getBestActionStack(target);
}

std::stack<action_t> AI::getBestActionStack(Robot target){
    
    std::stack<action_t> action_Stack;

    action_t best_Action = chooseAction(target);

    action_t search_Action = best_Action;
    search_Action.type = search;

    action_Stack.push(best_Action);
    action_Stack.push(search_Action);

    return action_Stack;
}

Robot AI::chooseTarget(int num_Robots){
    Robot robot;
    Robot target;
    float best_reward = -1000000;
    for(int i = 0; i < num_Robots; i++){
        robot = this->state.robots[i];
        if(robot.getIndex() != -1){
            if(robot.current_Plank.getReward() > best_reward && !robot.current_Plank.willExitGreen()){
                best_reward = robot.current_Plank.getReward();
                target = robot;
            }
        }
    }
    return target;
}

action_t AI::chooseAction(Robot target){

    // // Temporary max rewarded action
    action_t best_Action = action_Empty;
    
    // best_Action.where_To_Act.travel_Time = interception.travel_Time;
    
    action_t step_Action;

    for(int i = 1; i < target.current_Plank.getNumPlankPoints()-1; i++) {
      std::cout << "Plank point " << i << ": " << target.current_Plank.getPoint(i).point.x << ", " << target.current_Plank.getPoint(i).point.y << std::endl;

      step_Action = getBestActionAtPosition(target.getOrientation(), target.current_Plank.getPoint(i));
      if (step_Action.reward > best_Action.reward) {
          best_Action = step_Action;
      }
    }
    best_Action.target = target.getIndex();
    return best_Action;
}

action_t AI::getBestActionAtPosition(float target_orientation, plank_point_t position) {
    int num_Iterations = 5; // Number of iterations when summing along a plank
    action_t action;
    action.where_To_Act = position.point;
    // float time_After_Turn_Start = fmod(this->state.getTimeStamp() + position.travel_Time + time_after_interception, 20);

    Plank plank_On_Top = Plank(position.point, fmod(target_orientation + (MATH_PI/4), 2*MATH_PI), 
                                    position.time_since_start_turn);
    Plank plank_In_Front = Plank(position.point, fmod(target_orientation + MATH_PI, 2*MATH_PI), 
                                    position.time_since_start_turn);

    return actionWithMaxReward(plank_On_Top.getReward(), plank_In_Front.getReward(), action);
}

action_t AI::actionWithMaxReward(float reward_On_Top, float reward_In_Front, action_t action){

    if(reward_On_Top > reward_In_Front){
        action.type = land_On_Top_Of;
        action.reward = reward_On_Top;
    } else{
        action.type = land_In_Front_Of;
        action.reward = reward_In_Front;
    }
    return action;
}

bool AI::update(observation_t observation,float elapsed_time) {
    return this->state.updateState(observation, elapsed_time);
}

bool AI::updateRobot(observation_t observation,float elapsed_time){
    return this->state.updateRobotState(observation, elapsed_time);
}

bool AI::updateDrone(observation_t observation,float elapsed_time){
    return this->state.updateDroneState(observation, elapsed_time);
}