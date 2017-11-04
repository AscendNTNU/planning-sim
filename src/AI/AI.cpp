#include "AI.h"
#include "Robot.h"
#include <typeinfo>
AI::AI() {
    this->state = State();
}

std::stack<action_t> AI::getBestGeneralActionStack(int num_Robots) {
    Robot target_ = chooseTarget(10);
    return getBestActionStack(target_);
}

std::stack<action_t> AI::getBestActionStack(Robot target) {
    std::stack<action_t> action_Stack_;

    action_t best_Action_ = chooseAction(target);
    action_Stack_.push(best_Action_);

    // If search is not already spesified, add a search (goto) action to complete the next action
    if (best_Action_.type != search) {
        action_t search_Action_ = best_Action_;
        search_Action_.type = search;
        action_Stack_.push(search_Action_);
    }

    return action_Stack_;
}

Robot AI::chooseTarget(int num_Robots) {
    Robot robot_;
    float best_reward_ = -1000000;

    // Return an invalid robot if none was assigned
    Robot target_ = Robot(-1);

    for (int i = 0; i < num_Robots; i++) {
        robot_ = this->state.robots[i];

        if (robot_.getIndex() != -1 && robot_.getVisibility()) {

            if (robot_.current_Plank.getReward() > best_reward_ && !robot_.current_Plank.willExitGreen()) {
                best_reward_ = robot_.current_Plank.getReward();
                target_ = robot_;
            }
        }
    }

    return target_;
}

action_t AI::chooseAction(Robot target) {

    // // Temporary max rewarded action
    action_t best_Action_ = action_Empty;

    // best_Action_.where_To_Act.travel_Time = interception.travel_Time;

    // Check if we have a visible target
    if (target.getIndex() != -1) {
        action_t step_Action_;

        for (int i = 1; i < target.current_Plank.getNumPlankPoints() - 1; i++) {
            std::cout << "Plank point " << i << ": " << target.current_Plank.getPoint(i).point.x << ", " << target.current_Plank.getPoint(i).point.y << std::endl;

            step_Action_ = getBestActionAtPosition(target.getOrientation(), target.current_Plank.getPoint(i));

            if (step_Action_.reward > best_Action_.reward) {
                best_Action_ = step_Action_;
            }
        }

        best_Action_.target = target.getIndex();
    }
    // If no target is visible, we do a patrol round
    else {
        action_t search_Action_ = action_Empty;

        point_t nextSearchPoint_ = point_Zero;

        point_t pos_ = this->state.drone.getPosition();
        float x_ = pos_.x;
        float y_ = pos_.y;
        float trackWidth_ = 20;
        float trackHeight_ = 20;
        float trackCenterX_ = trackWidth_ / 2;
        float trackCenterY_ = trackHeight_ / 2;
        float padding_ = 3;

        // The drone flies in a square path in a clockwise order
        if (x_ > trackCenterX_ && y_ > trackCenterY_) {
            nextSearchPoint_.x = trackWidth_ - padding_;
            nextSearchPoint_.y = padding_;
        } else if (x_ > trackCenterX_ && y_ < trackCenterY_) {
            nextSearchPoint_.x = padding_;
            nextSearchPoint_.y = padding_;
        } else if (x_ <= trackCenterX_ && y_ <= trackCenterY_) {
            nextSearchPoint_.x = padding_;
            nextSearchPoint_.y = trackHeight_ - padding_;
        } else if (x_ < trackCenterX_ && y_ > trackCenterY_) {
            nextSearchPoint_.x = trackWidth_ - padding_;
            nextSearchPoint_.y = trackHeight_ - padding_;
        }

        search_Action_.type = search;
        search_Action_.where_To_Act = nextSearchPoint_;
        search_Action_.target = 0;

        best_Action_ = search_Action_;
    }

    return best_Action_;
}

action_t AI::getBestActionAtPosition(float target_orientation, plank_point_t position) {
    int num_Iterations_ = 5; // Number of iterations when summing along a plank
    action_t action_;
    action_.where_To_Act = position.point;
    // float time_After_Turn_Start_ = fmod(this->state.getTimeStamp() + position.travel_Time + time_after_interception, 20);

    Plank plank_On_Top_ = Plank(position.point, fmod(target_orientation + (MATH_PI/4), 2*MATH_PI), 
                                    position.time_since_start_turn);
    Plank plank_In_Front_ = Plank(position.point, fmod(target_orientation + MATH_PI, 2*MATH_PI), 
                                    position.time_since_start_turn);

    return actionWithMaxReward(plank_On_Top_.getReward(), plank_In_Front_.getReward(), action_);
}

action_t AI::actionWithMaxReward(float reward_On_Top, float reward_In_Front, action_t action) {

    if (reward_On_Top > reward_In_Front) {
        action.type = land_On_Top_Of;
        action.reward = reward_On_Top;
    } else {
        action.type = land_In_Front_Of;
        action.reward = reward_In_Front;
    }

    return action;
}

bool AI::update(observation_t observation,float elapsed_time) {
    return this->state.updateState(observation, elapsed_time);
}

bool AI::updateRobot(observation_t observation,float elapsed_time) {
    return this->state.updateRobotState(observation, elapsed_time);
}

bool AI::updateDrone(observation_t observation,float elapsed_time) {
    return this->state.updateDroneState(observation, elapsed_time);
}
