#pragma once

#include "Robot.h"
#include "State.h"
#include "Plank.h"
#include <stack>

class AI{
public:
    AI();
    State* state;
    Robot* chooseTarget(int num_Robots);
    action_t chooseAction(Robot* target);

    action_t getBestActionAtPosition(Robot* target, point_t position, float time_after_interception);
    action_t actionWithMaxReward(float reward_On_Top, float reward_In_Front, action_t action);
    std::stack<action_t> getBestGeneralActionStack();
    std::stack<action_t> getBestActionStack(Robot* target_id);

    bool update(observation_t observation);
    bool updateDrone(observation_t observation);
    bool updateRobot(observation_t observation);
};