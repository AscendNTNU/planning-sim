/**
@class AI
@brief AI class

This class handles all AI functions. This includes the choice of target and the choice of target.
*/

#pragma once
#include "TreeSearch/TreeSearch.h"
#include <stack>

class AI{

public:

    /**
    @brief Returns best target based on current planks.
    @param num_Robots Number of robots in game.
    @return The robot with best current plank.
    Loops through the robots in current state to find the robot with the highest
    value plank. Returns this robot if found, otherwise returns an empty Robot.
    */
    Robot chooseTarget(std::array<Robot,10> robots, Drone drone);


    /**
    @brief Returns best action along the plank of a target robot
    @param target Target robot.
    @return The best action.
    Given a robot, returns the action from different points on the plank that gives best value.
    If none found, returns an empty action, ie at (0,0)
    */
    action_t chooseAction(Robot target, Drone drone);

    /**
    @brief Gets the best action for a robot at a given position and time
    @param target Target robot.
    @param position Position for calculation.
    @param time_after_interception How long to wait after the drone arrives at position
    @return The best action.
    */


    action_t squareSearch(Robot target, Drone drone);

    action_t getBestActionAtPosition(float target_orientation, plank_point_t position);

    /**
    @brief Returns best action given rewards
    @param reward_On_Top Reward of resultant plank from landing on top of the robot
    @param reward_In_Front Reward of resultant plank from landing in front of the robot
    @return The best action.
    Compares landing ontop to landing in front and returns an action_t with the action of highest reward
    */
    action_t actionWithMaxReward(float reward_On_Top, float reward_In_Front, action_t action);

    /**
    @brief Returns action stack for the best action over all robots.
    @return Best possible action stack.
    Checks the best action for each robot and returns the action stack for performing the action with best reward/
    */
    action_t getBestGeneralAction(Observation observation);

    /**
    @brief Returns action stack for the best action for a target robot.
    @param num_Robots Number of robots in game.
    @return Best possible action stack.
    Checks the best action for a target robot robot and returns the action stack for performing the action with best reward.
    */
    action_t getBestAction(Robot target, Observation observation);
};