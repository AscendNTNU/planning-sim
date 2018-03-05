#include "AI.h"
#include "Robot.h"
#include <array>

action_t AI::getBestGeneralAction(Observation observation) {
    Robot target = chooseTarget(observation.getRobots(), observation.getTimeStamp());
    if (target.getIndex() == -1) {
        return empty_action;
    }
    return getBestAction(target);
}

// This function is probably redundant
action_t AI::getBestAction(Robot target) {
    action_t best_Action = chooseAction(target);
    return best_Action;
}

Robot AI::chooseTarget(std::array<Robot,10> robots, float elapsed_time) {
    Robot robot;
    float best_reward = -1000000;

    // Return an invalid robot if none was assigned
    Robot target = Robot(-1);

    for (int i = 0; i < robots.size(); i++) {
        robot = robots[i];

        if (robot.current_Plank.willExitRed()) {
            continue;
        }

        if (robot.getVisibility() && robot.isMoving()) {

            if (robot.current_Plank.getReward() > best_reward && !robot.current_Plank.willExitGreen()) {
                best_reward = robot.current_Plank.getReward();
                target = robot;
            }
        }
    }

    return target;
}

//Alternative method for choosing target, this one uses the best drone position at intersection instead of the best plank.
// Robot AI::chooseTarget(std::array<Robot,10> robots, Drone drone) {
//     Robot robot;
//     point_t best_pos = point_Zero;
//     float best_reward = Plank().getReward();//world.getGridValue(best_pos.x, best_pos.y);

//     // Return an invalid robot if none was assigned
//     Robot target = Robot(-1);

//     for (int i = 0; i < robots.size(); i++) {
//         robot = robots[i];
//         if (robot.getIndex() != -1 && robot.getVisibility() && robot.isMoving()) {

//             // std::cout << "index" << robot.getIndex();
//             if(world.getGridValue(best_pos.x, best_pos.y) > best_reward && !robot.current_Plank.willExitGreen()){
//                 best_pos = drone.getInterceptPoint(robot);
//                 best_reward = world.getGridValue(best_pos.x, best_pos.y);
//                 target = robot;
//             }
//         }
//     }
//     return target;
// }

action_t AI::chooseAction(Robot target) {
    // // Temporary max rewarded action
    action_t best_Action = empty_action;

    // Check if current plank is good enough?

    // Check if we have a visible target
    if (target.getIndex() != -1) {
        best_Action.reward = target.getCurrentPlank().getReward();

        action_t step_Action;

        for (int i = 2; i < target.current_Plank.getNumPlankPoints() - 2; i++) {
            // std::cout << "Plank point " << i << ": " << target.current_Plank.getPoint(i).point.x << ", " << target.current_Plank.getPoint(i).point.y << std::endl;

            step_Action = getBestActionAtPosition(target.getOrientation(), target.current_Plank.getPoint(i));

            if (step_Action.reward > best_Action.reward) {
                best_Action = step_Action;
            }
        }

        best_Action.target = target.getIndex();
    }
    // If no target is visible, we return the empty action
    // This will trigger the search sequence
    return best_Action;
}

action_t AI::getBestActionAtPosition(float target_orientation, plank_point_t position) {
    int num_Iterations = 5; // Number of iterations when summing along a plank
    action_t action;
    action.where_To_Act = position.point;

    Plank plank_On_Top = Plank(position.point, fmod(target_orientation + 2*MATH_PI - (MATH_PI/4), 2*MATH_PI), 
                                    position.time_since_start_turn, ROBOT_TURN_TIME);
    Plank plank_In_Front = Plank(position.point, fmod(target_orientation + MATH_PI, 2*MATH_PI), 
                                    position.time_since_start_turn, ROBOT_TURN_TIME);

    return actionWithMaxReward(plank_On_Top.getReward(), plank_In_Front.getReward(), action);
}

action_t AI::actionWithMaxReward(float reward_On_Top, float reward_In_Front, action_t action) {

    if (reward_On_Top > reward_In_Front) {
        action.type = land_on_top_of;
        action.reward = reward_On_Top;
    } else {
        action.type = land_in_front_of;
        action.reward = reward_In_Front;
    }

    return action;
}