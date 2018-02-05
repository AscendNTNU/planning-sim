#include "AI.h"
#include <array>
#include <queue>

std::queue<action_t> AI::getBestGeneralActionQueue(Observation observation) {
    
    std::cout << "--building tree--" << std::endl;
    TreeSearch tree = TreeSearch(observation);
    std::cout << "--searching tree--" << std::endl;
    tree.DFSBestAction(tree.getRoot());
    Node best_node  = tree.getBestNode();
    std::cout << "--Is best node the root? " << best_node.isRoot() << std::endl;
    std::cout << "--returning action queue" << std::endl;
    std::queue<action_t> queue = tree.getActionQueue(best_node);
    std::cout << "--number of actions in action queue: " << queue.size() << std::endl;
    return queue;
    //Robot target = chooseTarget(observation.getRobots(),observation.getDrone());
    //return getBestAction(target, observation);
bool robotsAtTurnTime(float elapsed_time) {
    float rest = fmod(elapsed_time, 20); 
    if (rest < 3) {
        return true;
    }
    return false;
}

action_t AI::getBestGeneralAction(Observation observation) {
    Robot target = chooseTarget(observation.getRobots(),observation.getDrone());
    return getBestAction(target, observation);
}

action_t AI::getBestAction(Robot target, Observation observation) {
    if (robotsAtTurnTime(observation.getTimeStamp())){
        return empty_action;
    }
    action_t best_Action = chooseAction(target, observation.getDrone());
    return best_Action;
}
/*
Robot AI::chooseTarget(std::array<Robot,10> robots, Drone drone) {
    Robot robot;
    float best_reward = -1000000;

    // Return an invalid robot if none was assigned
    Robot target = Robot(-1);

    for (int i = 0; i < robots.size(); i++) {
        robot = robots[i];

        if (robot.getIndex() != -1 && robot.getVisibility() && robot.isMoving()) {

            if (robot.current_Plank.getReward() > best_reward && !robot.current_Plank.willExitGreen()) {
                best_reward = robot.current_Plank.getReward();
                target = robot;
            }
        }
    }

    return target;
}
*/

//Alternative method for choosing target, this one uses the best drone position at intersection instead of the best plank.
Robot AI::chooseTarget(std::array<Robot,10> robots, Drone drone) {
    Robot robot;
    point_t best_pos = point_Zero;
    float best_reward = getGridValue(best_pos.x, best_pos.y);

    // Return an invalid robot if none was assigned
    Robot target = Robot(-1);

    for (int i = 0; i < robots.size(); i++) {
        robot = robots[i];
        if (robot.getIndex() != -1 && robot.getVisibility() && robot.isMoving()) {

            if(getGridValue(best_pos.x, best_pos.y) > best_reward && !robot.current_Plank.willExitGreen()){
                best_pos = drone.getInterceptPoint(robot);
                best_reward = getGridValue(best_pos.x, best_pos.y);
                target = robot;
            }
        }
    }
    return target;
}

action_t AI::squareSearch(Robot target, Drone drone) {
    action_t search_Action = empty_action;

    point_t next_search_point = point_Zero;

    point_t pos = drone.getPosition();
    float x = pos.x;
    float y = pos.y;
    float track_width = 20;
    float track_height = 20;
    float track_center_x = track_width / 2;
    float track_center_y = track_height / 2;
    float padding = 3;

    // The drone flies in a square path in a clockwise order
    if (x > track_center_x && y > track_center_y) {
        next_search_point.x = track_width - padding;
        next_search_point.y = padding;
    } else if (x > track_center_x && y < track_center_y) {
        next_search_point.x = padding;
        next_search_point.y = padding;
    } else if (x <= track_center_x && y <= track_center_y) {
        next_search_point.x = padding;
        next_search_point.y = track_height - padding;
    } else if (x < track_center_x && y > track_center_y) {
        next_search_point.x = track_width - padding;
        next_search_point.y = track_height - padding;
    }

    search_Action.type = search;
    search_Action.where_To_Act = next_search_point;
    search_Action.target = 0;

    return search_Action;
}

action_t AI::chooseAction(Robot target, Drone drone) {
    // // Temporary max rewarded action
    action_t best_Action = empty_action;

    // best_Action.where_To_Act.travel_Time = interception.travel_Time;

    // Check if we have a visible target
    if (target.getIndex() != -1) {
        action_t step_Action;

        for (int i = 1; i < target.current_Plank.getNumPlankPoints() - 1; i++) {
            // std::cout << "Plank point " << i << ": " << target.current_Plank.getPoint(i).point.x << ", " << target.current_Plank.getPoint(i).point.y << std::endl;

            step_Action = getBestActionAtPosition(target.getOrientation(), target.current_Plank.getPoint(i));

            if (step_Action.reward > best_Action.reward) {
                best_Action = step_Action;
            }
        }

        best_Action.target = target.getIndex();
    }
    // If no target is visible, we do a patrol round
    else {
        best_Action = this->squareSearch(target, drone);
    }

    return best_Action;
}

action_t AI::getBestActionAtPosition(float target_orientation, plank_point_t position) {
    int num_Iterations = 5; // Number of iterations when summing along a plank
    action_t action;
    action.where_To_Act = position.point;

    Plank plank_On_Top = Plank(position.point, fmod(target_orientation + 2*MATH_PI - (MATH_PI/4), 2*MATH_PI), 
                                    position.time_since_start_turn);
    Plank plank_In_Front = Plank(position.point, fmod(target_orientation + MATH_PI, 2*MATH_PI), 
                                    position.time_since_start_turn);

    return actionWithMaxReward(plank_On_Top.getReward(), plank_In_Front.getReward(), action);
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