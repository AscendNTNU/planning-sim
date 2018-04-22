#include "structs.h"
#include "Robot.h"

//functions
//isOutsideOfPlank

tree_action_t DFS(Robot robot,tree_action_t currentLevel, int n){
	
	tree_action_t temp = currentLevel;
	tree_action_t max

	action_Top = empty_action;
    action_Top.type = land_on_top_of;

	action_Front empty_action;
    action_Front.type = land_in_front_of;
	
	point_t temp_Position = robot.position;

	float orientation = robot.orientation;

	int i = 0;
	
	while (i > 0) {
		if (robot.plank->isOutsideOfPlank(temp_Position)) {
            if (backwards) {
                return best_action;
            } else {
                i = n+1;
                backwards = true;
                angle += MATH_PI;
            }
		}

        // Hvordan regner vi ut intersection point for hver runde?
        // Fra hvilken droneposisjon?
		robot.addToTimer(target.intersection.travel_time+time_after_intersection);


		//Try action land on top.
        Plank temp_Plank = new Plank(temp_Position, orientation, time_after_turn_start, n);
        temp.reward = findRobotValue(robot);
		temp = currentLevel;
		robot.setPositionOrientation(robot.getPosition(), robot.plank.getAngle()+0.785);
		action_Top.when_To_Act = i;
        temp.actions.push(action_Top);
        temp = DFS(robot,temp, n);

        if(temp.reward > max.reward){
        	max = temp;
        }

        //Try action land in front
        temp = currentLevel;
        robot.setPositionOrientation(robot.getPosition(), robot.plank.getAngle()-0.785 + 3.14);
        float rewardInFront = findRobotValue(robot);
        temp.reward = findRobotValue(robot);
        action_Front.when_To_Act = i;
        temp.actions.push(action_Top);
        temp = DFS(robot, temp, n);
        if(temp.reward > max.reward){
        	max = temp;
        }


        if (backwards) {
            temp_Position.x -= step_x;
            temp_Position.y -= step_y;
            robot.setPositionOrientation(temp_Position, start_Orientation + 3.14);
            i -= 1;

        } else {
			temp_Position.x += temp_Position.x;
            temp_Position.y += temp_Position.y;
            robot.setPositionOrientation(temp_Position, start_Orientation);
            i += 1;
        }
        time_after_intersection = time_after_intersection + (step_size)/Config::Robot_Speed;
	}
    return max;
}

// dfs(state)
// for each action from state
//		get action, get action reward, get new state
//		if reward better than previous max reward update reward and action array
//		dfs(new_state)
//		
//		

// procedure DFS(G,v):
// 2      label v as discovered
// 3      for all edges from v to w in G.adjacentEdges(v) do
// 4          if vertex w is not labeled as discovered then
// 5              recursively call DFS(G,w)

// ActionNode root = new ActionNode(NULL);
// for(int target = 0; target < number_Of_Targets; target){
// 	Robot target = new Robot(this.state.robot[i]);
// 	reward = 
// 	action_t action1 = {.type = sim_CommandType_LandOnTopOf, .target = target, .reward = 
//     action_t action2 = {.type = sim_CommandType_LandInFrontOf, 
// 	ActionNode child1 = new ActionNode(root*, );
// 	ActionNode child2 = new ActionNode(root*);
// 	root.addChild(child1*);
// 	root.addChild(child2*);
// }



// struct action_t{
// 	action_Type type;
// 	Robot target;
// 	point_t waypoints[10];
// 	float reward;
// 	float when_To_Act;
// }

//     int index = target.index;
//     float angle = wrap_angle(state.target_q[index]);
    
//     target.intersection = calculateInterceptionPoint(state, target);
//     float temp = target.intersection.travel_time;
//     //target.intersection.travel_time = 0;

//     float n = 10;
//     float step_size = target.plank.length/n;
//     float step_x = step_size*cos(angle);
//     float step_y = step_size*sin(angle);

//     float x = target.intersection.x;
//     float y = target.intersection.y;
//     float time_after_intersection = 0;

//     // Temporary max rewarded action
//     ActionReward best_action;
//     best_action.reward = -1000;
//     best_action.action = ai_waiting;
//     best_action.time_until_intersection = target.intersection.travel_time;
    
//     ActionReward action_to_check;
//     bool backwards = false;
//     int i = 1;
//     while (i > 0) {
//         printActionIteration(i, target, x, y, state, best_action.time_until_intersection, time_after_intersection);

//         if (isOutsideOfPlank(x,y, target.plank)) {
//             std::cout << "End of plank was reached " << std::endl;
//             if (backwards) {
//                 return best_action;
//             } else {
//                 i = n+1;
//                 backwards = true;
//                 target.angle += MATH_PI;
//             }
//         }
//         action_to_check = getBestActionAtPoint(target, x, y, state, time_after_intersection);

//         if (action_to_check.reward > best_action.reward) {
//             best_action = action_to_check;
//             best_action.x = x;
//             best_action.y = y;
//             best_action.time_after_intersection = time_after_intersection;
//             best_action.time_until_intersection = target.intersection.travel_time;
//         }

        

//         if (backwards) {
//             x = x-step_x;
//             y = y-step_y;
//             i -= 1;
//         } else {
//             x = x+step_x;
//             y = y+step_y;
//             i += 1;
//         }
//         time_after_intersection = time_after_intersection + (step_size)/Config::Robot_Speed;

//     }
//     best_action.time_until_intersection = temp;
//     return best_action;
// }