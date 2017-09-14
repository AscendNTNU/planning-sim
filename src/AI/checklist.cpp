#define SIM_IMPLEMENTATION
#define SIM_CLIENT_CODE
#include "../sim.h"
#include "../gui.h"
#include <stdio.h>
#include <iostream>
#include <algorithm>

static double SPEED = 0.33;
static double MATH_PI = 3.141592653589793238;




bool target_inActionRange(sim_Observed_State observed_state, int target){
    float dx = observed_state.drone_x - observed_state.target_x[target];
    float dy = observed_state.drone_y - observed_state.target_y[target];
    if(sqrt(dx*dx + dy*dy) > Sim_Drone_Target_Proximity){
        return true;
    }
    return false;
}

// targetIsMoving -> Robot.isMoving()


// getInterceptPointWithTurn and
// calculateInterceptionPoint -> Drone.calculateInterceptionPoint


//Pseudo ish kode
// choose_target -> AI.chooseTarget



// getBestActionAtPoint -> AI.getBestAction

// isOutsideOfPlank -> Plank pointIsOutsideOfPlank

void printActionIteration(int i, Target target, float x, float y, sim_Observed_State state, float time_until_intersection, float time_after_intersection) {
    float time_to_act = state.elapsed_time + 
                      time_until_intersection + 
                      time_after_intersection;

    std::cout << std::endl << "Iteration " << i << std::endl;
    std::cout << "Plank   X = [" << target.plank.x_1 << ", " << target.plank.x_2 << "]" << std::endl;
    std::cout << "Plank   Y = [" << target.plank.y_1 << ", " << target.plank.y_2 << "]" << std::endl;
    std::cout << "Current X = "<< x << std::endl;
    std::cout << "Current Y = "<< y << std::endl;
    std::cout << "Time to act: " << time_to_act << std::endl;
    std::cout << "Global time: " << state.elapsed_time << std::endl;
    std::cout << "Time until intersection: " << time_until_intersection << std::endl;
    std::cout << "Time after intersection: " << time_after_intersection << std::endl;
}

// choose_action -> AI.choose_Action

int main()
{
    sim_init_msgs(true);

    ActionReward action_pos_reward;
	action_pos_reward.action = ai_waiting;
    ai_State ai_state = ai_chooseTarget;

    sim_State state;
    bool running = true;

    sim_Command cmd;
    sim_Observed_State observed_state;
    sim_Observed_State previous_state;
    int target_index = -1;
    float last_action_time;
	float time_to_act = 200.0;

    Target target;

    while (running)
    {
        sim_recv_state(&state);
        previous_state = observed_state;
        sim_Observed_State observed_state = sim_observe_state(state);
		if(observed_state.elapsed_time > 600) {
			break;
		}
        
            last_action_time = observed_state.elapsed_time;

            //if(target_index == -1){
                //target = choose_target(observed_state, previous_state);
                //target_index = target.index; 
                ////ai_state = ai_tracking;
                ////cmd.type = sim_CommandType_Track;
                ////cmd.i = target.index;
                //std::cout << "Tracking" << std::endl;
            //}
//
            //else if(target_inActionRange(observed_state, target.index)
                    //&& targetIsMoving(target.index, previous_state, observed_state))
            //{                
                

            switch (ai_state)
            {
				case ai_chooseTarget:
					target = choose_target(observed_state, previous_state);
					target_index = target.index;
					std::cout << "Choose target" << std::endl;
					if(target.index == -1) { // In class version, index is never set to -1, we have to check this some other way
						ai_state = ai_noTargetFound;
					}
					else {
						ai_state = ai_chooseAction;
					}
				break;
				case ai_noTargetFound:
				{
					//for sim
					int teller = 0;
					for(int i = 0; i++; i < Num_Targets) {
						if(observed_state.target_removed[i]) {
							teller++;
						}
					}
					if(teller == Num_Targets) {
						ai_state = ai_terminate;
					}
					else {
						ai_state = ai_chooseTarget;
					}
				}	
				break;
				case ai_chooseAction:
					std::cout << "In state Choose Action" << std::endl;
						
					action_pos_reward = choose_action(observed_state, target);
					ai_state = ai_waiting;
					if(action_pos_reward.action == ai_landingInFront) {
						std::cout << "Choose Action: Land in Front" << std::endl;
					}
					else if(action_pos_reward.action == ai_landingOnTop) {
						std::cout << "Choose Action: Land on top" << std::endl;
					}
					else if(action_pos_reward.action == ai_waiting) {
						std::cout << "Choose Action: Waiting" << std::endl;
						ai_state = ai_chooseTarget;
						break;
					}
					else {
						std::cout << "Choose Action: ... erm, what?" << std::endl;
					}

					cmd.type = sim_CommandType_Search;
					cmd.x = action_pos_reward.x;
					cmd.y = action_pos_reward.y;
                    cmd.reward = action_pos_reward.reward;
					sim_send_cmd(&cmd);
					// Tell drone do do action at time
					time_to_act = observed_state.elapsed_time + 
										action_pos_reward.time_until_intersection +
										action_pos_reward.time_after_intersection;

                    //previous_state = observed_state;

                    //while(!observed_state.drone_cmd_done) {
                        //observed_state = sim_observe_state(state);
                    //}

				break;
                case ai_landingOnTop:
					std::cout << "Land On Top" << std::endl;
                    cmd.type = sim_CommandType_LandOnTopOf;
                    cmd.i = target.index;
                    cmd.reward = action_pos_reward.reward;
                    sim_send_cmd(&cmd);
					ai_state = ai_waitForAction;
                break;
                case ai_landingInFront:
                    std::cout << "Land In Front" << std::endl;
                    cmd.type = sim_CommandType_LandInFrontOf;
                    cmd.i = target.index;
                    cmd.reward = action_pos_reward.reward;
                    sim_send_cmd(&cmd);
					ai_state = ai_waitForAction;
                break;
				case ai_waitForAction:
					if(observed_state.drone_cmd_done) {
						ai_state = ai_chooseTarget;
					}
				break;
                case ai_waiting:
					if(observed_state.target_removed[target.index]) {
						std::cout << "Target removed before we could do action. Choose target again." << std::endl;
						ai_state = ai_chooseTarget;
					}
					if(action_pos_reward.action == ai_waiting) {
						std::cout << "Drone decided that current plank is best. Choose target again." << std::endl;
						ai_state = ai_chooseTarget;
					}
                    if(observed_state.drone_cmd_done && 
							observed_state.elapsed_time >= time_to_act && 
							targetIsMoving(target.index, previous_state, observed_state)) {
						std::cout << "Do action" << std::endl;
						ai_state = action_pos_reward.action;
					}
					// if(ai_state == action_pos_reward.action){
					// 	//if robot has drifted
					// 	if(targetIsMoving(target.index, previous_state, observed_state) && (int)observed_state.elapsed_time %20 < 2) {
					// 		std::cout << "Think a robot has drifted. Choose action again" << std::endl;
					// 		ai_state = ai_chooseAction;
					// 	}
						//if(observed_state.elapsed_time > time_to_act) {
						//ai_state = action_pos_reward.action;
						//}
					// }
                break;
				case ai_terminate:
					running = false;
				break;
                default:
                break;
				
        //target_index = -1;
        }
    }
	std::cout << "Time is up!" << std::endl;
	//std::cout << "Time's up!" << std:endl;
    return 0;
}
