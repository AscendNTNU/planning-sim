#include "World.h"
#include "AI.h"

#include "SimSim.h"

enum world_Type_t {simSim, rosSim, realWorld};

World* world;

world_Type_t world_Type = simSim;

bool timer_Started = false;

bool simSimLoop(AI* ai){
	SimSim* simSim = new SimSim();
	observation_t observation;
	while(1){
		//get observations
		observation = simSim->updateObservation();
		if (!timer_Started) {
			timer_Started = world->startTimer();
		}
		//Send to AI
		ai->update(observation);
		//Return AI command
		Robot* target = ai->chooseTarget(10);
		action_t action = ai->chooseAction(target);
		//Send AI command to simulator
		bool verify = simSim->sendCommand(action); //TODO
		// std::cout << "Action send worked? " << verify << std::endl;
	}
}

int main(){
	float compass_orientation = 0.0;
	
	world = new World(compass_orientation);
	
	AI* ai = new AI();

	switch(world_Type){
		case simSim:
			simSimLoop(ai);
		break;
		case rosSim:
			//todo
		break;
		case realWorld:
			//todo
		break;
	}

}
//From SolutionV0.2

// void printActionIteration(int i, Target target, float x, float y, sim_Observed_State state, float time_until_intersection, float time_after_intersection) {
//     float time_to_act = state.elapsed_time + 
//                       time_until_intersection + 
//                       time_after_intersection;

//     std::cout << std::endl << "Iteration " << i << std::endl;
//     std::cout << "Plank   X = [" << target.plank.x_1 << ", " << target.plank.x_2 << "]" << std::endl;
//     std::cout << "Plank   Y = [" << target.plank.y_1 << ", " << target.plank.y_2 << "]" << std::endl;
//     std::cout << "Current X = "<< x << std::endl;
//     std::cout << "Current Y = "<< y << std::endl;
//     std::cout << "Time to act: " << time_to_act << std::endl;
//     std::cout << "Global time: " << state.elapsed_time << std::endl;
//     std::cout << "Time until intersection: " << time_until_intersection << std::endl;
//     std::cout << "Time after intersection: " << time_after_intersection << std::endl;
// }

// int main(){
//     sim_init_msgs(true);

//     ActionReward action_pos_reward;
// 	action_pos_reward.action = ai_waiting;
//     ai_State ai_state = ai_chooseTarget;

//     sim_State state;
//     bool running = true;

//     sim_Command cmd;
//     sim_Observed_State observed_state;
//     sim_Observed_State previous_state;
//     int target_index = -1;
//     float last_action_time;
// 	float time_to_act = 200.0;

//     Target target;

//     while (running)
//     {
//         sim_recv_state(&state);
//         previous_state = observed_state;
//         sim_Observed_State observed_state = sim_observe_state(state);
// 		if(observed_state.elapsed_time > 600) {
// 			break;
// 		}
        
//             last_action_time = observed_state.elapsed_time;

//             switch (ai_state)
//             {
// 				case ai_chooseTarget:
// 					target = choose_target(observed_state, previous_state);
// 					target_index = target.index;
// 					std::cout << "Choose target" << std::endl;
// 					if(target.index == -1) { // In class version, index is never set to -1, we have to check this some other way
// 						ai_state = ai_noTargetFound;
// 					}
// 					else {
// 						ai_state = ai_chooseAction;
// 					}
// 				break;
// 				case ai_noTargetFound:
// 				{
// 					//for sim
// 					int teller = 0;
// 					for(int i = 0; i++; i < Num_Targets) {
// 						if(observed_state.target_removed[i]) {
// 							teller++;
// 						}
// 					}
// 					if(teller == Num_Targets) {
// 						ai_state = ai_terminate;
// 					}
// 					else {
// 						ai_state = ai_chooseTarget;
// 					}
// 				}	
// 				break;
// 				case ai_chooseAction:
// 					std::cout << "In state Choose Action" << std::endl;
						
// 					action_pos_reward = choose_action(observed_state, target);
// 					ai_state = ai_waiting;
// 					if(action_pos_reward.action == ai_landingInFront) {
// 						std::cout << "Choose Action: Land in Front" << std::endl;
// 					}
// 					else if(action_pos_reward.action == ai_landingOnTop) {
// 						std::cout << "Choose Action: Land on top" << std::endl;
// 					}
// 					else if(action_pos_reward.action == ai_waiting) {
// 						std::cout << "Choose Action: Waiting" << std::endl;
// 						ai_state = ai_chooseTarget;
// 						break;
// 					}
// 					else {
// 						std::cout << "Choose Action: ... erm, what?" << std::endl;
// 					}

// 					cmd.type = sim_CommandType_Search;
// 					cmd.x = action_pos_reward.x;
// 					cmd.y = action_pos_reward.y;
//                     cmd.reward = action_pos_reward.reward;
// 					sim_send_cmd(&cmd);
// 					// Tell drone do do action at time
// 					time_to_act = observed_state.elapsed_time + 
// 										action_pos_reward.time_until_intersection +
// 										action_pos_reward.time_after_intersection;

// 				break;
//                 case ai_landingOnTop:
// 					std::cout << "Land On Top" << std::endl;
//                     cmd.type = sim_CommandType_LandOnTopOf;
//                     cmd.i = target.index;
//                     cmd.reward = action_pos_reward.reward;
//                     sim_send_cmd(&cmd);
// 					ai_state = ai_waitForAction;
//                 break;
//                 case ai_landingInFront:
//                     std::cout << "Land In Front" << std::endl;
//                     cmd.type = sim_CommandType_LandInFrontOf;
//                     cmd.i = target.index;
//                     cmd.reward = action_pos_reward.reward;
//                     sim_send_cmd(&cmd);
// 					ai_state = ai_waitForAction;
//                 break;
// 				case ai_waitForAction:
// 					if(observed_state.drone_cmd_done) {
// 						ai_state = ai_chooseTarget;
// 					}
// 				break;
//                 case ai_waiting:
// 					if(observed_state.target_removed[target.index]) {
// 						std::cout << "Target removed before we could do action. Choose target again." << std::endl;
// 						ai_state = ai_chooseTarget;
// 					}
// 					if(action_pos_reward.action == ai_waiting) {
// 						std::cout << "Drone decided that current plank is best. Choose target again." << std::endl;
// 						ai_state = ai_chooseTarget;
// 					}
//                     if(observed_state.drone_cmd_done && 
// 							observed_state.elapsed_time >= time_to_act && 
// 							targetIsMoving(target.index, previous_state, observed_state)) {
// 						std::cout << "Do action" << std::endl;
// 						ai_state = action_pos_reward.action;
// 					}
//                 break;
// 				case ai_terminate:
// 					running = false;
// 				break;
//                 default:
//                 break;
				
//         }
//     }
// 	std::cout << "Time is up!" << std::endl;
//     return 0;
// }