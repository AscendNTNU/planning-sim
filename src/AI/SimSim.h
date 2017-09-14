#pragma once
#include "structs.h"
#define SIM_IMPLEMENTATION
#define SIM_CLIENT_CODE
#include "../../sim.h"
#include "../../gui.h"

class SimSim{
private:
	sim_State state;
	sim_Command cmd;
    sim_Observed_State observed_state;
    sim_Observed_State previous_state;

public:
	SimSim();
	bool getNewObservation();
	bool sendCommand(action_t action);
	bool isActionDone();
	observation_t updateObservation();
};


SimSim::SimSim(){
	sim_init_msgs(true);
}
bool SimSim::getNewObservation(){
	 sim_recv_state(&this->state);
     this->previous_state = this->observed_state;
     this->observed_state = sim_observe_state(state);
     return true;
}

bool SimSim::isActionDone(){
	observation_t observation = this->updateObservation();
	return observation.drone_cmd_done;
}

sim_CommandType aiActionConverter(action_Type_t action){
	switch(action){

		case land_On_Top_Of:
			return sim_CommandType_LandOnTopOf;
		break;
		case land_In_Front_Of:
			return sim_CommandType_LandInFrontOf;
		break;
		case land_At_Point:
			return sim_CommandType_NoCommand;
		break;
		case search:
			return sim_CommandType_Search;
		break;
		default:
			return sim_CommandType_NoCommand;
		break;

	}
}


bool SimSim::sendCommand(action_t action){

	//Fly to interception point
	observation_t observation = this->updateObservation();


	this->cmd.type = sim_CommandType_Search;
	this->cmd.x = action.where_To_Act.x;
	this->cmd.y = action.where_To_Act.y;
	sim_send_cmd(&this->cmd);

	while (!this->isActionDone()) {
		// std::cout << "Travel not done yet" << std::endl;
	}

	//Wait for correct time to act
	observation = this->updateObservation();
	float action_Start = observation.elapsed_time;
	while(observation.elapsed_time-action_Start <= action.when_To_Act){
		if(observed_state.target_x[action.target] > 20 || observed_state.target_x[action.target] < 0 ||
			observed_state.target_y[action.target] > 20 || observed_state.target_y[action.target] < 0) {
			std::cout << "Target removed before we could do action. Choose target again." << std::endl;
			return false;
		}
		observation = this->updateObservation();
		// std::cout << "Waiting for action timing" << std::endl;
	}

	//If target is not turning act, if not dont do anything(most likely a stupid action if timer is wrong)
	// if(action.target.isMoving()){
	this->cmd.type = aiActionConverter(action.type);
	this->cmd.i = action.target;
	sim_send_cmd(&this->cmd);

	while (!this->isActionDone()) {
		// std::cout << "Action not done yet" << std::endl;
	}
	return true;
	// }
}

//Update world
observation_t SimSim::updateObservation(){
	
	observation_t observation;

	//Simulation updater
	this->getNewObservation();

	observation.elapsed_time = this->observed_state.elapsed_time;

	observation.drone_x = this->observed_state.drone_x;
	observation.drone_y = this->observed_state.drone_y;

	observation.drone_cmd_done = this->observed_state.drone_cmd_done;

	observation.num_Targets = this->observed_state.num_Targets;

	for(int i = 0; i < 10; i++){
		observation.robot_x[i] = this->observed_state.target_x[i];
		observation.robot_y[i] = this->observed_state.target_y[i];
		observation.robot_q[i] = this->observed_state.target_q[i];
	}
	for(int i = 0; i < 4; i++){
		observation.obstacle_x[i] = this->observed_state.obstacle_x[i];
		observation.obstacle_y[i] = this->observed_state.obstacle_y[i];
		observation.obstacle_q[i] = this->observed_state.obstacle_q[i];
	}

	return observation;
}