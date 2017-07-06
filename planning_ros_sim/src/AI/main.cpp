#include "World.h"
#include "AI.h"
#include "SimSim.h"

enum world_Type_t {simSim, rosSim, realWorld};

World* world;
Robot* target;

world_Type_t world_Type = simSim;
int target_index = -1;

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
		target_index = -1;
		while(target_index == -1){
			target = ai->chooseTarget(observation.num_Targets);
			target_index = target->getIndex();
		}
		action_t action = ai->chooseAction(target);
		//Send AI command to simulator
		bool verify = simSim->sendCommand(action); //TODO
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
			//Can use observation.elapsed_time = world->getCurrentTime()?
		break;
	}
}