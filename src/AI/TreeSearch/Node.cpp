#include "Node.h"
#include "../Plank.h"

Node::Node(Node* parent_p, Observation state, action_t action) {
	this->state = state;
	this-> action = action;
	this->reward = this->state.getStateValue();
	this->time = parent_p->time + (this->state.getTimeStamp() - parent_p->time)

	this->parent_p = parent_p;
	this->children = createChildren(this->state);
}

std::list<Node*> Node::createChildren(){
	
	std::list<Node*> children;
	AccessToSim sim;

	for(int i=0; i<10; i++){
		Robot robot = this->observation.robots[i];
		std::array<point_t, 10> action_points

		for(int j=1; j<11;j++){
		    action_t action;
		    action.where_To_Act = robot.plank.getPoint(j);


		    sim = AccessToSim(this->state);
		    action.type = sim_CommandType_LandOnTopOf;
		    Observation state = sim.simulateAction(action);
		    Node* = &Node(this, state, action);
			children.push_back();

			sim = AccessToSim(this->state);
			action.type = sim_CommandType_LandInFrontOf;
		    Observation state = sim.simulateAction(action);
		    Node* = &Node(this, state, action);
			children.push_back();
		}
	}
}

void simulateAction(action_t action){
	action_t fly_to = action;
	fly_to.type = sim_CommandType_Search
	Observation state = sim.simulateAction(action);
	int tick = 0;
	while(!pointsWithinThreshold(action.where_To_Act, drone.position, 0.5) || tick > 60*40){
		sim.stepNoCommand();
		tick++;
	}
	sim.simulateAction(action);
}

