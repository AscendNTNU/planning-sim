#include "Node.h"
#include "../Plank.h"

Node::Node(Node* parent_p, Observation state, action_t action) {
	this->state = state;
	this->from_action = action;
	this->reward = this->state.getStateValue();
	this->time = parent_p->time + (this->state.getTimeStamp() - parent_p->time)

	this->parent_p = parent_p;
	this->children = createChildren(20.0);
	this->is_root = false;
}

float Node::getTime(){
	return this->time;
}

std::list<Node*> Node::createChildren(float tree_time_depth){
	
	std::list<Node*> children;
	AccessToSim sim;
	Node* node_p;
	Observation state;

	for(int i=0; i<10; i++){
		Robot robot = this->observation.robots[i];
		std::array<point_t, 10> action_points

		for(int j=1; j<11;j++){
		    action_t action;
		    action.where_To_Act = robot.plank.getPoint(j);

		    sim = AccessToSim(this->state);
		    action.type = sim_CommandType_LandOnTopOf;
		    state = sim.simulateAction(action);
		    node_p = &Node(this, state, action);

		    if(node_p->getTime() < tree_time_depth){
				children.push_back(node_p);
		    }

			sim = AccessToSim(this->state);
			action.type = sim_CommandType_LandInFrontOf;
		    state = sim.simulateAction(action);
		    node_p = &Node(this, state, action);

		    if(node_p->getTime() < tree_time_depth){
				children.push_back(node_p);
		    }
		}
	}

	return children;
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
	return sim.getObservation();
}

