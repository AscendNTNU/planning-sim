#include "Node.h"
#include "../Plank.h"
#include "../AI.h"


Node::Node(Node* parent_p, Observation state, action_t action) {
	this->state = state;
	this-> action = action;
	this->reward = total_plank_value(this->state);
	this->time = parent_p->time + (this->state.getTimeStamp() - parent_p->time)

	this->parent_p = parent_p;
	this->children = create_children(this->state);
}

std::list<node_t*> Node::create_children(){
	
	std::list<node_t*> children;

	for(int i=0; i<10; i++){
		Robot robot = this->observation.robots[i];
		std::array<point_t, 10> action_points

		for(int j=1; j<11;j++){
			point_t action_point = robot.plank.getPoint(j);

			node_t* = &Node(this, state, action);
			children.push_back();
		}
		

		Plank(position.point, fmod(target_orientation + (MATH_PI/4), 2*MATH_PI), 
                                     position.time_since_start_turn);
	}
}