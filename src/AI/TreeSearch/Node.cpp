#include "Node.h"


Node::Node(Node* parent_p, Observation state, action_t action) {
	this->state = state;
	this-> action = action;
	this->reward = total_plank_value(this->state);
	this->time = parent_p->time + (this->state.getTimeStamp() - parent_p->time)

	this->parent_p = parent_p;
	this->children = create_children(this->state);
}