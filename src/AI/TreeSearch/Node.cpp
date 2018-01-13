#include "Node.h"


Node::Node(Node* parent_p, Observation state, action_t action) {
	this->state = state;
	this-> action = action;
	this->reward = this->state.getStateValue();
	this->time = parent_p->time + (this->state.getTimeStamp() - parent_p->time)

	this->parent_p = parent_p;
	this->children = create_children(this->state);
}