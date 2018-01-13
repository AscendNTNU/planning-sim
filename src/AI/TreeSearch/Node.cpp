#include "Node.h"
#include "../Plank.h"

Node::Node(Node* parent_p, Observation state, action_t action) {
    this->state = state;
    this->from_action = action;
    this->reward = this->state.getStateValue();
    if(parent_p == NULL){
    	this->time = 0;
    }
    else{
    	this->time = parent_p->time + (this->state.getTimeStamp() - parent_p->time);
	}
	this->parent_p = parent_p;
	this->children = createChildren(20.0);
}

Node* Node::getParentPointer(){
	return this->parent_p;
}

Observation Node::getState(){
	return this->state;
}

action_t Node::getAction(){
	return this->from_action;
}

float Node::getReward() {
    return this->reward;
}

float Node::getTime() {
    return this->time;
}

std::list<Node*> Node::getChildren(){
	return this->children;
}

bool Node::isRoot(){
	if(this->parent_p == NULL){
		return true;
	}
	return false;
}


std::list<Node*> Node::createChildren(float tree_time_depth) {
    std::list<Node*> children;
    Node* node_p;
    Observation state;

    for(int i=0; i<10; i++) {
        Robot robot = this->state.getRobot(i);
        std::array<point_t, 10> action_points;

		for(int j=1; j<11;j++){
		    action_t action;
		    action.where_To_Act = robot.getCurrentPlank().getPoint(j).point;

            AccessToSim sim = AccessToSim(this->state);
            action.type = land_On_Top_Of;
            state = sim.simulateAction(action);
            node_p = &Node(this, state, action);

            if(node_p->getTime() < tree_time_depth) {
                children.push_back(node_p);
            }

            sim = AccessToSim(this->state);
            action.type = land_In_Front_Of;
            state = sim.simulateAction(action);
            node_p = &Node(this, state, action);

            if(node_p->getTime() < tree_time_depth) {
                children.push_back(node_p);
            }
        }
    }

    this->children = children;

    return children;
}

Observation simulateAction(action_t action) {
    action_t fly_to = action;
    fly_to.type = search;
    Observation state = sim.simulateAction(action);
    int tick = 0;
    while(!pointsWithinThreshold(action.where_To_Act, state.robot[action.target].position, 0.5) || tick > 60*40) {
        state = sim.stepNoCommand();
        tick++;
    }
    if(action.type == land_On_Top_Of) {
        return sim.simulateAction(action);
    }



}
