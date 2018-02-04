#include "Node.h"

Node::Node(){
    this->from_action = empty_action;
    this->reward = -10000;
    this->time_stamp = -1;
    this->time_elapsed = -1;
    this->parent_p=NULL;
    this->depth=0;
}


Node::Node(Observation state){
    this->state = state;
    this->from_action = empty_action;
    this->reward = -10000;
    this->time_stamp = state.getTimeStamp();
    this->time_elapsed = 0;
    this->parent_p=NULL;
    this->depth = 0;
    createChildren(20.0);
}

Node::Node(std::shared_ptr<Node> parent_p, Observation state, action_t action) {
    this->state = state;
    this->from_action = action;
    this->reward = this->state.getStateValue();
    // std::cout << "reward for node is " << this->reward << std::endl;
    this->time_stamp = state.getTimeStamp();
    this->parent_p = parent_p;
    this->depth = parent_p->getDepth()+1;
    this->time_elapsed = parent_p->time_elapsed + (this->state.getTimeStamp() - parent_p->getTimeStamp());
    createChildren(20.0);
}

std::shared_ptr<Node> Node::getParentPointer(){
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

float Node::getTimeElapsed() {
    return this->time_elapsed;
}

float Node::getTimeStamp(){
    return this->time_stamp;
}

int Node::getDepth(){
    return this->depth;
}

// std::list<Node*> Node::getChildren(){
//     std::cout << "children contains " << this->children.size() << " elements.\n";
//     return this->children;
// }

bool Node::isRoot(){
    if(this->depth == 0){
        return true;
    }
    return false;
}


void Node::createChildren(float time_limit) {
    Observation state;
    AccessToSim sim = AccessToSim(this->state);
    bool time_out = false;
    action_t action;

    std::cout<<"Created child node" <<std::endl;

    for(int i=0; i<1 ; i++) {

        Robot robot = this->state.getRobot(i);
        // std::cout << "robot index " << robot.getIndex() << std::endl;

        for(int j=3; j<11;j = j+3){
            std::cout<<"==================" << std::endl;

            action.target = robot.getIndex();
            action.where_To_Act = robot.getCurrentPlank().getPoint(j).point;
            // std::cout << action << std::endl;

            sim = AccessToSim(this->state);
            
            action.type = land_On_Top_Of;
            time_out = sim.simulateActionSequence(action,time_limit);
            state = sim.getObservation();
            std::cout<< "Depth: " << this->depth << std::endl;
            // std::cout << "time elapsed since root node timestamp: " << this->getTimeElapsed() + (state.getTimeStamp()-this->getTimeStamp()) << std::endl;
            if(this->getTimeElapsed() + (state.getTimeStamp()-this->getTimeStamp()) < time_limit &&
                    this->depth < 2){
                std::cout<<"creating new child" << std::endl;
                this->children.push_back(Node(std::make_shared<Node>(*this), state, action));
            }

            sim = AccessToSim(this->state);
            
            action.type = land_In_Front_Of;
            time_out = sim.simulateActionSequence(action, time_limit);
            state = sim.getObservation();
            std::cout<< "Depth: " << this->depth << std::endl;
            // std::cout << "time elapsed since root node timestamp: " << this->getTimeElapsed() + (state.getTimeStamp()-this->getTimeStamp()) << std::endl;
            if(this->getTimeElapsed() + (state.getTimeStamp()-this->getTimeStamp()) < time_limit &&
                    this->depth < 2){
                this->children.push_back(Node(std::make_shared<Node>(*this), state, action));
            }
            std::cout<<"==================" << std::endl;
        }
    }

    
}