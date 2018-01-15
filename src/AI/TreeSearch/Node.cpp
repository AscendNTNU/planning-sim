#include "Node.h"

Node::Node(){
    std::cout<<"called empty node constructor"<<std::endl;
    this->from_action = empty_action;
    this->reward = 0;
    this->time_stamp = 0;
    this->root = false;
    this->time_elapsed = 0;
    this->parent_p=NULL;
}


Node::Node(Observation state){
    this->state = state;
    this->from_action = empty_action;
    this->reward = this->state.getStateValue();
    this->time_stamp = state.getTimeStamp();
    this->root = true;
    this->time_elapsed = 0;
    this->parent_p=NULL;
    createChildren(30.0);
}

Node::Node(std::shared_ptr<Node> parent_p, Observation state, action_t action) {
    this->state = state;
    this->from_action = action;
    this->reward = this->state.getStateValue();
    this->time_stamp = state.getTimeStamp();
    this->parent_p = parent_p;
    this->root = false;
    this->time_elapsed = parent_p->time_elapsed + (this->state.getTimeStamp() - parent_p->getTimeStamp());
    createChildren(30.0);
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

// std::list<Node*> Node::getChildren(){
//     std::cout << "children contains " << this->children.size() << " elements.\n";
//     return this->children;
// }

bool Node::isRoot(){
    if(this->root){
        return true;
    }
    return false;
}


void Node::createChildren(float tree_time_depth) {

    Observation state;

    for(int i=0; i<2; i++) {

        Robot robot = this->state.getRobot(i);

        for(int j=1; j<11;j = j+3){
            action_t action;
            action.where_To_Act = robot.getCurrentPlank().getPoint(j).point;

            AccessToSim sim = AccessToSim(this->state);
            action.type = land_On_Top_Of;
            state = simulateAction(action, sim);

            if(this->getTimeElapsed() + (state.getTimeStamp()-this->getTimeStamp()) < tree_time_depth){
                std::cout<<"creating new node" <<std::endl;
                std::cout << "node created" << std::endl;
                std::cout<<"pushing Node" <<std::endl;
                this->children.push_back(Node(std::make_shared<Node>(*this), state, action));
                std::cout << "children now contains " << this->children.size() << " elements.\n" << std::endl;


            }

            sim = AccessToSim(this->state);
            action.type = land_In_Front_Of;
            state = simulateAction(action, sim);

            if(this->getTimeElapsed() + (state.getTimeStamp()-this->getTimeStamp()) < tree_time_depth){
                std::cout<<"creating new node" <<std::endl;
                std::cout << "node created" << std::endl;

                std::cout<<"pushing Node" <<std::endl;
                this->children.push_back(Node(std::make_shared<Node>(*this), state, action));
                std::cout << "children now contains " << this->children.size() << " elements.\n" << std::endl;
            }
        }
    }

    
}

Observation simulateAction(action_t action, AccessToSim sim) {
    action_t fly_to = action;
    fly_to.type = search;
    Observation state = sim.simulateAction(action);
    int tick = 0;
    while(tick < 20 && !pointsWithinThreshold(action.where_To_Act, state.getRobot(action.target).getPosition(), 0.5)) {
        state = sim.stepNoCommand();
        tick++;
    }
    return sim.simulateAction(action);
}
