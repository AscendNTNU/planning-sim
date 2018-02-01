#include "Node.h"

Node::Node(){
    this->from_action = empty_action;
    this->reward = -10000;
    this->time_stamp = -1;
    this->root = false;
    this->time_elapsed = -1;
    this->parent_p=NULL;
}


Node::Node(Observation state){
    this->state = state;
    this->from_action = empty_action;
    this->reward = -10000;
    this->time_stamp = state.getTimeStamp();
    this->root = true;
    this->time_elapsed = 0;
    this->parent_p=NULL;
    createChildren(10.0);
}

Node::Node(std::shared_ptr<Node> parent_p, Observation state, action_t action) {
    this->state = state;
    this->from_action = action;
    this->reward = this->state.getStateValue();
    // std::cout << "reward for node is " << this->reward << std::endl;
    this->time_stamp = state.getTimeStamp();
    this->parent_p = parent_p;
    this->root = false;
    this->time_elapsed = parent_p->time_elapsed + (this->state.getTimeStamp() - parent_p->getTimeStamp());
    createChildren(10.0);
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
    AccessToSim sim = AccessToSim(this->state);
    std::cout<<"Created child node" <<std::endl;
    for(int i=0; i<1 ; i++) { //hvorfor ser vi kun på en robot?

        Robot robot = this->state.getRobot(i);
        // std::cout << "robot index " << robot.getIndex() << std::endl;

        for(int j=1; j<11;j = j+3){
            std::cout<<"==================" << std::endl;
            action_t action;
            action.target = robot.getIndex();
            action.where_To_Act = robot.getCurrentPlank().getPoint(j).point;
            std::cout << action << std::endl;
            sim = AccessToSim(this->state);
            
            action.type = land_On_Top_Of;
            state = simulateAction(action, sim);

            // std::cout << "time elapsed since root node timestamp: " << this->getTimeElapsed() + (state.getTimeStamp()-this->getTimeStamp()) << std::endl;
            if(this->getTimeElapsed() + (state.getTimeStamp()-this->getTimeStamp()) < tree_time_depth){
                this->children.push_back(Node(std::make_shared<Node>(*this), state, action));
            }

            sim = AccessToSim(this->state);
            action.type = land_In_Front_Of;
            state = simulateAction(action, sim);
            // std::cout << "time elapsed since root node timestamp: " << this->getTimeElapsed() + (state.getTimeStamp()-this->getTimeStamp()) << std::endl;
            if(this->getTimeElapsed() + (state.getTimeStamp()-this->getTimeStamp()) < tree_time_depth){
                this->children.push_back(Node(std::make_shared<Node>(*this), state, action));
            }
            std::cout<<"==================" << std::endl;
        }
    }

    
}

Observation simulateAction(action_t action, AccessToSim sim) {
    action_t fly_to = action;
    fly_to.type = search;
    Observation state = sim.simulateAction(fly_to);
    int tick = 0;
    while(tick < 10 && !pointsWithinThreshold(action.where_To_Act, state.getRobot(action.target).getPosition(), 0.5)) {
        state = sim.stepNoCommand();
        tick++;
    }
    if(tick >= 10
        ){
        std::cout << "time out :( " << std::endl;
    }
    //if tictimeout blah blah
    return sim.simulateAction(action);
}
