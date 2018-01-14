#include "Node.h"

Node::Node(Node* parent_p, Observation state, action_t action) {
    this->state = state;
    this->from_action = action;
    this->reward = this->state.getStateValue();
    this->time_stamp = state.getTimeStamp();
    this->parent_p = parent_p;

    if(parent_p == NULL){
        this->time_elapsed = 0;
    }
    else{
        this->time_elapsed = parent_p->time_elapsed + (this->state.getTimeStamp() - parent_p->getTimeStamp());
    }
    this->children = createChildren(10.0);
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

float Node::getTimeElapsed() {
    return this->time_elapsed;
}

float Node::getTimeStamp(){
    return this->time_stamp;
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
    Observation state;

    for(int i=0; i<10; i++) {
        Robot robot = this->state.getRobot(i);
        std::array<point_t, 10> action_points;

        for(int j=1; j<11;j++){
            action_t action;
            action.where_To_Act = robot.getCurrentPlank().getPoint(j).point;

            AccessToSim sim = AccessToSim(this->state);
            action.type = land_On_Top_Of;
            state = simulateAction(action, sim);

            if(this->getTimeElapsed() + (state.getTimeStamp()-this->getTimeStamp()) < tree_time_depth){
                std::cout << "here" << std::endl;
                Node node = Node(this, state, action);
                std::cout << "lets go" << std::endl;
                Node* node_p = &node;
                children.push_back(node_p);
            }

            sim = AccessToSim(this->state);
            action.type = land_In_Front_Of;
            state = simulateAction(action, sim);

            if(this->getTimeElapsed() + (state.getTimeStamp()-this->getTimeStamp()) < tree_time_depth){
                std::cout << "here" << std::endl;
                Node node = Node(this, state, action);
                std::cout << "lets go" << std::endl;
                Node* node_p = &node;
                children.push_back(node_p);
            }
        }
    }

    this->children = children;

    return children;
}

Observation simulateAction(action_t action, AccessToSim sim) {
    action_t fly_to = action;
    fly_to.type = search;
    Observation state = sim.simulateAction(action);
    int tick = 0;
    while(tick < 10*60/5 && !pointsWithinThreshold(action.where_To_Act, state.getRobot(action.target).getPosition(), 0.5)) {
        state = sim.stepNoCommand();
        tick++;
    }
    return sim.simulateAction(action);
}
