#include "TreeSearch.h"

TreeSearch::TreeSearch(Observation state) {
    this->state = state;
    Node* root_p = Node(NULL, state, command_Empty);

    std::list<Node> children;
    this->root_p = root_p;
}

Node* TreeSearch::getRootPointer() {
    return this->root_p;
}

Node* TreeSearch::getBestNodePointer() {
    return this->best_node_p;
}

void TreeSearch::DFSBestAction(Node* node_p) {

    if(node_p->getReward() > this->best_node_p->getReward()){
        this->best_node_p = node_p;
    }

    std::list<Node*> children = node_p->getChildren();
    std::list<Node*>::iterator it;

    for (it = children.begin(); it != children.end(); it++) {
        Node* child_p = *it;
        this->DFSBestAction(child_p);
    }
}


 std::queue<action_t> TreeSearch::getActionQueue(Node* node_p) {
    std::queue<action_t> action_queue;

    if (!node_p->getParentPointer()->isRoot()) {
        action_queue = this->getActionQueue(node_p->getParentPointer());
    }

    action_queue.push(node_p->getAction());
    return action_queue;
}