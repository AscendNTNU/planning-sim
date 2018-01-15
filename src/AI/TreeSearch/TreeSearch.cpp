#include "TreeSearch.h"

TreeSearch::TreeSearch(Observation state) {
    this->state = state;
    Node root = Node(NULL, state, empty_action);

    std::list<Node> children;
    this->root_p = &root;
    this->best_node_p = this->root_p;
}

Node* TreeSearch::getRootPointer() {
    return this->root_p;
}

Node* TreeSearch::getBestNodePointer() {
    return this->best_node_p;
}

void TreeSearch::DFSBestAction(Node* node_p) {
    if(node_p->getReward() > this->best_node_p->getReward()){
        std::cout << "updating best node" << std::endl;
        this->best_node_p = node_p;
    }

    // std::list<Node>::iterator it;

    std::cout << "children contains " << node_p->children.size() << " elements.\n" << std::endl;
    if(node_p->children.size()<10 && node_p->children.size()>0){
        for (auto it = node_p->children.begin(); it != node_p->children.end(); it++) {
            std::cout << "deref iterator" << std::endl;
            Node child = *it;
            std::cout << "recursion call" << std::endl;
            this->DFSBestAction(&child);
            std::cout << "recursion out" << std::endl;

        }
    }
    std::cout << "done" << std::endl;
}


 std::queue<action_t> TreeSearch::getActionQueue(Node* node_p) {
    std::queue<action_t> action_queue;

    if (!node_p->getParentPointer()->isRoot()) {
        action_queue = this->getActionQueue(node_p->getParentPointer());
    }

    action_queue.push(node_p->getAction());
    return action_queue;
}