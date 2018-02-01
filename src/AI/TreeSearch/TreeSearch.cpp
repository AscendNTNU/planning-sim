#include "TreeSearch.h"

TreeSearch::TreeSearch(Observation state) {
    this->state = state;
    std::cout<<"Creating root node" << std::endl;
    Node root = Node(state);

    std::list<Node> children;
    this->root = root;
    this->best_node = this->root;
}

Node TreeSearch::getRoot() {
    return this->root;
}

Node TreeSearch::getBestNode() {
    return this->best_node;
}

void TreeSearch::DFSBestAction(Node node) {
    // std::cout << "current best node reward: " << this->best_node.getReward() << std::endl;
    // std::cout << "this node reward: " << node.getReward() << std::endl;
    // std::cout << "root node reward: " << this->root.getReward() << std::endl;
    
    if(node.getReward() > this->best_node.getReward()){
        this->best_node = node;
        // std::cout << "new best reward is " << this->best_node.getReward() << std::endl;
    }

    for (auto it = node.children.begin(); it != node.children.end(); it++) {
        Node child = *it;
        this->DFSBestAction(child);

    }
}

std::queue<action_t> TreeSearch::getActionQueue(Node& node) {
    std::queue<action_t> action_queue;  

    if (!node.isRoot()) {
        action_queue = this->getActionQueue(*(node.parent_p));
        action_queue.push(node.getAction());
    }
    return action_queue;
}