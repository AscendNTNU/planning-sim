#include "TreeSearch.h"

TreeSearch::TreeSearch(Observation state) {
    this->state = state;
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
    if(node.getReward() > this->best_node.getReward()){
        this->best_node = node;
    }

    // std::list<Node>::iterator it;

    std::cout << "children contains " << node.children.size() << " elements.\n" << std::endl;
    for (auto it = node.children.begin(); it != node.children.end(); it++) {
        std::cout << "deref iterator" << std::endl;
        Node child = *it;
        std::cout << "recursion call" << std::endl;
        this->DFSBestAction(child);

    }
    std::cout << "done" << std::endl;
}

 std::queue<action_t> TreeSearch::getActionQueue(Node& node) {
    std::queue<action_t> action_queue;

    std::cout<<"1"<<std::endl;

    if (!node.parent_p->isRoot()) {
        std::cout<<"2"<<std::endl;

        action_queue = this->getActionQueue(*(node.parent_p));
    }

    std::cout<<"3"<<std::endl;
    action_queue.push(node.getAction());
    return action_queue;
}