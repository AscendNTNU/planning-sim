#include "TreeSearch.h"
#include <iostream>
#include <list>

TreeSearch::TreeSearch(Observation observation) {
    this->observation_ = observation;

    node_t root;
    root.observation = observation;
    root.from_action = command_Empty;
    root.reward = 0;
    root.parent = NULL;
    root.root = true;

    std::list<node_t> children;
    for (int i = 0; i < Num_Targets; i++) {
        node_t child;
        child.parent = &root;
        root.children.push_front(child);
    }

    this->root_ = &root;
}

node_t TreeSearch::getBestAction(node_t node) {
    if (node.time >= 20) {
        return node;
    }

    std::list<node_t> children = node.children;
    node_t best_node;

    std::list<node_t>::iterator it;
    for (it = children.begin(); it != children.end(); it++) {
        node_t child = *it;
        node_t child_candidate = this->getBestAction(child);

        // Set best child candidate
        best_node = child_candidate;
    }

    return best_node;
}

std::list<node_t> TreeSearch::getChildren(node_t node) {
    return node.children;
}
