#include "TreeSearch.h"
#include <iostream>
#include <list>

TreeSearch::TreeSearch(Observation state) {
    this->state_ = state;

    node_t root;
    root.state = state;
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

void TreeSearch::DFS(node_t* node_p) {

    if(node_p->reward > this->best_node_p_->reward){
        this->best_node_p_ = node_p;
    }


    std::list<node_t*> children = node_p->children;
    std::list<node_t*>::iterator it;

    for (it = node_p->children.begin(); it != node_p->children.end(); it++) {
        node_t* child_p = *it;
        this->DFS(child_p);
    }
}

std::list<node_t> TreeSearch::getChildren(node_t node) {
    return node.children;
}