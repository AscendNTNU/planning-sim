#include "TreeSearch.h"
#include "Node.h"
#include <iostream>
#include <list>
#include <stack>

TreeSearch::TreeSearch(Observation state) {
    this->state_ = state;

    Node root;
    root.state = state;
    root.from_action = command_Empty;
    root.reward = 0;
    root.parent = NULL;
    root.root = true;

    std::list<Node> children;
    for (int i = 0; i < Num_Targets; i++) {
        Node child;
        child.parent = &root;
        root.children.push_front(child);
    }

    this->root_ = &root;
}

void TreeSearch::DFS(Node* node_p) {

    if(node_p->reward > this->best_node_p_->reward){
        this->best_node_p_ = node_p;
    }


    std::list<Node*> children = node_p->children;
    std::list<Node*>::iterator it;

    for (it = node_p->children.begin(); it != node_p->children.end(); it++) {
        Node* child_p = *it;
        this->DFS(child_p);
    }
}

std::list<Node> TreeSearch::getChildren(Node node) { // might be unnecessary
    return node.children;
}

 std::stack<action_t> TreeSearch::getActionStack(Node* node_p, std::stack<action_t>& action_stack) {
    action_stack.push(node_p->action);

    if (node_p->parent != NULL) {
        this->getActionStack(node_p->parent, action_stack);
    }
    return action_stack;
}