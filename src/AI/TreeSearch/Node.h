#pragma once

#include <list>
#include "../structs.h"

class Node {
    private:

    Node* parent_p;
    std::list<Node*> children;
    Observation state;
    action_t from_action;
    float reward;
    float time;
    bool is_root;

    public:
        float getTime();
        Node(Node* parent_p, Observation state, action_t action);
        std::list<Node*> createChildren(float tree_time_depth);
};
