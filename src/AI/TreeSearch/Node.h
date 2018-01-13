#pragma once

#include <list>
#include "../Observation.h"
#include "../structs.h"
#include "AccessToSim.h"

class Node {
    private:
        Node* parent_p;
        std::list<Node*> children;
        Observation state;
        action_t from_action;
        float reward;
        float time;

    public:
        Node(Node* parent_p, Observation state, action_t action);

        float getTime();
        float getReward();
        std::list<Node*> getChildren();
        Node* getParentPointer();
        action_t getAction();

        bool isRoot();
        std::list<Node*> createChildren(float tree_time_depth);
};
