#pragma once

#include <list>
#include "AccessToSim.h"

class Node {
    private:
        Node* parent_p;
        Observation state;
        action_t from_action;
        float reward;
        float time_stamp;
        float time_elapsed;
        bool root;

    public:
        std::list<Node> children;
        Node();
        Node(Observation state);
        Node(Node* parent_p, Observation state, action_t action);

        float getTimeElapsed();
        float getTimeStamp();
        float getReward();
        Observation getState();
        // std::list<Node*> getChildren();
        Node* getParentPointer();
        action_t getAction();

        bool isRoot();
        void createChildren(float tree_time_depth);
};

Observation simulateAction(action_t action, AccessToSim sim);