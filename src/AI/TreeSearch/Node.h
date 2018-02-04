#pragma once

#include <list>
#include "AccessToSim.h"
#include <memory>


class Node {
    private:
        Observation state;
        action_t from_action;
        float reward;
        float time_stamp;
        float time_elapsed;
        int depth;

    public:
        std::list<Node> children;
        std::shared_ptr<Node> parent_p;

        Node();
        Node(Observation state);
        Node(std::shared_ptr<Node> parent_p, Observation state, action_t action);

        float getTimeElapsed();
        float getTimeStamp();
        float getReward();
        Observation getState();
        //std::list<Node*> getChildren();
        std::shared_ptr<Node> getParentPointer();
        action_t getAction();

        int getDepth();

        bool isRoot();
        void createChildren(float tree_time_depth);
};

Observation simulateAction(action_t action, AccessToSim sim);