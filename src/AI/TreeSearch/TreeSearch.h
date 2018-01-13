#pragma once

#include "AccessToSim.h"
#include "Node.h"
#include <queue>

class TreeSearch {
    private:
        Observation observation;
        Node* root_p;
        Node* best_node_p;

    public:
        TreeSearch(Observation state);
        void DFSBestAction(Node* node);
        std::queue<action_t> getActionQueue(Node* node_p) {

};