#pragma once

#include "Node.h"
#include <queue>

class TreeSearch {
    private:
        Observation state;
        Node* root_p;
        Node* best_node_p;

    public:
        TreeSearch(Observation state);

        Node* getRootPointer();
        Node* getBestNodePointer();

        void DFSBestAction(Node* node);
        std::queue<action_t> getActionQueue(Node* node_p);
};
