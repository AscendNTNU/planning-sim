#pragma once

#include <queue>
#include "Node.h"

class TreeSearch {
    private:
        Observation state;
        Node root;
        Node best_node;

    public:
        TreeSearch(Observation state);

        Node getRootPointer();
        Node getBestNodePointer();

        void DFSBestAction(Node node);
        std::queue<action_t> getActionQueue(Node node);
};
