#pragma once

#include <queue>
#include <memory>
#include "Node.h"

class TreeSearch {
    private:
        Observation state;
        Node root;
        Node best_node;

    public:
        TreeSearch(Observation state);

        Node getRoot();
        Node getBestNode();

        void DFSBestAction(Node node);
        std::queue<action_t> getActionQueue(Node& node);
};
