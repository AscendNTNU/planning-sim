#include "AccessToSim.h"
#include "../structs.h"
#include <list>

struct node_t { 
    node_t* parent_p;
    AccessToSim sim;
    std::list<node_t*> children;
    Observation state;
    sim_Command from_action;
    float reward = 0;
    float time = 0;
    bool root = false;
};

class TreeSearch {
    private:
        Observation observation_;
        node_t* root_p_;
        node_t* best_node_p_;

    public:
        TreeSearch(Observation state);
        void DFS(node_t node);
};