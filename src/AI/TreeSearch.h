#include "AccessToSim.h"
#include "structs.h"
#include <list>

struct node_t {
    node_t* parent;
    std::list<node_t> children;
    Observation observation;
    sim_Command from_action;
    float reward = 0;
    float time = 0;
    bool root = false;
};

class TreeSearch {
    private:
        Observation observation_;
        node_t* root_;
    public:
        TreeSearch(Observation observation);
        node_t getBestAction(node_t node);
        std::list<node_t> getChildren(node_t node);
};
