#include "AI.h"
#include "structs.h"
#include "Observation.h"
#include <list>

class Node {
    private:
        Node* parent_p;
        std::list<Node*> children;
        Observation state;
        action_t action; // should be renamed action from?
        float reward;
        float time;

    public:
        Node(Node* parent_p, Observation state, action_t action);
        std::list<node_t*> Node::createChildren(AccessToSim sim);
        
};
