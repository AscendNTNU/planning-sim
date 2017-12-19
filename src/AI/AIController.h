#include "AI.h"

enum ai_state_t{
    no_input_data,
    idle,
    fly_to,
    waiting,
    perform_action
};

class AIController{
private:
    AI ai_;
    ai_state_t state_;
    Robot target_;
    action_t current_action_;

public:

    Observation observation;

    AIController();

    action_t stateHandler();
    void noInputDataState();
    void idleState();
    action_t flyToState();
    void waitingState();
    action_t performActionState();
};