/**
@class AIController
@brief AIController class

This class handles the AI state machine.
*/

#include "AI.h"

///The different AI states.
enum ai_state_t{
    no_input_data, ///Starting state with no input data.
    idle, ///Nothing happening, AI waiting to find the next best action.
    positioning, /// Combo of fly_to and waiting
    land_in_front,
    land_on_top
};

class AIController{
private:
    AI ai_;
    ai_state_t state_;
    action_t current_action_;

public:

    Observation observation;

    AIController();

    /**
    @brief State machine for the AI.
    @return Either an empty action or the action to be completed by the drone.
    A switch case through the different AI states. Look at ai_state_t enum or 
    the rest of the functions in this class for details of what happens in each
    state.
    */
    action_t stateHandler();
    
    /**
    @brief Checks if there is input data.
    @return
    Checks if there is input data and if so changes state to idle. This is the starting
    state of the AI upon running the planning node.
    */
    void noInputDataState();
    
    /**
    @brief Finds a new action to complete
    @return
    This is the state after we have gotten input or after we have finished/aborted and action.
    It runs our AI functions to find the next action we want to perform and switches state to the
    fly to state.
    */
    void idleState();

    /**
    @brief Waits for the target to reach the action point then sends out the action.
    @return
    A robot interaction action. This state waits for the target to reach the action point and updates the action
    based on changes in our observation of the target. Also includes an abort action
    if the observation changes over a certain threshold.
    */
    action_t positioningState();

    /**
    @brief Sends out the action and returns to idle state.
    @return A robot interaction action.This state waits for the target to reach the action point and updates the action
    */
    action_t landOnTopState();
    
    /**
    @brief Sends out the action and returns to idle state.
    @return A robot interaction action.This state waits for the target to reach the action point and updates the action
    */
    action_t landInFrontState();
};