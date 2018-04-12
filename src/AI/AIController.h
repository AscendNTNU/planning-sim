/**
@class AIController
@brief AIController class

This class handles the AI state machine.
*/

#include "AI.h"
#include "World.h"

extern World world;


///The different AI states.
enum ai_state_t{
    no_input_data, ///Starting state with no input data.
    idle, ///Nothing happening, AI waiting to find the next best action.
    positioning, /// Combo of fly_to and waiting
    land_in_front,
    land_on_top,
    mission_complete,
    no_visible_robots, /// No targets available, find them!
    take_off_state
};

inline std::string stateToString(ai_state_t state) {
    switch(state) {
        case no_input_data:
            return "No input data state";
        case idle:
            return "Idle state";
        case positioning:
            return "Positioning state";
        case land_in_front:
            return "Land in front state";
        case land_on_top:
            return "Land on top state";
        case mission_complete:
            return "Mission complete state";
        case no_visible_robots:
            return "No visible robots state";
        case take_off_state:
            return "Take off state";
    }
}

inline std::ostream& operator<<(std::ostream &strm, const ai_state_t &state) {
    strm << stateToString(state);
    return strm;
};

class AIController{
private:
    AI ai_;
    ai_state_t state_;
    action_t planned_action_;
    float prev_transition_timestamp;

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
    void transitionTo(ai_state_t state);

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

    action_t missionCompleteState();

    action_t noVisibleRobotsState();

    action_t takeOffState();
};