#ifndef PLANNING_FSM_HPP
#define PLANNING_FSM_HPP

#include <functional>

#include "StateInterface.h"
#include "BeginState.h"
#include "PrestartState.h"
#include "NoInputState.h"
#include "IdleState.h"
#include "PositioningState.h"
#include "SearchState.h"
#include "LandOnTopState.h"
#include "LandInFrontState.h"
#include "MissionCompleteState.h"

#include "../AI.h"
#include "../structs.h"
///Main FSM logic
class PlanningFSM {
private:
    //Add state classes as friend classes here - allowing them to use transitionTo.
    friend class BeginState;
    friend class PrestartState;
    friend class NoInputState;
    friend class IdleState;
    friend class PositioningState;
    friend class SearchState;
    friend class LandOnTopState;
    friend class LandInFrontState;
    friend class MissionCompleteState;
    
    //Static instances of the different states
    //Also add them to allStates_ vector in constructor
    static BeginState BEGIN_STATE;
    static PrestartState PRESTART_STATE;
    static NoInputState NO_INPUT_STATE;
    static IdleState IDLE_STATE;
    static PositioningState POSITIONING_STATE;
    static SearchState SEARCH_STATE;
    static LandOnTopState LAND_ON_TOP_STATE;
    static LandInFrontState LAND_IN_FRONT_STATE;
    static MissionCompleteState MISSION_COMPLETE_STATE;

    /**
     * @brief Holds a pointer to current running state
     * @details Struct "vault" explanation:
     *    The struct (with instance _stateHolder) keeps the _pCurrentState private. 
     *    The struct friends the FSM class so the FSM class can access the pointer. 
     *    Even though other classes or function might have access to FSM private variables through friend,
     *    they still wont have access to the pointer.
     */
    struct {
        friend class PlanningFSM;
    private:
        StateInterface* current_state_p_ = nullptr; //This need to be set to a start state in constructor
    } state_vault_;

    ///Has FSM been initiated?
    bool states_is_ready_ = false;

    ///Callback when a transition is made
    std::function<void()> on_state_changed_ = [](){};

    ///Copy constructor deleted
    PlanningFSM(const PlanningFSM&) = delete;

    ///Assignment operator deleted
    PlanningFSM& operator=(const PlanningFSM&) = delete;

    ///Initializes all states
    void initStates();

    ///AI object where AI functions are called from
    AI ai_;

    //Current action that needs to be preformed by the drone/sent to control
    action_t current_action_;

    //Planned action the state machine is working towards
    action_t planned_action_;

    //Current observation
    Observation observation;

protected:
    /**
     * @brief Changes the current running state
     * @details Allows the current running state to change the current state pointer
     * @param state Which state instance to transition to0
     * @param caller_p Which state that requests the transition
     * @param event Which event triggered the transition request
     */
    void transitionTo(StateInterface& state, StateInterface* caller_p);
    
public:
        
    ///Constructor sets default/starting state
    PlanningFSM();
    ///Destructor not used to anything specific.
    ~PlanningFSM() {}

    ///Get pointer to the current running state
    StateInterface* getState() { return state_vault_.current_state_p_; }
     
    ///Loops the current running state
    void loopCurrentState(void);

    ///Sets new callback function for onStateChanged
    void setOnStateChangedCB(std::function<void()> cb) { on_state_changed_ = cb; }

    ///Checks if all states are ready
    bool isReady();

    ///Transition to preflight from begin state
    void startPreflight();

    ///Handles loss of offboard mode
    void handleManual();
};

#endif

