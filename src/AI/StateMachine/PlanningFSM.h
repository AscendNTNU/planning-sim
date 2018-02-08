#ifndef CONTROL_FSM_HPP
#define CONTROL_FSM_HPP

#include "StateInterface.h"
#include "NoInputState.h"
#include "IdleState.h"

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
    static PrestartState PREFLIGHT_STATE;
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

    ///Current drone position
    struct {
        //Not all states needs direct access to position and flags
        friend class PlanningFSM;
        friend class EstimateAdjustState;
    private:
        geometry_msgs::PoseStamped position;
        bool is_set = false;
        bool valid_xy = true; //Assumes XY is valid if not set otherwise
    } drone_position_;

    ///Struct holding information about drones state
    struct {
        bool is_offboard = false;
        bool is_armed = false;
        bool is_preflight_completed = false;
    } drone_state_;

    ///Has FSM been initiated?
    bool states_is_ready_ = false;

    ///Callback when a transition is made
    std::function<void()> on_state_changed_ = [](){};

    ///Copy constructor deleted
    PlanningFSM(const PlanningFSM&) = delete;

    ///Assignment operator deleted
    PlanningFSM& operator=(const PlanningFSM&) = delete;

    ///Shared nodehandle for all states
    ros::NodeHandle node_handler_;

    ///Struct holding all shared PlanningFSM ros subscribers
    struct {
        friend class PlanningFSM;
    private:
        ros::Subscriber mavros_state_changed_sub;
    } subscribers_;

    ///Callback for mavros state changed
    void mavrosStateChangedCB(const mavros_msgs::State& state);

    ///Initializes all states
    void initStates();

    ///All states needs access to obstacle avoidance
    control::ObstacleAvoidance obstacle_avoidance_;


protected:
    /**
     * @brief Changes the current running state
     * @details Allows the current running state to change the current state pointer
     * @param state Which state instance to transition to0
     * @param caller_p Which state that requests the transition
     * @param event Which event triggered the transition request
     */
    void transitionTo(StateInterface& state, StateInterface* caller_p, const EventData& event);
    
public:
    ///Constructor sets default/starting state
    PlanningFSM();
    ///Constructor using custom obstacle avoidance
    PlanningFSM(control::ObstacleAvoidance ob) : PlanningFSM() { obstacle_avoidance_ = ob; }
    ///Destructor not used to anything specific.
    ~PlanningFSM() {}

    ///Get pointer to the current running state
    StateInterface* getState() { return state_vault_.current_state_p_; }
    
    /**
     * @brief Handles incoming (external) events
     * @details Events are sent to current running state
     * @param event Information about the external event
     */
    void handleEvent(const EventData& event);
    
    ///Loops the current running state
    void loopCurrentState(void);

    
    ///Returns setpoint from current state
    mavros_msgs::PositionTarget getMavrosSetpoint();

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

