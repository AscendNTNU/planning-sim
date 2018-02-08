#include "control/fsm/control_fsm.hpp"
#include <ros/ros.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include ""

BeginState PlanningFSM::BEGIN_STATE;
PrestartState PlanningFSM::PREFLIGHT_STATE;
NoInputState PlanningFSM::NO_INPUT_STATE;
IdleState PlanningFSM::IDLE_STATE;
PositioningState PlanningFSM::POSITIONING_STATE;
SearchState PlanningFSM::SEARCH_STATE;
LandOnTopState PlanningFSM::LAND_ON_TOP_STATE;
LandInFrontState PlanningFSM::LAND_IN_FRONT_STATE;
MissionCompleteState PlanningFSM::MISSION_COMPLETE_STATE;

//Change the current running state - be carefull to only change into an allowed state
//Due to poor design, transitionTo has no strong nothrow guarantees - not exception safe!!
//Will lead to undefined behaviour if exception is thrown
void PlanningFSM::transitionTo(StateInterface& state, StateInterface* caller_p, const EventData& event) {
    //Only current running state is allowed to change state
    if(getState() == caller_p) {
        //Run stateEnd on current running state before transitioning
        getState()->stateEnd(*this, event);
        //Set the current state pointer
        state_vault_.current_state_p_ = &state;
        control::handleInfoMsg("Current state: " + getState()->getStateName());
        //Pass event to new current state
        getState()->stateBegin(*this, event);
        //Notify state has changed
        on_state_changed_();
    } else {
        control::handleErrorMsg("Transition request made by not active state");
    }
}

//Send external event to current state and to "next" state
void PlanningFSM::handleEvent(const EventData& event) {
    if(getState() == nullptr) {
        control::handleCriticalMsg("Bad implementation of FSM - FSM allways need a state");
        return;
    }
    if(event.event_type == EventType::MANUAL) {
        //If drone entered manual mode: Abort current operation, and go to stable.
        this->handleManual();
    }
    //Pass event to current running state
    getState()->handleEvent(*this, event);
}

//Runs state specific code on current state
void PlanningFSM::loopCurrentState(void) {
    try {
        assert(getState() != nullptr);
        getState()->loopState(*this);
    } catch(const std::exception& e) {
        //If exceptions aren't handled by states - notify and try to go to blind hover
        //Will lead to undefined behaviour- but still safer than nothing!
        control::handleCriticalMsg(e.what());
        RequestEvent abort_event(RequestType::ABORT);
        transitionTo(BLIND_HOVER_STATE, getState(), abort_event);
    }
}

PlanningFSM::PlanningFSM() {
    //Set starting state
    state_vault_.current_state_p_ = &BEGIN_STATE;

    //Initialize all states
    this->initStates();

    //Subscribe to neccesary topics
    std::string& stateTopic = control::Config::mavros_state_changed_topic;
    subscribers_.mavros_state_changed_sub = node_handler_.subscribe(stateTopic, 1, &PlanningFSM::mavrosStateChangedCB, this);
}

void PlanningFSM::initStates() {
    //Only init once
    if(states_is_ready_) return;
    for(auto it = StateInterface::cbegin(); it != StateInterface::cend(); it++) {
        (*it)->stateInit(*this);
    }
    states_is_ready_ = true;
}

bool PlanningFSM::isReady() {
    //Only check states if not already passed
    if(drone_state_.is_preflight_completed) return true;

    //All states must run their own checks
    for(auto it = StateInterface::cbegin(); it != StateInterface::cend(); it++) {
        control::handleInfoMsg((*it)->getStateName() + " is testing");
        if(!(*it)->stateIsReady(*this)) return false;
    }
    //Make sure obstacle avoidance is ready if enabled
    if(control::Config::require_obstacle_detection) {
        //Obstacle avoidance must be ready
        if(!obstacle_avoidance_.isReady()) {
            control::handleWarnMsg("Preflight Check: Obstacle avoidance not ready!");
            return false;
        }
    }

    //Some checks can be skipped for debugging purposes
    if(control::Config::require_all_data_streams) {
        try {
            //Check that we're recieving position
            if(!control::DroneHandler::isPoseValid()) {
                control::handleWarnMsg("Preflight Check: No valid pose data");
                return false;
            }
            //Mavros must publish state data
            if (subscribers_.mavros_state_changed_sub.getNumPublishers() <= 0) {
                control::handleWarnMsg("Preflight Check: No valid mavros state data!");
                return false;
            } 
            //Land detector must be ready
            if (!LandDetector::getSharedInstancePtr()->isReady()) {
                control::handleWarnMsg("Preflight Check: No valid land detector data!");
                return false;
            }
        } catch(const std::exception& e) {
            ///Critical bug -
            control::handleCriticalMsg(e.what());
            return false;
        }

        if(control::Config::require_obstacle_detection) {
            try {
                using control::ObstacleStateHandler;
                //Land detector must be ready
                if (!ObstacleStateHandler::isInstanceReady()) {
                    control::handleWarnMsg("Missing obstacle state stream!");
                    return false;
                }
            } catch(const std::exception& e) {
                control::handleErrorMsg("Exception: " + std::string(e.what()));
                return false;
            }
        }
    }

    //Preflight has passed - no need to check it again.
    drone_state_.is_preflight_completed = true;
    return true;
}

void PlanningFSM::startPreflight() {
    if(!isReady()) {
        control::handleWarnMsg("FSM not ready, can't transition to preflight!");
        return;
    }
    if(getState() == &BEGIN_STATE) {
        RequestEvent event(RequestType::PREFLIGHT);
        transitionTo(PREFLIGHT_STATE, &BEGIN_STATE, event);
    } else {
        std::string err_msg = "Can't transition to preflight from ";
        err_msg += getState()->getStateName();
        control::handleErrorMsg(err_msg);
    }
}