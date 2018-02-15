#pragma once

#include <iostream>
#include <list>

class PlanningFSM;

///Abstract interface class inherited by all states
/*
NOTE:
FSM is not async so do not run any blocking code 
in any of these methods.
EventData is passed by reference and is NOT guaranteed to remain in scope.
DO NOT store event data by reference.
States should handle all exceptions.
Unhandled exceptions will be catched by try-catch in fsm loop, but it can lead to
undefined behaviour. All state methods should gurantee nothrow!
*/
class StateInterface;
class StateInterface {
private:

    /**
     * get vector of all instianciated states
     * This idiom is a workaround for the "static initialiazation fiasco"
     * @return vector of instanciated states
     */

    bool is_ready_ = false;

    static std::list<StateInterface*>* getAllStatesVector();

    ///Assigmnet operator should be removed
    StateInterface& operator=(const StateInterface&) = delete;

public:

    ///Constructor
    StateInterface() {  getAllStatesVector()->push_back(this); }

    ///States should never be copied
    StateInterface(const StateInterface&) = delete;

    ///Virtual destructor - override if needed
    virtual ~StateInterface() {}
     
    ///Runs on current state AFTER transition
    /**stateBegin is only implemented if needed by state.*/
    virtual void stateBegin(PlanningFSM& fsm) {}

    ///Runs on current state BEFORE transition
    /**stateEnd is only implemented if needed by state*/
    virtual void stateEnd(PlanningFSM& fsm) {}
    
    ///Runs state specific code
    /**loopState is only implemented if needed by state*/
    virtual void loopState(PlanningFSM& fsm) {}
    
    ///Should return name of the state - used for debugging purposes
    virtual std::string getStateName() const = 0;
    
    ///Static interface returning iterator to first state
    static std::list<StateInterface*>::const_iterator cbegin() { return getAllStatesVector()->cbegin(); }
    
    ///Static interface returning iterator to last + 1 state
    static std::list<StateInterface*>::const_iterator cend() { return getAllStatesVector()->cend(); }
  
    ///Returns number of instanciated states
    static size_t getNumStates() { return getAllStatesVector()->size(); }

    ///Used for state setup - remember to implement isReady if overriding
    virtual void stateInit(PlanningFSM& fsm) { is_ready_ = true; }
};
