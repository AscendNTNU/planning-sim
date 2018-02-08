#include "control/fsm/control_fsm.hpp"

GoToState::GoToState() : StateInterface::StateInterface() {
    setpoint_.type_mask = default_mask;
}

void GoToState::stateBegin(ControlFSM& fsm) {
}

void GoToState::stateEnd(ControlFSM& fsm, const EventData& event) {
}

void GoToState::loopState(ControlFSM& fsm) {
    this->current_action_ = ai_.getBestGeneralAction(this->observation);    
    fsm.transitionTo(ControlFSM::POSITION_HOLD_STATE, this, abort_event);
}

//Returns valid setpoint
const mavros_msgs::PositionTarget* GoToState::getSetpointPtr() {
    setpoint_.header.stamp = ros::Time::now();
    return &setpoint_;
}

bool GoToState::stateIsReady(ControlFSM& fsm) {
    return true;
}
    
void GoToState::handleManual(ControlFSM& fsm) {
    cmd_.eventError("Lost OFFBOARD");
    cmd_ = EventData();
    RequestEvent manual_event(RequestType::MANUALFLIGHT);
    fsm.transitionTo(ControlFSM::MANUAL_FLIGHT_STATE, this, manual_event);
}

//Check if velocity is close enough to zero
bool droneNotMoving(const geometry_msgs::TwistStamped& target) {
    using control::Config;
    using std::pow;
    auto& t_l = target.twist.linear;
    //Calculate square velocity
    double dx_sq = pow(t_l.x, 2);
    double dy_sq = pow(t_l.y, 2);
    double dz_sq = pow(t_l.z, 2);
    return (dx_sq + dy_sq + dz_sq) < pow(Config::velocity_reached_margin, 2);
}

fsm.transitionTo(PlanningFSM::)
