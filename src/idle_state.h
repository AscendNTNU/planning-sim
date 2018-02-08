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

//Initialize state
void GoToState::stateInit(ControlFSM& fsm) {
    using control::Config;
    //TODO Uneccesary variables - Config can be used directly
    //Set state variables
    delay_transition_.delayTime = ros::Duration(Config::go_to_hold_dest_time);

    control::handleInfoMsg("GoTo init completed!");
}

/**
 * @brief Returns a yaw that is a multiple of 90 degrees
 * @details Drone should fly as straight forward as possible
 * , but yaw should be a multiple of 90 degrees.
 * This method assumes dx and dy != 0 at the same time
 * @param dx difference in x
 * @param dy difference in y
 * @return Yaw angle in radians - not mavros corrected
 */
double calculatePathYaw(double dx, double dy) {
    //Avoid fatal error if dx and dy is too small
    //If method is used correctly this should NEVER be a problem
    if(std::fabs(dx * dx + dy * dy) < 0.001) {
        return 0;
    }
    /*
    angle = acos(([dx, dy] dot [1,0]) / (norm([dx, dy]) * norm([1,0]))) = acos(dx / (norm([dx, dy]) * 1))
    */
    double angle = std::acos(dx / std::sqrt(dx * dx + dy * dy));

    //Select closest multiple of 90 degrees
    if(angle > 3 * PI / 4) {
        angle = PI;
    } else if(angle > PI / 4) {
        angle = PI / 2.0;
    } else {
        angle = 0;
    }
    //Invert if dy is negative
    if (dy < 0) {
        angle = -angle;
    }

    return angle;
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

void GoToState::destinationReached(ControlFSM& fsm) {
    //Transition to correct state
    if(cmd_.isValidCMD()) {
        switch(cmd_.command_type) {
            case CommandType::LANDXY: {
                //If no valid twist data it's unsafe to land
                if(!control::DroneHandler::isTwistValid()) {
                    control::handleErrorMsg("No valid twist data, unsafe to land! Transitioning to poshold");
                    cmd_.eventError("Unsafe to land!");
                    cmd_ = EventData();
                    RequestEvent abort_event(RequestType::ABORT);
                    fsm.transitionTo(ControlFSM::POSITION_HOLD_STATE, this, abort_event);
                    return;
                }
                //Check if drone is moving
                if(droneNotMoving(control::DroneHandler::getCurrentTwist())) {
                    //Hold current position for a duration - avoiding unwanted velocity before doing anything else
                    if(!delay_transition_.enabled) {
                        delay_transition_.started = ros::Time::now();
                        delay_transition_.enabled = true;

                        if(cmd_.isValidCMD()) {
                            cmd_.sendFeedback("Destination reached, letting drone slow down before transitioning!");
                        }
                    }
                    //Delay transition
                    if(ros::Time::now() - delay_transition_.started < delay_transition_.delayTime) {
                        return;
                    }
                    //If all checks passed - land!
                    fsm.transitionTo(ControlFSM::LAND_STATE, this, cmd_);
                } else {
                    //If drone is moving, reset delayed transition
                    delay_transition_.enabled = false;
                }
                break;
            }
                //NOTE: Land groundrobot algorithm not implemented yet, so this is commented out
                /*
                case CommandType::LANDGB:
                    fsm.transitionTo(ControlFSM::TRACK_GB_STATE, this, cmd_);
                    break;
                */
            case CommandType::GOTOXYZ: {
                cmd_.finishCMD();
                RequestEvent done_event(RequestType::POSHOLD);
                //Attempt to hold position target
                done_event.position_goal = cmd_.position_goal;
                fsm.transitionTo(ControlFSM::POSITION_HOLD_STATE, this, done_event);
            }
                break;
            default:
                control::handleWarnMsg("Unrecognized command type");
                break;
        }
    } else {
        RequestEvent pos_hold_event(RequestType::POSHOLD);
        pos_hold_event.position_goal = cmd_.position_goal;
        fsm.transitionTo(ControlFSM::POSITION_HOLD_STATE, this, pos_hold_event);
    }

    delay_transition_.enabled = false;
}