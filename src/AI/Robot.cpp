#include "Robot.h"

// Redundant as you can call Robot(-1)
Robot::Robot() {
    this->index = -1;
    this->position = point_zero;
    this->old_Position = point_zero;
    this->orientation = 0;
    this->speed = 0.33;
    this->plank = Plank();
    this->time_after_turn_start = 0;
    this->wasInteractedWith = false;
    this->visible = false;
}

Robot::Robot(int index) {
    this->index = index;
    this->position = point_zero;
    this->old_Position = point_zero;
    this->orientation = 0;
    this->speed = 0.33;
    this->plank = Plank();
    this->time_after_turn_start = 0;
    this->wasInteractedWith = false;
    this->visible = false;
}

// Static function
bool Robot::robotsAtTurnTime(float elapsed_time) {
    float time_drift = 3.0;
    float rest = fmod(elapsed_time, 20); 
    if (rest < ROBOT_TURN_TIME + time_drift) {
        return true;
    }
    return false;
}

float Robot::getTimeLastSeen(){
    return this->time_last_seen;
}

int Robot::getIndex() {
    return this->index;
}
point_t Robot::getPosition() {
    return this->position;
}
float Robot::getOrientation() {
    return this->orientation;
}
float Robot::getTimeAfterTurn() {
    return this->time_after_turn_start;
}
float Robot::getSpeed() {
    return this->speed;
}
Plank Robot::getCurrentPlank() {
    return this->plank;
}
bool Robot::getVisible() {
    return this->visible;
}

bool Robot::getWasInteractedWith() {
    return this->wasInteractedWith;
}

void Robot::setInteractedWithTrue() {
    this->wasInteractedWith = true;
}

void Robot::setVisible(bool set_value){
    this->visible = set_value;
}

bool Robot::isMoving() {
    if (this->old_Position.x == this->position.x &&
        this->old_Position.y == this->position.y) {
        return false;
    } else {
        return true;
    }
}

void Robot::update(int index, point_t new_Position, float new_Orientation, float elapsed_time, bool visible) {
    float estimated_orientation = 0;

    this->old_Position = this->position;
    this->old_Orientation = this->orientation;
    this->index = index;
    this->position = new_Position;
    this->time_after_turn_start = fmod(elapsed_time, 20);
    this->visible = visible;
    
    if (this->time_after_turn_start < ROBOT_TURN_TIME) {
        estimated_orientation = fmod(this->orientation - MATH_PI, 2*MATH_PI);
        this->plank.updatePlank(this->position, estimated_orientation, this->time_after_turn_start, ROBOT_TURN_TIME); // Will this make Plank construct a plank which the robot never will follow?
    } else {
        this->orientation = fmod(new_Orientation, 2*MATH_PI);
        this->plank.updatePlank(this->position, this->orientation, this->time_after_turn_start, ROBOT_TURN_TIME);
    }
}

void Robot::update(Robot robot){
    this->old_Position = this->position;
    this->old_Orientation = this->orientation;
    this->position = robot.getPosition();
    this->orientation = robot.getOrientation();
    this->time_after_turn_start = robot.getTimeAfterTurn();
    this->visible =  robot.getVisible();
}

Robot Robot::getRobotPositionAtTime(float elapsed_time){
    point_t point = this->plank.getRobotPositionAtTime(elapsed_time);
    Robot robot;
    robot.update(this->index, point, this->orientation, elapsed_time, true);
    return robot;
}

void Robot::setPositionOrientation(point_t position, float q) {
    this->position = position;
    this->orientation = q;
}

// When is this used? Double check that it is used the way we want.
// Adding time to time_after_turn_start makes the variable name be
// misleading, except when correcting for drift.
void Robot::addToTimer(float time) {
    this->time_after_turn_start += time;
}

std::ostream& operator<<(std::ostream &strm, const Robot &robot) {
    strm << "--- Robot ---" << std::endl
    << "Index: "            << robot.index                 << std::endl
    << "Position: "         << robot.position              << std::endl
    << "Old pos.: "         << robot.old_Position          << std::endl
    << "Orientation: "      << robot.orientation           << std::endl
    << "Old orient.: "      << robot.old_Orientation       << std::endl
    << "Time after: "       << robot.time_after_turn_start << std::endl
    << "Speed: "            << robot.speed                 << std::endl
    << "Interacted With: "  << robot.wasInteractedWith     << std::endl
    << "Current plank: "    << robot.plank
    << "-------------"                                      << std::endl;
    return strm;
};
