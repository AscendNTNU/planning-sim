#include "Robot.h"

// Redundant as you can call Robot(-1)
Robot::Robot() {
    this->index = -1;
    this->position = point_Zero;
    this->old_Position = point_Zero;
    this->orientation = 0;
    this->speed = 0.33;
    this->current_Plank = Plank();
    this->time_After_Turn_Start = 0;
    this->wasInteractedWith = false;
    this->visible = false;
}

Robot::Robot(int index) {
    this->index = index;
    this->position = point_Zero;
    this->old_Position = point_Zero;
    this->orientation = 0;
    this->speed = 0.33;
    this->current_Plank = Plank();
    this->time_After_Turn_Start = 0;
    this->wasInteractedWith = false;
    this->visible = false;
}

// Static function
bool Robot::robotsAtTurnTime(float elapsed_time) {
    float time_drift = 3.0;//(elapsed_time * 3.0)/600.0;
    //std::cout << "timedrift: " << time_drift << std::endl;
    float rest = fmod(elapsed_time, 20); 
    if (rest < ROBOT_TURN_TIME + time_drift) {
        return true;
    }
    return false;
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
    return this->time_After_Turn_Start;
}
float Robot::getSpeed() {
    return this->speed;
}
Plank Robot::getCurrentPlank() {
    return this->current_Plank;
}
bool Robot::getVisibility() {
    return this->visible; //true
}
void Robot::setVisibility(bool visible) {
    this->visible = visible;
}

bool Robot::approaching(point_t point) {
    float old_dist = pow(pow(point.x - this->old_Position.x,2) + pow(point.y - this->old_Position.y,2), 0.5);
    float new_dist = pow(pow(point.x - this->position.x,2) + pow(point.y - this->position.y,2), 0.5);

    return (new_dist < old_dist);
}

bool Robot::getWasInteractedWith() {
    return this->wasInteractedWith;
}

void Robot::setInteractedWithTrue() {
    this->wasInteractedWith = true;
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
    this->time_After_Turn_Start = fmod(elapsed_time, 20);
    this->visible = visible;
    
    if (this->time_After_Turn_Start < ROBOT_TURN_TIME) {
        estimated_orientation = fmod(this->orientation - MATH_PI, 2*MATH_PI);
        this->current_Plank.updatePlank(this->position, estimated_orientation, this->time_After_Turn_Start, ROBOT_TURN_TIME); // Will this make Plank construct a plank which the robot never will follow?
    } else {
        this->orientation = fmod(new_Orientation, 2*MATH_PI);
        this->current_Plank.updatePlank(this->position, this->orientation, this->time_After_Turn_Start, ROBOT_TURN_TIME);
    }
}

void Robot::setPositionOrientation(point_t position, float q) {
    this->position = position;
    this->orientation = q;
}

// When is this used? Double check that it is used the way we want.
// Adding time to time_After_Turn_Start makes the variable name be
// misleading, except when correcting for drift.
void Robot::addToTimer(float time) {
    this->time_After_Turn_Start += time;
}

std::ostream& operator<<(std::ostream &strm, const Robot &robot) {
    strm << "--- Robot ---" << std::endl
    << "Index: "            << robot.index                 << std::endl
    << "Position: "         << robot.position              << std::endl
    << "Old pos.: "         << robot.old_Position          << std::endl
    << "Orientation: "      << robot.orientation           << std::endl
    << "Old orient.: "      << robot.old_Orientation       << std::endl
    << "Time after: "       << robot.time_After_Turn_Start << std::endl
    << "Speed: "            << robot.speed                 << std::endl
    << "Interacted With: "  << robot.wasInteractedWith     << std::endl
    << "Current plank: "    << robot.current_Plank
    << "-------------"                                      << std::endl;
    return strm;
};
