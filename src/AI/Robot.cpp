#include "Robot.h"

Robot::Robot() {
    this->index = -1;
    this->position = point_Zero;
    this->old_Position = point_Zero;
    this->orientation = 0;
    this->speed = 0.33;
    this->current_Plank = Plank();
    this->time_After_Turn_Start = 0;
    this->visible = true;
    this->wasInteractedWith = false;
}

Robot::Robot(int index) {
    this->index = index;
    this->position = point_Zero;
    this->old_Position = point_Zero;
    this->orientation = 0;
    this->speed = 0.33;
    this->current_Plank = Plank();
    this->time_After_Turn_Start = 0;
    this->visible = true;
    this->wasInteractedWith = false;
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

bool Robot::getWasInteractedWith() {
    return this->wasInteractedWith;
}

void Robot::setInteractedWithTrue() {
    printf("SETTING INTERACTED WITH TO TRUE!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! \n");
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
    
    if (this->time_After_Turn_Start < 2) {
        estimated_orientation = fmod(this->orientation - MATH_PI, 2*MATH_PI);
        this->current_Plank.updatePlank(this->position, estimated_orientation, this->time_After_Turn_Start); // Will this make Plank construct a plank which the robot never will follow?
    } else {
        this->orientation = fmod(new_Orientation, 2*MATH_PI);
        this->current_Plank.updatePlank(this->position, this->orientation, this->time_After_Turn_Start);
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
