#include "Robot.h"
#include <cmath>

// Redundant as you can call Robot(-1)
Robot::Robot():Robot(-1) {
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

    this->side_camera = false;
    //this->prev_pos_update = 0;
    this->time_between_updates = 1;
}

// Static function
bool Robot::robotsAtTurnTime(double elapsed_time) {
    double time_drift = 3.0;//(elapsed_time * 3.0)/600.0;
    double rest = fmod(elapsed_time, 20); 
    if (rest < ROBOT_TURN_TIME + time_drift) {
        return true;
    }
    return false;
}

double Robot::getTimeLastSeen(){
    return this->time_last_seen;
}

int Robot::getIndex() {
    return this->index;
}
point_t Robot::getPosition() {
    return this->position;
}
double Robot::getOrientation() {
    return this->orientation;
}
double Robot::getTimeAfterTurn() {
    return this->time_after_turn_start;
}
double Robot::getSpeed() {
    return this->speed;
}
Plank Robot::getCurrentPlank() {
    return this->plank;
}
bool Robot::getVisible() {
    return this->visible;
}

bool Robot::getSideCamera(){
    return this->side_camera;
}

void Robot::setSideCamera(bool value){
    this->side_camera = value;
}

bool Robot::approaching(point_t point) {
    double old_dist = pow(pow(point.x - this->old_Position.x,2) + pow(point.y - this->old_Position.y,2), 0.5);
    double new_dist = pow(pow(point.x - this->position.x,2) + pow(point.y - this->position.y,2), 0.5);

    return (new_dist < old_dist);
}

bool Robot::getWasInteractedWith() {
    return this->wasInteractedWith;
}

void Robot::setIndex(int index){
    this->index = index;
}

void Robot::setOrientation(float angle){
    this->orientation = angle;
}

void Robot::setInteractedWithTrue() {
    this->wasInteractedWith = true;
}

void Robot::setVisible(bool set_value){
    this->visible = set_value;
}

bool Robot::isInArena(){

    bounds_t bounds = world.getBounds();

    double out_limit = 0.3;
    if(this->position.x < 0 - out_limit  || this->position.x > bounds.x_max + out_limit){
        return false;
    }
    else if(this->position.y > bounds.y_max + out_limit || this->position.y < 0 - out_limit){
        return false;
    }
    return true;
}

bool Robot::isMoving() {
    double dist_threshold = this->speed * this->time_between_updates - 0.1; // distance normally driven in 1sec
    double dist = pow(pow(this->position.x - this->old_Position.x,2) + pow(this->position.y - this->old_Position.y,2), 0.5);
    
    //std::cout << "dist_threshold: " << dist_threshold << std::endl;
    //std::cout << "dist: " << dist << std::endl;

    if (dist < dist_threshold) {
        //std::cout << "ROBOT " << index << " TURNING" << std::endl;
        return false;
    }
    else {
        return true;
    }
}

void Robot::update(int index, point_t new_Position, double new_Orientation, double elapsed_time, bool visible) {
    double estimated_orientation = 0;
    int planning_ros_rate = 20;

    this->pos_queue.push(new_Position); // push_back
    this->orientation_queue.push(fmod(new_Orientation, 2*MATH_PI));

    if (pos_queue.size() >= planning_ros_rate * this->time_between_updates) {
        this->old_Position = pos_queue.front();
        this->pos_queue.pop(); // pop_front
        this->old_Orientation = orientation_queue.front();
        this->orientation_queue.pop(); // pop_front
    }

    this->position = pos_queue.back();

    this->orientation = orientation_queue.back();

    this->index = index;
    this->time_after_turn_start = fmod(elapsed_time, 20);
    this->time_last_seen = elapsed_time;
    this->visible = visible;

    if (this->time_after_turn_start < ROBOT_TURN_TIME) {
        estimated_orientation = fmod(this->orientation - MATH_PI, 2*MATH_PI);
        this->plank.updatePlank(this->position, estimated_orientation, this->time_after_turn_start, ROBOT_TURN_TIME); // Will this make Plank construct a plank which the robot never will follow?
    } else {
        this->plank.updatePlank(this->position, this->orientation, this->time_after_turn_start, ROBOT_TURN_TIME);
    }
}

void Robot::update(Robot robot){

    this->old_Position = this->position;
    this->old_Orientation = this->orientation;
    this->position = robot.getPosition();
    this->side_camera = robot.getSideCamera();
    this->orientation = fmod(robot.getOrientation(), 2*MATH_PI);

    this->time_after_turn_start = robot.getTimeAfterTurn();
    this->time_last_seen = robot.getTimeLastSeen();
    this->visible =  robot.getVisible();

}

Robot Robot::getRobotPositionAtTime(double elapsed_time){
    point_t point = this->plank.getRobotPositionAtTime(elapsed_time);
    Robot robot;
    robot.update(this->index, point, this->orientation, elapsed_time, true);
    return robot;
}

void Robot::setPositionOrientation(point_t position, double q) {
    this->position = position;
    this->orientation = q;
}

// When is this used? Double check that it is used the way we want.
// Adding time to time_after_turn_start makes the variable name be
// misleading, except when correcting for drift.
void Robot::addToTimer(double time) {
    this->time_after_turn_start += time;
}

float Robot::getOrientationFromPositionHistory(){
    return atan2(position.y-old_Position.y, position.x-old_Position.x);
}

bool Robot::recentlySeen(float time){
    if(time-time_last_seen < 0.5){
        return true;
    }
    return false;
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
