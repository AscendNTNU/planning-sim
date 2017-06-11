#include "World.h"
#include "Robot.h"

Robot::Robot(){
	this->index = 0;
	this->position = point_Zero;
	this->old_Position = point_Zero;
	this->orientation = 0;
	this->time_After_Turn = fmod(world->getCurrentTime(), 20); // Seconds after beginning of turn. When 20 it will start to turn again
	this->speed = 0.33;
	this->current_Plank = new Plank();
}

int Robot::getIndex(){
	return this->index;
}

point_t Robot::getPosition(){
	return this->position;
}
float Robot::getOrientation(){
	return this->orientation;
}
float Robot::getTimeAfterTurn(){
	return this->time_After_Turn;
}
float Robot::getSpeed(){
	return this->speed;
}
Plank* Robot::getCurrentPlank(){
	return this->current_Plank;
}

bool Robot::isMoving(){
	if (this->old_Position.x == this->position.x && 
	    this->old_Position.y == this->position.y) {
	    return false;
	} else {
		return true;
	}
}

void Robot::update(int index, point_t new_Position, float new_Orientation){
	this->old_Position = this->position;
	this->old_Orientation = this->orientation;
	this->time_After_Turn = fmod(world->getCurrentTime(), 20);
	this->index = index;
	this->position = new_Position;
	this->orientation = new_Orientation;
	this->current_Plank->updatePlank(this->position, this->orientation, this->time_After_Turn, 10);
}

void Robot::setPositionOrientation(point_t position, float q){
	this->position = position;
	this->orientation = q;
}

void Robot::addToTimer(float time){
	this->time_After_Turn += time;
}

std::ostream& operator<<(std::ostream &strm, const Robot &robot) {

    float orientation = fmod(robot.orientation,MATH_PI);
    float old_Orientation = fmod(robot.old_Orientation,MATH_PI);

    strm << "--- Robot ---" << std::endl
    << "Index: "   		<< robot.index 			<< std::endl
    << "position: "   	<< robot.position 		<< std::endl
    << "Old pos.: "     << robot.old_Position 	<< std::endl
    << "Orientation: "  << orientation 			<< std::endl
    << "Old orient.: "  << old_Orientation		<< std::endl
    << "Time after: "	<< robot.time_After_Turn<< std::endl
    << "Speed: " 		<< robot.speed 			<< std::endl
    << "Current plank: "<< *robot.current_Plank
    << "-------------"	<< std::endl;
    return strm;
};


















