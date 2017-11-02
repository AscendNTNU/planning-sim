#include "State.h"

State::State(){
	this->drone = Drone();
	
	for(int i = 0; i<10; i++){
		this->robots[i] = Robot();
	}
	
	for(int i = 0; i<4; i++){
		this->obstacles[i] = Robot();
	}
	this->time_Stamp = 0;
}

Drone State::getDrone(){
	return this->drone;
}
Robot State::getRobot(int index){
	if(index < 0 || index > 10){
		return Robot(-1);
	}
	return this->robots[index];
}
Robot State::getObstacle(int index){
	if(index < 0 || index > 4){
		return  Robot(-1);
	}
	return this->obstacles[index];
}

float State::getTimeStamp(){
	return this->time_Stamp;
}


bool State::updateState(observation_t observation, float elapsed_time){
	bool drone_Updated = updateDroneState(observation, elapsed_time);
	bool robot_Updated = updateRobotState(observation, elapsed_time);
	return (drone_Updated and robot_Updated);
}

bool State::updateDroneState(observation_t observation, float elapsed_time){

	this->time_Stamp = elapsed_time;
	this->drone.update(observation);
	return true;
}

bool State::updateRobotState(observation_t observation, float elapsed_time){
	point_t position = point_Zero;
	this->time_Stamp = elapsed_time;
	for(int i = 0; i < 10; i++){
		position = (point_t){.x = observation.robot_x[i], .y = observation.robot_y[i]};
		this->robots[i].update(i, position, observation.robot_q[i], this->time_Stamp);
	}
	for(int i = 0; i < 4; i++){
		position = (point_t){.x = 0, .y = 0};
		this->obstacles[i].update(i, position, 0, this->time_Stamp);
	}
	return true;
}