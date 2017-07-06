#include "State.h"

State::State(){
	this->drone = new Drone();
	
	for(int i = 0; i<10; i++){
		this->robots[i] = new Robot();
	}
	
	for(int i = 0; i<4; i++){
		this->obstacles[i] = new Robot();
	}
	this->time_Stamp = 0;
}

Drone* State::getDrone(){
	return this->drone;
}
Robot* State::getRobot(int index){
	return this->robots[index];
}
Robot* State::getObstacle(int index){
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
	this->time_Stamp = observation.elapsed_time;
	this->drone->update(observation);
	return true;
}

bool State::updateRobotState(observation_t observation, float elapsed_time){
	point_t position = point_Zero;
	this->time_Stamp = observation.elapsed_time;
	for(int i = 0; i < 10; i++){
		position = (point_t){.x = observation.robot_x[i], .y = observation.robot_y[i]};
		this->robots[i]->update(i, position, observation.robot_q[i], this->time_Stamp);
	}
	for(int i = 0; i < 4; i++){
		position = (point_t){.x = observation.obstacle_x[i], .y = observation.obstacle_y[i]};
		this->obstacles[i]->update(i, position, observation.obstacle_q[i], this->time_Stamp);
	}
	return true;
}