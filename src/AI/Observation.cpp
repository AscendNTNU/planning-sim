#include "Observation.h"

using planning::Config;

Observation::Observation(): drone(Drone()), 
							robots(Config::NUMBER_OF_ROBOTS), 
							obstacles(Config::NUMBER_OF_OBSTACLES)  
{

	for(int i = 0; i<Config::NUMBER_OF_ROBOTS; i++){
		this->robots[i] = Robot();
	}
	
	for(int i = 0; i<Config::NUMBER_OF_OBSTACLES; i++){
		this->obstacles[i] = Robot();
	}
	this->time_Stamp = 0;

	this->any_robots_visible = false;
}

Drone Observation::getDrone(){
	return this->drone;
}

Robot Observation::getRobot(int index){
	if(index < 0 || index > Config::NUMBER_OF_ROBOTS){
		return Robot(-1);
	}
	return this->robots[index];
}

std::vector<Robot> Observation::getRobots(){
	return this->robots;
}


Robot Observation::getObstacle(int index){
	if(index < 0 || index > Config::NUMBER_OF_OBSTACLES){
		return  Robot(-1);
	}
	return this->obstacles[index];
}

std::vector<Robot> Observation::getObstacles(){
	return this->obstacles;
}

float Observation::getTimeStamp(){
	return this->time_Stamp;
}

bool Observation::anyRobotsVisible(){
	return this->any_robots_visible;
}


bool Observation::update(observation_t observation, float elapsed_time){
	bool drone_Updated = updateDrone(observation, elapsed_time);
	bool robot_Updated = updateRobot(observation, elapsed_time);
	return (drone_Updated and robot_Updated);
}

bool Observation::updateDrone(observation_t observation, float elapsed_time){
	this->time_Stamp = elapsed_time;
	this->drone.update(observation);
	return true;
}

void Observation::updateInteraction(int index) {
	this->robots[index].setInteractedWithTrue();
}

bool Observation::updateRobot(observation_t observation, float elapsed_time){
	bool any_robots_visible = false;
	point_t position = point_zero;
	this->time_Stamp = elapsed_time;
	for(int i = 0; i < Config::NUMBER_OF_ROBOTS; i++){ // should loop through lenght of observed robots not 10.
		position = (point_t){.x = observation.robot_x[i], .y = observation.robot_y[i]};
		this->robots[i].update(i, position, observation.robot_q[i], this->time_Stamp, observation.robot_visible[i]);
		if (observation.robot_visible[i] == true) {
			any_robots_visible = true;
		}
	}
	for(int i = 0; i < Config::NUMBER_OF_OBSTACLES; i++){
		position = (point_t){.x = 0, .y = 0};
		this->obstacles[i].update(i, position, 0, this->time_Stamp, true);
	}
	this->any_robots_visible = any_robots_visible;
	return true;
}