#pragma once
#include "Robot.h"
#include "Drone.h"

class State{
private:
	float time_Stamp;
public:
	Robot* robots[10];
	Robot* obstacles[4];
	Drone* drone;
	
	State();

	Drone* getDrone();
	Robot* getRobot(int index);
	Robot* getObstacle(int index);
	float getTimeStamp();

	bool updateState(observation_t observation, float elapsed_time);
	bool updateDroneState(observation_t observation, float elapsed_time);
	bool updateRobotState(observation_t observation, float elapsed_time);
};