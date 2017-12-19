#pragma once
#include "Robot.h"
#include "Drone.h"
#include <array>

class Observation{
private:
	float time_Stamp;
	std::array<Robot,10> robots;
	std::array<Robot,4> obstacles;
	Drone drone;

public:

	Observation();

	Drone getDrone();

	Robot getRobot(int index);
	std::array<Robot,10> getRobots();

	Robot getObstacle(int index);
	std::array<Robot,4> getObstacles();

	float getTimeStamp();

	bool update(observation_t observation, float elapsed_time);
	bool updateDrone(observation_t observation, float elapsed_time);
	bool updateRobot(observation_t observation, float elapsed_time);
};