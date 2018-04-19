#pragma once
#include "Robot.h"
#include "Drone.h"
#include <array>

class Observation{
private:
	float time_Stamp;
	std::array<Robot,Config::NUMBER_OF_TARGETS> robots;
	std::array<Robot,Config::NUMBER_OF_OBSTACLES> obstacles;
	Drone drone;
	bool any_robots_visible;
public:

	Observation();

	Drone getDrone();

	Robot getRobot(int index);
	std::array<Robot,Config::NUMBER_OF_TARGETS> getRobots();

	Robot getObstacle(int index);
	std::array<Robot,Config::NUMBER_OF_OBSTACLES> getObstacles();

	float getTimeStamp();

	bool anyRobotsVisible();

	bool update(observation_t observation, float elapsed_time);
	bool updateDrone(observation_t observation, float elapsed_time);
	bool updateRobot(observation_t observation, float elapsed_time);
	void updateInteraction(int index);
};