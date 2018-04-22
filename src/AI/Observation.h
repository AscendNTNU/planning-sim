#pragma once
#include "Robot.h"
#include "Drone.h"
#include <array>

using planning::Config;

class Observation{
private:
	float time_Stamp;
	std::vector<Robot> robots;
	std::vector<Robot> obstacles;
	Drone drone;
	bool any_robots_visible;
public:

	Observation();

	Drone getDrone();

	Robot getRobot(int index);
	std::vector<Robot> getRobots();

	Robot getObstacle(int index);
	std::vector<Robot> getObstacles();

	float getTimeStamp();

	bool anyRobotsVisible();

	bool update(observation_t observation, float elapsed_time);
	bool updateDrone(observation_t observation, float elapsed_time);
	bool updateRobot(observation_t observation, float elapsed_time);
	void updateInteraction(int index);
};