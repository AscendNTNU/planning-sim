#pragma once
#include "Robot.h"
#include "Drone.h"
#include <array>

class Observation{
private:
	double time_Stamp;
	std::array<Robot,10> robots;
	std::array<Robot,4> obstacles;
	Drone drone;
	bool any_robots_visible;
public:

	Observation();

	Drone getDrone();

	Robot getRobot(int index);
	std::array<Robot,10> getRobots();

	Robot getObstacle(int index);
	std::array<Robot,4> getObstacles();

	double getTimeStamp();

	bool anyRobotsVisible();

	bool update(observation_t observation, double elapsed_time);
	bool updateDrone(observation_t observation, double elapsed_time);
	bool updateRobot(observation_t observation, double elapsed_time);
	void updateInteraction(int index);
};