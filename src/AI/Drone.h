
/**
@class Drone
@brief Drone class
This calss holds the information for the drone as updated by the simulator and the AI.
It also includes functions for determining the drones relation to other parts of the world (e.g. points and ground robots).
*/


#include <cmath>
#include <iostream>
#include "structs.h"
#include "Robot.h"

class Drone{
private:
	point_t position;
	double orientation;
	double angle_Of_Motion;
	double speed;

	bool command_Done;

	point_t prev_Position;
	double prev_Orientation;

public:
	Drone();
	///return position of drone on the field
	point_t getPosition();

	///update drone based on the new observation from the sim
	bool update(observation_t observation);


	//actions
	void wait(double time);

	/**
	@brief Calculates and returns the straight line distance between the drones current position and the input point
	@param point - point struct to calculate distance to
	@return double representing distance in meters between the drone and the given point
	*/
	double getDistanceToPoint(point_t point);
	
	// double getTravelTimeToPoint(point_t point);
	
	/**
	@brief Finds the earliest point where the drone and the robot can intercept
	@param robot - a robot object refering to the given robot
	@return a point struct defining the earliest possible intecpet point
	*/
	point_t getInterceptPoint(Robot robot);
};