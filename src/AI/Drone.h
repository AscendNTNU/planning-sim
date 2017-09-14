#include <cmath>
#include <iostream>
#include "structs.h"
#include "Robot.h"

class Drone{
private:
	point_t position;
	float orientation;
	float angle_Of_Motion;
	float speed;

	bool command_Done;

	point_t prev_Position;
	float prev_Orientation;

	drone_State_t state;

public:
	Drone();
	//get
	point_t getPosition();
	drone_State_t getState();

	//update
	bool update(observation_t observation);


	//actions
	void wait(float time);
	float getDistanceToPoint(point_t point);
	// float getTravelTimeToPoint(point_t point);
	point_t getInterceptPoint(Robot* robot);
};