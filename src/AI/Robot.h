/**
@class Robot
@brief Robot class

This class handles all Robot functions. This includes getters and setters and checks.
*/

#pragma once

#include <cmath>
#include "structs.h"
#include "Plank.h"

class Robot{
private:
	int index;
	point_t position;
	float orientation;
	point_t old_Position;
	float old_Orientation;
	float time_After_Turn_Start;
	float speed;	
public:
	///Robot constructors
	Robot();
	Robot(int index);
	Plank current_Plank;

	/**
	@brief Gets the index associated with the robot instance.
	@return The robot index
	*/
	int getIndex();

	/**
	@brief Gets the current position for the robot
	@return The current position
	*/
	point_t getPosition();

	/**
	@brief Gets the current orientation
	*/
	float getOrientation();
	float getTimeAfterTurn();
	float getSpeed();
	Plank getCurrentPlank();

	//set
	void setPositionOrientation(point_t position, float q);
	void addToTimer(float time);
	void update(int index, point_t position,float q, float elapsed_time);
	
	//methods
	bool isMoving();
	friend std::ostream& operator<<(std::ostream &strm, const Robot &robot);
};
