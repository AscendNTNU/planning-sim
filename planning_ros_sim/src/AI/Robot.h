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
	//constructors
	Robot();
	Robot(int index);
	Plank* current_Plank;

	//get
	int getIndex();
	point_t getPosition();
	float getOrientation();
	float getTimeAfterTurn();
	float getSpeed();
	Plank* getCurrentPlank();

	//set
	void setPositionOrientation(point_t position, float q);
	void addToTimer(float time);
	
	//methods
	action_t chooseAction(Robot target);
	bool isMoving();
	void update(int index, point_t position,float q, float elapsed_time);
	friend std::ostream& operator<<(std::ostream &strm, const Robot &robot);
};