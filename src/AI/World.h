
/**
@class World
@brief World class
This calss creates the playing field "World" that the drone lives in. 
It is responsible for creating acoordinate system and appropriate reward values along the course.
*/

#pragma once

#include "structs.h"
#include <time.h>
#include <cmath>

class World{
private:
	point_t origin;
	float orientation;
	bounds_t bounds;

public:
	//Constructors
	/**
    @brief Contructor for the grid world.

    @param orientation - float value describing the orientation of the field.
	*/
	World(float orientation);

	///returns the point defined as the origin of the world
	point_t getOrigin();

	///returns the orientation of the field (deg or rad?)
	float getOrientation();
	
	///returns the bounds of the field
	bounds_t getBounds();

	/**
    @brief 2D function of grid value. Determined using value iteration.

    @param x - The x position on the field
    @param y - The y position on the field
    @return the points value at the (x, y) position given
	*/
	float getGridValue(float X, float Y);
};