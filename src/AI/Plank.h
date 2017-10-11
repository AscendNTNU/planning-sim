/**
@class Plank
@brief Plank class

This class creates the plank describing a ground robots motion given no random
movement. It is responisble for setting value on a ground robots position.
*/

#pragma once
#include <iostream>
#include "structs.h"
#include "World.h"

extern World world;

class Plank{
	private:
	    point_t end_point; //First endpoint robot meets
	    point_t start_point; //Second endpoint robot meets
	    point_t[10] plank_points; //Points between endpoints
	    float length;
	    float reward;
	    float angle;
	public:
		///Plank constructor for a robot at (0,0)
		Plank();

		///Plank constructor for a robot at a given position/time
		Plank(point_t position, float angle, float time_After_Turn_Start, int num_Iterations);

		///Get reward of current plank
		float getReward();

		///Get angle of current plank
		float getAngle();

		///Get length of current plank
		float getLength();

		///@brief Get a point on the plank
		///There are 12 points on the plank. The start point 0, the end point 11 and 10 points between
		point_t getPoint(int i);
		
		/**
		@brief Check if the plank crosses the green line
	    @return Boolean
	    Returns true if the robot will cross the green line given it's current motion.
		*/
		bool willExitGreen();

		/**
		@brief Check if the plank crosses the red line
	    @return Boolean
	    Returns true if the robot will cross any red line given it's current motion.
		*/
		bool willExitRed();

		/**
		@brief Calculate position of 10 equispaced points on the plank
	    Splits the plank into 10 points that are equally spaced. This is for reward calculation
	    and for iterating through the plank for decision making.
		*/		
		void calculateAllPlankPoints()


		/**
		@brief Calculate the reward of a plank
	    @param n Number of integration steps
	    @return Reward value of the plank
	    Calculates the value of a plank by integrating the plank over the world value
	    grid.
		*/
		float calculateReward(int n);

		/**
		@brief Updates the plank given the ground robots state.
		@param position Robot position
		@param angle Robot angle in radians
		@param time_After_Turn_Start Seconds passed since the robot started turning
		*/
		void updatePlank(point_t position, float angle, float time_After_Turn_Start, int num_Iterations);
		
		///Check if a point is outside of the plank
		/**
		@brief Check if a point is outside of the plank.
		@param point Point to check
		@return Boolean
		*/
		bool pointIsOutsideOfPlank(point_t point);

	    friend std::ostream& operator<<(std::ostream &strm, const Plank &plank);
};