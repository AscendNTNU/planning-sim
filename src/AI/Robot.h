/**
@class Robot
@brief Robot class

This class handles all Robot functions. This includes getters and setters and checks.
*/

#pragma once

#include <cmath>
#include "structs.h"
#include "Plank.h"

const float ROBOT_TURN_TIME = 2.5;

class Robot{
private:
    int index;
    point_t position;
    float orientation;
    point_t old_Position;
    float old_Orientation;
    float time_After_Turn_Start; ///< Is the elapsed time passed since the start of the last rotation/turn.
                                 ///< Remember that the robot turns for about 2 seconds.
    float speed;
    bool visible;
public:
    ///Robot constructors
    Robot();
    Robot(int index);
    Plank current_Plank;

    static bool robotsAtTurnTime(float elapsed_time);

    /**
    @brief Get the index associated with the robot instance.
    @return The robot index
    */
    int getIndex();

    /**
    @brief Get the current position for the robot.
    @return The current position as a point_t struct
    */
    point_t getPosition();

    /**
    @brief Get the robots current orientation relative to the grid.
    @return Orientation of the robot
    */
    float getOrientation();

    /**
    @brief Get the elapsed time passed since it last started turning/rotating.
    @return Time passed since last turn started
    */
    float getTimeAfterTurn();

    /**
    @brief Get the current speed of the Robot
    @return Current speed of the Robot
    */
    float getSpeed();

    /**
    @brief Get the current plank of the Robot
    */
    Plank getCurrentPlank();

    /**
    @brief Get the visibility of the Robot
    @return If it is visible
    */
    bool getVisibility();

    /**
    @brief Set the position and orientation of the Robot
    @param position Point struct of the new position
    @param q The new orientational angle in radians
    */
    void setPositionOrientation(point_t position, float q);

    /**
    @brief Add time to the robots time 
    @param time Time to add to robots elapsed time since the last turn started
    */
    void addToTimer(float time);
    void update(int index, point_t position,float q, float elapsed_time, bool visible);
    
    /**
    @brief Checks if the Robot is moving. Is often equivalent to the robot turning.
    */
    bool isMoving();
    friend std::ostream& operator<<(std::ostream &strm, const Robot &robot);
};
