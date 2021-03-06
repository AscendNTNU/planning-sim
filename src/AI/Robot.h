/**
@class Robot
@brief Robot class

This class handles all Robot functions. This includes getters and setters and checks.
*/

#pragma once

#include <cmath>
#include "structs.h"
#include "Plank.h"
#include <queue>
#include <opencv2/opencv.hpp>

const double ROBOT_TURN_TIME = 2.5;

class Robot{
protected:
    int index;
    point_t position;
    double orientation;
    point_t old_Position;
    double old_Orientation;
    double time_after_turn_start; ///< Is the elapsed time passed since the start of the last rotation/turn.
                                 ///< Remember that the robot turns for about 2 seconds.
    double time_last_seen;
    //double prev_pos_update;
    double time_between_updates;
    std::queue<point_t> pos_queue;
    std::queue<double> orientation_queue;

    bool side_camera;

    double speed;
    bool visible;
    bool wasInteractedWith;
    cv::Mat F;//(3,3,DataType<double>::type);
    cv::Mat H;
    cv::Mat P_k_k;
    cv::Mat P_k_km1;
    cv::Mat P_km1_km1;
    
    cv::Mat x_hat_k_k;
    cv::Mat x_hat_km1_km1;
    cv::Mat x_hat_k_km1;

    cv::Mat R_k;
    cv::Mat Q_k;
    double t_k;
    double t_km1;
    double dt;

    double xMeasCovar;
    double yMeasCovar;
    double thMeasCovar_downCam;
    double thMeasCovar_sideCam;

    bool firstTimeTurning;
    bool firstTimeDriving;

    
public:
    
    cv::Mat x_hat_k;
    ///Robot constructors
    Robot();
    Robot(int index);
    
    Plank plank;

    static bool robotsAtTurnTime(double elapsed_time);

    /**
    @brief Get the index associated with the robot instance.
    @return The robot index
    */
    int getIndex();

    double getTimeLastSeen();

    /**
    @brief Get the current position for the robot.
    @return The current position as a point_t struct
    */
    point_t getPosition();

    /**
    @brief Get the robots current orientation relative to the grid.
    @return Orientation of the robot
    */
    double getOrientation();

    /**
    @brief Get the elapsed time passed since it last started turning/rotating.
    @return Time passed since last turn started
    */
    double getTimeAfterTurn();

    /**
    @brief Get the current speed of the Robot
    @return Current speed of the Robot
    */
    double getSpeed();

    /**
    @brief Get the current plank of the Robot
    */
    Plank getCurrentPlank();

    /**
    @brief Get the visibility of the Robot
    @return If it is visible
    */
    bool getVisible();

    void setVisible(bool set_value);

    void setIndex(int index);

    bool getSideCamera();

    void setSideCamera(bool value);
    
    /**
    @brief check if robot is approaching a point
    @param a point_t point
    */
    bool approaching(point_t point);
    
    bool isInArena();
    
    /**
    @brief Set the visibility of the Robot
    bool setVisibility();
    @param visibility Boolean value representing if the robot is visible or not
    */
    void setVisibility(bool visible);

    
    void setOrientation(float angle);
    /**
    @brief Set the position and orientation of the Robot
    @param position Point struct of the new position
    @param q The new orientational angle in radians
    */
    void setPositionOrientation(point_t position, double q);

    /**
    @brief Add time to the robots time 
    @param time Time to add to robots elapsed time since the last turn started
    */
    void addToTimer(double time);

    /**
    The kalman filter assumes that this is updated frequently (at least once a second, but preferably more often)
    */
    void update(int index, point_t position,double q, double elapsed_time, bool visible);
    void update(Robot robot);
    /**
    @brief Checks if the Robot is moving. Is often equivalent to the robot turning.
    */
    bool isMoving();

    /**
    @brief returns whether or not the robot has interacted with the drone
    @return if the robot has interacted with the drone or not
    */
    bool getWasInteractedWith();

    /**
    @brief sets the wasInteractedWith variable to true. Only sets to true, since it should never be reset to false after it is first set to true.
    */
    void setInteractedWithTrue();


    /**
     * @brief      Gets the robot position at a time_stamp given no random movement, drone interactions or collisions.
     * @param[in]  elapsed_time  The elapsed time
     * @return     The robot position at time.
     */
    Robot getRobotPositionAtTime(double elapsed_time);


    float getOrientationFromPositionHistory();

    bool recentlySeen(float time);

    friend std::ostream& operator<<(std::ostream &strm, const Robot &robot);
};
