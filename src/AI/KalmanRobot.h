/**
@class Robot
@brief Robot class

This class handles all Robot functions. This includes getters and setters and checks.
*/

#pragma once

#include <cmath>
#include "Robot.h"
#include <queue>
#include <opencv2/opencv.hpp>

class KalmanRobot : public Robot{
private:
    
    cv::Mat F;//(3,3,DataType<double>::type);
    cv::Mat H;
    cv::Mat P_k_k;
    cv::Mat P_k_km1;
    cv::Mat P_km1_km1;
    
    cv::Mat x_hat_k_k;
    cv::Mat x_hat_km1_km1;
    cv::Mat x_hat_km2_km2;
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

    bool outlier_observed;

    
public:
    KalmanRobot();
    KalmanRobot(int index);
    cv::Mat x_hat_k;
 
    /**
    The kalman filter assumes that this is updated frequently (at least once a second, but preferably more often)
    */
    void update(int index, point_t position,double q, double elapsed_time, bool visible);
    void update(Robot robot);

    void kalmanStep(point_t new_Position, double new_Orientation, double elapsed_time, bool visible);
    void kalmanPredict(point_t new_Position, double new_Orientation, double elapsed_time, bool visible);
    void kalmanMeasurementUpdate(point_t new_Position, double new_Orientation, double elapsed_time, bool visible);

    void kalmanStepNoObservation(double elapsed_time);

    void setPositionToKalmanPosition();

    friend std::ostream& operator<<(std::ostream &strm, const Robot &robot);
};

float angleDiff(float a, float b);