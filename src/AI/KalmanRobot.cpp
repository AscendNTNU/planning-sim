#include "KalmanRobot.h"
#include <cmath>

// Redundant as you can call Robot(-1)
KalmanRobot::KalmanRobot():KalmanRobot(-1) {
}

KalmanRobot::KalmanRobot(int index) {
    this->index = index;
    this->position = point_zero;
    this->old_Position = point_zero;
    this->orientation = 0;
    this->speed = 0.33;
    this->plank = Plank();
    this->time_after_turn_start = 0;
    this->wasInteractedWith = false;
    this->visible = false;

    this->side_camera = false;
    //this->prev_pos_update = 0;
    this->time_between_updates = 1;
    
    //Kalman parameters
    this->F = (cv::Mat_<double>(6,6) << 1, 0, 0, 0, 0, 0,
                                        0, 1, 0, 0, 0, 0,
                                        0, 0, 1, 0, 0, 0,
                                        0, 0, 0, 1, 0, 0,
                                        0, 0, 0, 0, 1, 0,
                                        0, 0, 0, 0, 0, 1); //state transition matrix while spinning
    
    this->H = (cv::Mat_<double>(3,6) << 1, 0, 0, 0, 0, 0,
                                        0, 0, 1, 0, 0, 0,
                                        0, 0, 0, 0, 1, 0
                                        ); //observation matrix
    

    double th_0 = this->index*((2*MATH_PI)/10); //initial theta
    
    this->x_hat_k_k = (cv::Mat_<double>(6,1) << 10+cos(th_0),
                                                0, 
                                                10+sin(th_0), 
                                                0,
                                                th_0,
                                                0); 

    this->x_hat_km1_km1 = (cv::Mat_<double>(6,1) << 10+.9*cos(th_0),
                                                0, 
                                                10+.9*sin(th_0), 
                                                0,
                                                th_0,
                                                0); 

    this->x_hat_km2_km2 = (cv::Mat_<double>(6,1) << 10+.9*cos(th_0),
                                                0, 
                                                10+.9*sin(th_0), 
                                                0,
                                                th_0,
                                                0); 

    this->x_hat_k_km1 = (cv::Mat_<double>(6,1) << 10+cos(th_0),
                                                0, 
                                                10+sin(th_0), 
                                                0,
                                                th_0,
                                                0); 
    
    //Measurement covariance parms
    this->xMeasCovar          = 10; //TODO: Tune!
    this->yMeasCovar          = 10; //TODO: Tune!
    this->thMeasCovar_downCam = 10;  //TODO: Tune!
    this->thMeasCovar_sideCam = 10; //TODO: Tune!
    this->R_k = (cv::Mat_<double>(3,3) << this->xMeasCovar, 0, 0, 
                                           0, this->yMeasCovar, 0, 
                                           0, 0, this->thMeasCovar_sideCam);
    
    //Model covariance parms
    double xModelCovar  = 0.1; //TODO: Tune!
    double yModelCovar  = 0.1; //TODO: Tune!
    double thModelCovar = 0.04; //TODO: Tune!
    double xDotModelCovar  = 0.1; //TODO: Tune!
    double yDotModelCovar  = 0.1; //TODO: Tune!
    double thDotModelCovar = 0.1; //TODO: Tune!
    this->Q_k = (cv::Mat_<double>(6,6) << xModelCovar, 0, 0, 0, 0, 0, 
                                          0, xDotModelCovar, 0, 0, 0, 0, 
                                          0, 0, yModelCovar, 0, 0, 0,  
                                          0, 0, 0, yDotModelCovar, 0, 0, 
                                          0, 0, 0, 0, thModelCovar, 0, 
                                          0, 0, 0, 0, 0, thDotModelCovar);
                                           

    this->t_k   = 0;
    this->t_km1 = 1;
    this->dt    = 1;
    this->P_k_k       = (cv::Mat_<double>(6,6) << 0, 0, 0, 0, 0, 0, 
                                                0, 0, 0, 0, 0, 0,
                                                0, 0, 0, 0, 0, 0,
                                                0, 0, 0, 0, 0, 0,
                                                0, 0, 0, 0, 0, 0,
                                                0, 0, 0, 0, 0, 0
                                                );
    this->P_k_km1     = (cv::Mat_<double>(6,6) << 0, 0, 0, 0, 0, 0,
                                                0, 0, 0, 0, 0, 0,
                                                0, 0, 0, 0, 0, 0,
                                                0, 0, 0, 0, 0, 0,
                                                0, 0, 0, 0, 0, 0,
                                                0, 0, 0, 0, 0, 0);
    this->P_km1_km1 = (cv::Mat_<double>(6,6) << 0, 0, 0, 0, 0, 0,
                                                0, 0, 0, 0, 0, 0,
                                                0, 0, 0, 0, 0, 0,
                                                0, 0, 0, 0, 0, 0,
                                                0, 0, 0, 0, 0, 0,
                                                0, 0, 0, 0, 0, 0);
    
    this->firstTimeTurning=true;
    this->firstTimeDriving=true;

}
void KalmanRobot::update(int index, point_t new_Position, double new_Orientation, double elapsed_time, bool visible) {
    double estimated_orientation = 0;
    int planning_ros_rate = 20;

    // this->pos_queue.push(new_Position); // push_back
    // this->orientation_queue.push(fmod(new_Orientation, 2*MATH_PI));

    // if (pos_queue.size() >= planning_ros_rate * this->time_between_updates) {
    //     this->old_Position = pos_queue.front();
    //     this->pos_queue.pop(); // pop_front

    //     this->old_Orientation = orientation_queue.front();
    //     this->orientation_queue.pop(); // pop_front
    // }

    // this->position = pos_queue.back();

    // this->orientation = orientation_queue.back();
    this->old_Position = this->position;
    this->old_Orientation = this->orientation;
    this->position = new_Position;
    // this->side_camera = robot.getSideCamera();
    this->orientation = fmod(new_Orientation, 2*MATH_PI);

    this->index = index;
    this->time_after_turn_start = fmod(elapsed_time, 20);
    this->time_last_seen = elapsed_time;
    this->visible = visible;


    if (this->time_after_turn_start < ROBOT_TURN_TIME) {
        estimated_orientation = fmod(this->orientation - MATH_PI, 2*MATH_PI);
        this->plank.updatePlank(this->position, estimated_orientation, this->time_after_turn_start, ROBOT_TURN_TIME); // Will this make Plank construct a plank which the robot never will follow?
    } else {
        this->plank.updatePlank(this->position, this->orientation, this->time_after_turn_start, ROBOT_TURN_TIME);
    }

    kalmanStep(new_Position, new_Orientation, elapsed_time, visible);
}

void KalmanRobot::update(Robot robot){

    this->old_Position = this->position;
    this->old_Orientation = this->orientation;
    this->position = robot.getPosition();
    this->side_camera = robot.getSideCamera();
    this->orientation = fmod(robot.getOrientation(), 2*MATH_PI);

    this->time_after_turn_start = robot.getTimeAfterTurn();
    this->time_last_seen = robot.getTimeLastSeen();
    this->visible =  robot.getVisible();

    kalmanStep(robot.getPosition(), fmod(robot.getOrientation(), 2*MATH_PI), robot.getTimeLastSeen(), robot.getVisible());
}

void KalmanRobot::kalmanStep(point_t new_Position, double new_Orientation, double elapsed_time, bool visible) {


    // Kalman filter stuff
    this->t_km1 = this->t_k;
    this->t_k = elapsed_time;
    this->dt = this->t_k-this->t_km1;
    
    // std::cout << "Index: " << this->index << std::endl;

    //get updated system matrices
    //do model prediction
    kalmanPredict(new_Position, new_Orientation, elapsed_time, visible);

    //update state estimate
    //update covariance update
    kalmanMeasurementUpdate(new_Position, new_Orientation, elapsed_time, visible);

    //get state estimate and covariance to the state ready for the next update
    this->P_km1_km1 = this->P_k_k;
    this->x_hat_km2_km2 = this->x_hat_km1_km1;
    this->x_hat_km1_km1 = this->x_hat_k_k;

}

void KalmanRobot::kalmanStepNoObservation(double elapsed_time) {
    //get updated system matrices
    //do model prediction
    kalmanStep(point_zero, 0,  elapsed_time, false);
}

void KalmanRobot::kalmanPredict(point_t new_Position, double new_Orientation, double elapsed_time, bool visible) {
    double time_After_Turn_Start = fmod(elapsed_time, 20);
    double th_km1 = this->x_hat_km1_km1.at<double>(4,0);
    
    //spinning
    if(time_After_Turn_Start < 2.5 && elapsed_time>2.5) { 
        if(firstTimeTurning) {
            this->x_hat_km1_km1.at<double>(1,0) = 0;
            this->x_hat_km1_km1.at<double>(3,0) = 0;
            this->x_hat_km1_km1.at<double>(5,0) = MATH_PI/2.5;
            this->firstTimeTurning=false;
            this->firstTimeDriving=true;
        }
        // std::cout << "x_hat_km1_km1 = " << std::endl << this->x_hat_km1_km1 << std::endl; 
    }
    //driving straight
    else {
        double xdot = this->speed*cos(th_km1);
        double ydot = this->speed*sin(th_km1);

        this->x_hat_km1_km1.at<double>(1,0) = xdot;
        this->x_hat_km1_km1.at<double>(3,0) = ydot;
        this->x_hat_km1_km1.at<double>(5,0) = 0;

        if(firstTimeDriving) {
            this->firstTimeTurning=true;
            this->firstTimeDriving=false;
        }
        
        //print x_hat
        // std::cout << "x_hat_km1_km1 = " << std::endl << this->x_hat_km1_km1 << std::endl; 
    }

    this->F = (cv::Mat_<double>(6,6) << 1, this->dt, 0, 0, 0, 0,
                                        0, 1,        0, 0, 0, 0,
                                        0, 0, 1,     this->dt, 0, 0,
                                        0, 0, 0,     1, 0, 0,
                                        0, 0, 0,     0, 1, this->dt,
                                        0, 0, 0,     0, 0, 1);
    // std::cout << "F = " << std::endl << this->F << std::endl; 
    this->x_hat_k_km1 = this->F*this->x_hat_km1_km1;

    this->P_k_km1 = this->F*this->P_km1_km1*this->F.t() + this->Q_k;

}

void KalmanRobot::setPositionToKalmanPosition() {
    this->old_Position.x = this->x_hat_km1_km1.at<double>(0,0);
    this->old_Position.y = this->x_hat_km1_km1.at<double>(2,0);
    this->position.x = this->x_hat_k_k.at<double>(0,0);
    this->position.y = this->x_hat_k_k.at<double>(2,0);
    this->orientation = this->x_hat_k_k.at<double>(4,0);
}

void KalmanRobot::kalmanMeasurementUpdate(point_t new_Position, double new_Orientation, double elapsed_time, bool visible) {
    cv::Mat z_k; //state measurement
    cv::Mat K_k; //Kalman gain
    cv::Mat y_k; //residual (measurement-model prediction)
    cv::Mat S_k; //residual (measurement-model prediction)
    cv::Mat I = (cv::Mat_<double>(6,6) << 1, 0, 0, 0, 0, 0, 
                                          0, 1, 0, 0, 0, 0,
                                          0, 0, 1, 0, 0, 0,
                                          0, 0, 0, 1, 0, 0,
                                          0, 0, 0, 0, 1, 0,
                                          0, 0, 0, 0, 0, 1
                                                    );
    double delta_x = this->x_hat_km1_km1.at<double>(0,0) - this->x_hat_km2_km2.at<double>(0,0);
    double delta_y = this->x_hat_km1_km1.at<double>(2,0) - this->x_hat_km2_km2.at<double>(2,0);


    if(visible) {

        double th_meas;

        float angle_diff = angleDiff(new_Orientation, this->orientation);

        // If we get a single outlier ignore it, otherwise update with the new theta obs
        if(this->side_camera && 
            angle_diff > MATH_PI*1/3 && 
            this->outlier_observed == false){

            this->outlier_observed = true;
            th_meas = this->orientation;
        }
        else{
            this->outlier_observed = false;
            th_meas = new_Orientation;
        }

        this->R_k = (cv::Mat_<double>(3,3) << this->xMeasCovar, 0, 0, 
                                               0, this->yMeasCovar, 0, 
                                               0, 0, this->thMeasCovar_downCam);

        z_k = (cv::Mat_<double>(3,1) << double(new_Position.x), double(new_Position.y), th_meas);   
        
        // std::cout << "Measurements: " << std::endl;
        // std::cout << "Z_k = " << std::endl << z_k << std::endl; 

        y_k = z_k-this->H*this->x_hat_k_km1; //calculate residual



        S_k = this->R_k + this->H*this->P_k_km1*this->H.t(); //residual covariance
        K_k = this->P_k_km1*this->H.t()*S_k.inv();

        // std::cout << "K_k: " << K_k << std::endl;

    }
    else {
        //Set Kalman gain (K_k) and residual (y_k) to 0
        K_k = (cv::Mat_<double>(6,3) << 0, 0, 0, 
                                         0, 0, 0, 
                                         0, 0, 0,
                                         0, 0, 0, 
                                         0, 0, 0, 
                                         0, 0, 0);
        y_k = (cv::Mat_<double>(3,1) << 0, 0, 0);
    }

    // std::cout << "x_hat_k_km1 = " << std::endl << this->x_hat_k_km1 << std::endl; 
        
    this->x_hat_k_k = this->x_hat_k_km1 + K_k*y_k;
    this->P_k_k = (I - K_k*this->H)*this->P_k_km1*(I - K_k*this->H).t() + K_k*this->R_k*K_k.t();

    // std::cout << "x_hat_k_k = " << std::endl << this->x_hat_k_k << std::endl; 
        

}


float angleDiff(float a, float b){
    a = fmod(a, 2*MATH_PI);
    b = fmod(b, 2*MATH_PI);

    float dif = fmod(b - a + 180, 360);

    if (dif < 0){
        dif += 360.0;
    }
    return dif - 180.0;


}
