#include "Robot.h"
#include <cmath>

// Redundant as you can call Robot(-1)
Robot::Robot():Robot(-1) {
}

Robot::Robot(int index) {
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
    
    this->x_hat_k   = (cv::Mat_<double>(6,1) << 10+cos(th_0),
                                                0, 
                                                10+sin(th_0), 
                                                0,
                                                th_0,
                                                0); //TODO: Update xdot, ydot

    this->x_hat_km1 = (cv::Mat_<double>(6,1) << 10+.9*cos(th_0),
                                                0, 
                                                10+.9*sin(th_0), 
                                                0,
                                                th_0,
                                                0); //TODO: Update xdot, ydot
    
    //Measurement covariance parms
    this->xMeasCovar          = 1; //TODO: Tune!
    this->yMeasCovar          = 1; //TODO: Tune!
    this->thMeasCovar_downCam = 1;  //TODO: Tune!
    this->thMeasCovar_sideCam = 1; //TODO: Tune!
    this->R_k = (cv::Mat_<double>(3,3) << this->xMeasCovar, 0, 0, 
                                           0, this->yMeasCovar, 0, 
                                           0, 0, this->thMeasCovar_sideCam);
    
    //Model covariance parms
    double xModelCovar  = 10; //TODO: Tune!
    double yModelCovar  = 10; //TODO: Tune!
    double thModelCovar = 10; //TODO: Tune!
    double xDotModelCovar  = 10; //TODO: Tune!
    double yDotModelCovar  = 10; //TODO: Tune!
    double thDotModelCovar = 10; //TODO: Tune!
    this->Q_k = (cv::Mat_<double>(6,6) << xModelCovar, 0, 0, 0, 0, 0, 
                                          0, xDotModelCovar, 0, 0, 0, 0, 
                                          0, 0, yModelCovar, 0, 0, 0,  
                                          0, 0, 0, yDotModelCovar, 0, 0, 
                                          0, 0, 0, 0, thModelCovar, 0, 
                                          0, 0, 0, 0, 0, thDotModelCovar);
                                           

    this->t_k   = 0;
    this->t_km1 = 0;
    this->dt    = 0;
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

// Static function
bool Robot::robotsAtTurnTime(float elapsed_time) {
    float time_drift = 3.0;//(elapsed_time * 3.0)/600.0;
    float rest = fmod(elapsed_time, 20); 
    if (rest < ROBOT_TURN_TIME + time_drift) {
        return true;
    }
    return false;
}

float Robot::getTimeLastSeen(){
    return this->time_last_seen;
}

int Robot::getIndex() {
    return this->index;
}
point_t Robot::getPosition() {
    return this->position;
}
float Robot::getOrientation() {
    return this->orientation;
}
float Robot::getTimeAfterTurn() {
    return this->time_after_turn_start;
}
float Robot::getSpeed() {
    return this->speed;
}
Plank Robot::getCurrentPlank() {
    return this->plank;
}
bool Robot::getVisible() {
    return this->visible;
}

bool Robot::getSideCamera(){
    return this->side_camera;
}

void Robot::setSideCamera(bool value){
    this->side_camera = value;
}

bool Robot::approaching(point_t point) {
    float old_dist = pow(pow(point.x - this->old_Position.x,2) + pow(point.y - this->old_Position.y,2), 0.5);
    float new_dist = pow(pow(point.x - this->position.x,2) + pow(point.y - this->position.y,2), 0.5);

    return (new_dist < old_dist);
}

bool Robot::getWasInteractedWith() {
    return this->wasInteractedWith;
}

void Robot::setIndex(int index){
    this->index = index;
}

void Robot::setInteractedWithTrue() {
    this->wasInteractedWith = true;
}

void Robot::setVisible(bool set_value){
    this->visible = set_value;
}

bool Robot::isInArena(){
    float out_limit = 0.3;
    if(this->position.x < 0 - out_limit  || this->position.x > 20 + out_limit){
        return false;
    }
    else if(this->position.y > 20 + out_limit || this->position.y < 0 - out_limit){
        return false;
    }
    return true;
}

bool Robot::isMoving() {
    double dist_threshold = this->speed * this->time_between_updates - 0.1; // distance normally driven in 1sec
    double dist = pow(pow(this->position.x - this->old_Position.x,2) + pow(this->position.y - this->old_Position.y,2), 0.5);
    
    //std::cout << "dist_threshold: " << dist_threshold << std::endl;
    //std::cout << "dist: " << dist << std::endl;

    if (dist < dist_threshold) {
        //std::cout << "ROBOT " << index << " TURNING" << std::endl;
        return false;
    }
    else {
        return true;
    }
}

void Robot::update(int index, point_t new_Position, float new_Orientation, float elapsed_time, bool visible) {
    float estimated_orientation = 0;
    int planning_ros_rate = 20;

    this->pos_queue.push(new_Position); // push_back
    this->orientation_queue.push(fmod(new_Orientation, 2*MATH_PI));

    if (pos_queue.size() >= planning_ros_rate * time_between_updates) {
        this->old_Position = pos_queue.front();
        this->pos_queue.pop(); // pop_front

        this->old_Orientation = orientation_queue.front();
        this->orientation_queue.pop(); // pop_front
    }
    this->position = pos_queue.back();
    this->orientation = orientation_queue.back();


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

void Robot::update(Robot robot){
    this->old_Position = this->position;
    this->old_Orientation = this->orientation;
    this->position = robot.getPosition();
    this->side_camera = robot.getSideCamera();

    if(this->side_camera){
        this->orientation = atan2(this->old_Position.y-this->position.y,this->old_Position.x-this->position.x);
    }
    else{
        this->orientation = fmod(robot.getOrientation(), 2*MATH_PI);
    }


    this->time_after_turn_start = robot.getTimeAfterTurn();
    this->time_last_seen = robot.getTimeLastSeen();
    this->visible =  robot.getVisible();

    kalmanStep(robot.getPosition(), fmod(robot.getOrientation(), 2*MATH_PI), robot.getTimeLastSeen(), robot.getVisible());
}

Robot Robot::getRobotPositionAtTime(float elapsed_time){
    point_t point = this->plank.getRobotPositionAtTime(elapsed_time);
    Robot robot;
    robot.update(this->index, point, this->orientation, elapsed_time, true);
    return robot;
}

void Robot::setPositionOrientation(point_t position, float q) {
    this->position = position;
    this->orientation = q;
}

// When is this used? Double check that it is used the way we want.
// Adding time to time_after_turn_start makes the variable name be
// misleading, except when correcting for drift.
void Robot::addToTimer(float time) {
    this->time_after_turn_start += time;
}

void Robot::kalmanStep(point_t new_Position, float new_Orientation, float elapsed_time, bool visible) {
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
    // this->x_hat_km1 = this->x_hat_k;
}

void Robot::kalmanStepNoObservation(float elapsed_time) {
    //get updated system matrices
    //do model prediction
    kalmanStep(point_zero, 0,  elapsed_time, false);
}

void Robot::kalmanPredict(point_t new_Position, float new_Orientation, float elapsed_time, bool visible) {
    double time_After_Turn_Start = fmod(elapsed_time, 20);
    double th_km1 = this->x_hat_k.at<double>(4,0);
    
    //spinning
    if(time_After_Turn_Start < 2.5 && elapsed_time>2.5) { 
        if(firstTimeTurning) {
            this->x_hat_k.at<double>(1,0) = 0;
            this->x_hat_k.at<double>(3,0) = 0;
            this->x_hat_k.at<double>(5,0) = MATH_PI/2.5;
            this->firstTimeTurning=false;
            this->firstTimeDriving=true;
        }
        // std::cout << "x_hat_km1_km1 = " << std::endl << this->x_hat_k << std::endl; 
    }
    //driving straight
    else {
        double xdot = this->speed*cos(th_km1);
        double ydot = this->speed*sin(th_km1);

        this->x_hat_k.at<double>(1,0) = xdot;
        this->x_hat_k.at<double>(3,0) = ydot;

        if(firstTimeDriving) {
            this->x_hat_k.at<double>(5,0) = 0;
            this->firstTimeTurning=true;
            this->firstTimeDriving=false;
        }
        
        //print x_hat
        // std::cout << "x_hat_km1_km1 = " << std::endl << this->x_hat_k << std::endl; 
    }

    this->F = (cv::Mat_<double>(6,6) << 1, this->dt, 0, 0, 0, 0,
                                        0, 1,        0, 0, 0, 0,
                                        0, 0, 1,     this->dt, 0, 0,
                                        0, 0, 0,     1, 0, 0,
                                        0, 0, 0,     0, 1, this->dt,
                                        0, 0, 0,     0, 0, 1);
    // std::cout << "F = " << std::endl << this->F << std::endl; 
    
    this->x_hat_km1 = this->F*this->x_hat_k;

    this->P_k_km1 = this->F*this->P_km1_km1*this->F.t() + this->Q_k;

}

void Robot::setPositionToKalmanPosition() {
    this->position.x = this->x_hat_k.at<double>(0,0);
    this->position.y = this->x_hat_k.at<double>(2,0);
    
}

void Robot::kalmanMeasurementUpdate(point_t new_Position, float new_Orientation, float elapsed_time, bool visible) {
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
    double xk   = this->x_hat_km1.at<double>(0,0);
    double xkm1 = this->x_hat_k.at<double>(0,0);
    double yk   = this->x_hat_km1.at<double>(2,0);
    double ykm1 = this->x_hat_k.at<double>(2,0);

    if(visible) {
        double th_meas;
        if(false) { //sideCamera - TODO: update to actually change between the cameras
            if(fabs(yk-ykm1) < 0.001 && fabs(xk-xkm1) < 0.001) {
                th_meas = this->x_hat_k.at<double>(4,0);
            }
            else {
                th_meas   = atan2(yk-ykm1,xk-xkm1); //TODO: this should probably be the measurements, right?
            }
            this->R_k = (cv::Mat_<double>(3,3) << this->xMeasCovar, 0, 0, 
                                                   0, this->yMeasCovar, 0, 
                                                   0, 0, this->thMeasCovar_sideCam);
            // std::cout << "R_k = " << std::endl << this->R_k << std::endl;
        }
        else { //downCamera
            th_meas = new_Orientation;
            this->R_k = (cv::Mat_<double>(3,3) << this->xMeasCovar, 0, 0, 
                                                   0, this->yMeasCovar, 0, 
                                                   0, 0, this->thMeasCovar_downCam);
            // std::cout << "R_k = " << std::endl << this->R_k << std::endl;
        }
        z_k = (cv::Mat_<double>(3,1) << double(new_Position.x), double(new_Position.y), th_meas);   
        
        // std::cout << "Measurements: " << std::endl;
        // std::cout << "Z_k = " << std::endl << z_k << std::endl; 

        y_k = z_k-this->H*this->x_hat_km1; //calculate residual



        S_k = this->H*this->P_k_km1*this->H.t() + this->R_k; //residual covariance
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

    // std::cout << "x_hat_km1 = " << std::endl << this->x_hat_km1 << std::endl; 
        
    this->x_hat_k = this->x_hat_km1 + K_k*y_k;
    this->P_k_k = (I - K_k*this->H)*this->P_k_km1*(I - K_k*this->H).t() + K_k*this->R_k*K_k.t();

    // std::cout << "x_hat_k = " << std::endl << this->x_hat_k << std::endl; 
        

}

std::ostream& operator<<(std::ostream &strm, const Robot &robot) {
    strm << "--- Robot ---" << std::endl
    << "Index: "            << robot.index                 << std::endl
    << "Position: "         << robot.position              << std::endl
    << "Old pos.: "         << robot.old_Position          << std::endl
    << "Orientation: "      << robot.orientation           << std::endl
    << "Old orient.: "      << robot.old_Orientation       << std::endl
    << "Time after: "       << robot.time_after_turn_start << std::endl
    << "Speed: "            << robot.speed                 << std::endl
    << "Interacted With: "  << robot.wasInteractedWith     << std::endl
    << "Current plank: "    << robot.plank
    << "-------------"                                      << std::endl;
    return strm;
};
