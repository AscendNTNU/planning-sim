#include "Plank.h"

#include <cmath>

Plank::Plank(){
	this->end_point.point = point_zero;
	this->start_point.point = point_zero;
    for(int i = 0; i < sizeof(this->plank_points)/sizeof(this->plank_points[0]); i++) {
        this->plank_points[i].point = point_zero;
        this->plank_points[i].time_till_first_arrival = 0;
        this->plank_points[i].is_ahead = true;
        this->plank_points[i].time_since_start_turn = 0;
    }
    this->angle = 0;
    this->length = 0;
    this->reward = -200000;
}

Plank::Plank(point_t position, double angle, double time_after_turn_start, double ROBOT_TURN_TIME){
	Plank();
	this->updatePlank(position, angle, time_after_turn_start, ROBOT_TURN_TIME);
}

double Plank::getReward(){
	return this->reward;
}
double Plank::getAngle(){
	return this->angle;
}
double Plank::getLength(){
	return this->length;
}

plank_point_t Plank::getPoint(int i){
    if(i==0){
        return this->start_point;
    }
    else if(i==11){
        return this->end_point;
    }
    return this->plank_points[i-1];
}

double Plank::getNumPlankPoints() {
    return sizeof(this->plank_points)/sizeof(this->plank_points[0]) + 2;
}

bool Plank::willExitGreen(){
    double out_limit = 0.3;
    if(this->end_point.point.y > 20 + out_limit || this->start_point.point.y > 20 + out_limit){
        return true;
    }
	return false; //Fix this
}

bool Plank::willExitRed(){
	if(this->end_point.point.y < 0 || this->end_point.point.x > 20 || this->end_point.point.x  < 0){
        return true;
    }
    if(this->start_point.point.y < 0 || this->start_point.point.x > 20 || this->start_point.point.x  < 0){
        return true;
    }
    return false; //Fix this
}

void Plank::calculateAllPlankPoints(point_t robot_position){
    double step_length = this->length/10;
    double step_x = step_length*cosf(this->angle);
    double step_y = step_length*sinf(this->angle);

    this->end_point.time_till_first_arrival = getDistanceBetweenPoints(this->end_point.point, robot_position) / ROBOT_SPEED;
    this->start_point.time_till_first_arrival = (getDistanceBetweenPoints(this->start_point.point, robot_position) +  
                                                             2*getDistanceBetweenPoints(robot_position, this->end_point.point))/ ROBOT_SPEED + 2;
    for (int i = 0; i < 10; i++) {
        // std::cout << i << std::endl;

        this->plank_points[i].point.x = this->start_point.point.x + (i + 0.5) * step_x;
        this->plank_points[i].point.y = this->start_point.point.y + (i + 0.5) * step_y;

        this->plank_points[i].is_ahead = isPointAheadOfRobot(this->plank_points[i].point, robot_position);
        if(this->plank_points[i].is_ahead) {
            this->plank_points[i].time_till_first_arrival = getDistanceBetweenPoints(this->plank_points[i].point, robot_position) / ROBOT_SPEED;
        }
        else {
            this->plank_points[i].time_till_first_arrival = (getDistanceBetweenPoints(this->plank_points[i].point, robot_position) +  
                                                             2*getDistanceBetweenPoints(robot_position, this->end_point.point))/ ROBOT_SPEED + 2;
        }

        this->plank_points[i].time_since_start_turn = 2.0 + (i+1)*(18.0/(10.0+1));
    }
}

bool Plank::isPointAheadOfRobot(point_t point_in_question, point_t robot_position) {
    double x1 = std::min(this->start_point.point.x, point_in_question.x);
    double x2 = std::max(this->start_point.point.x, point_in_question.x);
    double y1 = std::min(this->start_point.point.y, point_in_question.y);
    double y2 = std::max(this->start_point.point.y, point_in_question.y);

    return (x1 < robot_position.x && robot_position.x < x2 &&
            y1 < robot_position.y && robot_position.y < y2);
}

double Plank::calculateReward(){
    double step_length = this->length/10;
    double reward = 0.0;
    double value = 0.0;

    for (int i = 0; i < 10; i++) {
        value = world.getGridValue(this->plank_points[i].point.x, this->plank_points[i].point.y);
        reward += value * step_length;
    }
    return reward;
}

void Plank::updatePlank(point_t position, double angle, double time_after_turn_start, double ROBOT_TURN_TIME){
    this->end_point.is_ahead   = true;
    this->start_point.is_ahead = false;
    this->end_point.time_since_start_turn = 20;
    this->start_point.time_since_start_turn = 0;

    this->angle = angle;
    if(time_after_turn_start < ROBOT_TURN_TIME){
        this->end_point.point.x = position.x;
        this->end_point.point.y = position.y;
    }
    else {
        this->end_point.point.x = position.x + (20 - time_after_turn_start)*ROBOT_SPEED*cosf(angle);
        this->end_point.point.y = position.y + (20 - time_after_turn_start)*ROBOT_SPEED*sinf(angle);
    }
    this->start_point.point.x = this->end_point.point.x - (20-ROBOT_TURN_TIME)*ROBOT_SPEED*cosf(this->angle); // Subtracting 2.5 because of turn time (no translation)
    this->start_point.point.y = this->end_point.point.y - (20-ROBOT_TURN_TIME)*ROBOT_SPEED*sinf(this->angle);
    

    double dx = this->start_point.point.x - this->end_point.point.x;
    double dy = this->start_point.point.y - this->end_point.point.y;
    
    this->length = sqrt(dx*dx + dy*dy);
    this->calculateAllPlankPoints(position);
    this->reward = calculateReward();
}

bool Plank::pointIsOutsideOfPlank(point_t point){

    double tol = 0.1;
	if ((point.x > (this->end_point.point.x + tol) && point.x > (this->start_point.point.x + tol)) || 
		(point.x < (this->end_point.point.x - tol) && point.x < (this->start_point.point.x - tol)) ||
	    (point.y > (this->end_point.point.y + tol) && point.y > (this->start_point.point.y + tol)) || 
	    (point.y < (this->end_point.point.y - tol) && point.y < (this->start_point.point.y - tol))) {
	    return true;
	} else {
	    return false;
	}
}

point_t Plank::getRobotPositionAtTime(double elapsed_time){
    double no_of_turns = elapsed_time/20;
    double driving_time = elapsed_time - 20*floor(no_of_turns);
    point_t point;
    if(driving_time > 0){
        point.x = driving_time*ROBOT_SPEED*cosf(this->angle)+this->start_point.point.x;
        point.y = driving_time*ROBOT_SPEED*sinf(this->angle)+this->start_point.point.y;
    }
    else{
        point.x = this->start_point.point.x;
        point.y = this->start_point.point.y;
    }
    return point;
}

std::ostream& operator<<(std::ostream &strm, const Plank &plank) {

    strm << std::endl << "--- Plank ---" << std::endl
                      << "Endpoint 1: "   << plank.end_point.point   << std::endl
                      << "Endpoint 2: "   << plank.start_point.point << std::endl
                      << "Reward: "       << plank.reward            << std::endl
                      << "Angle: "        << plank.angle             << std::endl
                      << "Length: "       << plank.length            << std::endl;
    return strm;
};
