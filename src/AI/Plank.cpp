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
    this->reward = Config::INITIAL_PLANK_REWARD;
}

Plank::Plank(point_t position, float angle, float time_after_turn_start){
	Plank();
	this->updatePlank(position, angle, time_after_turn_start);
}

float Plank::getReward(){
	return this->reward;
}
float Plank::getAngle(){
	return this->angle;
}
float Plank::getLength(){
	return this->length;
}

plank_point_t Plank::getPoint(int i){
    if(i==0){
        return this->start_point;
    }
    else if(i==Config::NUM_PLANK_POINTS+1){
        return this->end_point;
    }
    return this->plank_points[i-1];
}

float Plank::getTotalNumPlankPoints() {
    return sizeof(this->plank_points)/sizeof(this->plank_points[0]) + 2;
}

bool Plank::willExitGreen(){
    if(this->end_point.point.y > Config::GRID_BOUNDS_Y + Config::ACTION_EDGE_BUFFER 
        || this->start_point.point.y > Config::GRID_BOUNDS_Y + Config::ACTION_EDGE_BUFFER){
        return true;
    }
	return false; //Fix this
}

bool Plank::willExitRed(){
	if(this->end_point.point.y < 0 
        || this->end_point.point.x > Config::GRID_BOUNDS_X 
        || this->end_point.point.x  < 0){
        return true;
    }
    if(this->start_point.point.y < 0 
        || this->start_point.point.x > Config::GRID_BOUNDS_X 
        || this->start_point.point.x  < 0){
        return true;
    }
    return false; //Fix this
}

void Plank::calculateAllPlankPoints(point_t robot_position){
    float step_length = this->length/Config::NUM_PLANK_POINTS;
    float step_x = step_length*cosf(this->angle);
    float step_y = step_length*sinf(this->angle);

    this->end_point.time_till_first_arrival = getDistanceBetweenPoints(this->end_point.point, robot_position) 
                                                / Config::ROBOT_SPEED;
    this->start_point.time_till_first_arrival = (getDistanceBetweenPoints(this->start_point.point, robot_position)
                                                    + 2*getDistanceBetweenPoints(robot_position, this->end_point.point))
                                                    / Config::ROBOT_SPEED + Config::ROBOT_TURN_TIME;
    for (int i = 0; i < Config::NUM_PLANK_POINTS; i++) {
        // std::cout << i << std::endl;

        this->plank_points[i].point.x = this->start_point.point.x + (i + 0.5) * step_x;
        this->plank_points[i].point.y = this->start_point.point.y + (i + 0.5) * step_y;

        this->plank_points[i].is_ahead = isPointAheadOfRobot(this->plank_points[i].point, robot_position);
        if(this->plank_points[i].is_ahead) {
            this->plank_points[i].time_till_first_arrival = getDistanceBetweenPoints(this->plank_points[i].point, robot_position)
                                                                / Config::ROBOT_SPEED;
        }
        else {
            this->plank_points[i].time_till_first_arrival = (getDistanceBetweenPoints(this->plank_points[i].point, robot_position) 
                                                                + 2*getDistanceBetweenPoints(robot_position, this->end_point.point))
                                                                / Config::ROBOT_SPEED + Config::ROBOT_TURN_TIME;
        }

        // 2.5 + (20-2.5)*(i+1/(10.0+2.0));
        this->plank_points[i].time_since_start_turn = Config::ROBOT_TURN_TIME 
                                                    + (Config::ROBOT_REVERSE_INTERVAL 
                                                        - Config::ROBOT_TURN_TIME)
                                                    * (i+1/(Config::NUM_PLANK_POINTS+2.0)); // 2 = num endpoints
    }
}

bool Plank::isPointAheadOfRobot(point_t point_in_question, point_t robot_position) {
    float x1 = std::min(this->start_point.point.x, point_in_question.x);
    float x2 = std::max(this->start_point.point.x, point_in_question.x);
    float y1 = std::min(this->start_point.point.y, point_in_question.y);
    float y2 = std::max(this->start_point.point.y, point_in_question.y);

    return (x1 < robot_position.x && robot_position.x < x2 &&
            y1 < robot_position.y && robot_position.y < y2);
}

float Plank::calculateReward(){
    float step_length = this->length/Config::NUM_PLANK_POINTS;
    float reward = 0.0;
    float value = 0.0;

    for (int i = 0; i < Config::NUM_PLANK_POINTS; i++) {
        value = world.getGridValue(this->plank_points[i].point);
        reward += value * step_length;
    }
    reward += world.getGridValue(this->start_point.point);
    reward += world.getGridValue(this->end_point.point);
    return reward;
}

void Plank::updatePlank(point_t position, float angle, float time_after_turn_start){
    this->end_point.is_ahead   = true;
    this->start_point.is_ahead = false;
    this->end_point.time_since_start_turn   = Config::ROBOT_REVERSE_INTERVAL;
    this->start_point.time_since_start_turn = 0;

    this->angle = angle;

    if(time_after_turn_start < Config::ROBOT_TURN_TIME){
        this->start_point.point.x = position.x;
        this->start_point.point.y = position.y;
    }
    else {
        this->start_point.point.x = position.x 
            - (Config::ROBOT_REVERSE_INTERVAL - time_after_turn_start)
            * Config::ROBOT_SPEED*cosf(angle);
        this->start_point.point.y = position.y 
            - (Config::ROBOT_REVERSE_INTERVAL - time_after_turn_start)
            * Config::ROBOT_SPEED*sinf(angle);
    }
    this->end_point.point.x = this->end_point.point.x 
        + (Config::ROBOT_REVERSE_INTERVAL-Config::ROBOT_TURN_TIME)
        * Config::ROBOT_SPEED*cosf(this->angle);
    this->end_point.point.y = this->end_point.point.y 
        + (Config::ROBOT_REVERSE_INTERVAL-Config::ROBOT_TURN_TIME)
        * Config::ROBOT_SPEED*sinf(this->angle);
    

    float dx = this->start_point.point.x - this->end_point.point.x;
    float dy = this->start_point.point.y - this->end_point.point.y;
    
    this->length = sqrt(dx*dx + dy*dy);
    this->calculateAllPlankPoints(position);
    this->reward = calculateReward();
}

bool Plank::pointIsOutsideOfPlank(point_t point){

    float tol = Config::POINT_OUTSIDE_OF_PLANK_TOLERANCE;
	if ((point.x > (this->end_point.point.x + tol) && point.x > (this->start_point.point.x + tol)) || 
		(point.x < (this->end_point.point.x - tol) && point.x < (this->start_point.point.x - tol)) ||
	    (point.y > (this->end_point.point.y + tol) && point.y > (this->start_point.point.y + tol)) || 
	    (point.y < (this->end_point.point.y - tol) && point.y < (this->start_point.point.y - tol))) {
	    return true;
	} else {
	    return false;
	}
}

point_t Plank::getRobotPositionAtTime(float elapsed_time){
    float num_turns = elapsed_time/Config::ROBOT_REVERSE_INTERVAL;
    float driving_time = elapsed_time - Config::ROBOT_REVERSE_INTERVAL*floor(num_turns);
    point_t point;
    if(driving_time > 0){
        point.x = driving_time*Config::ROBOT_SPEED*cosf(this->angle)+this->start_point.point.x;
        point.y = driving_time*Config::ROBOT_SPEED*sinf(this->angle)+this->start_point.point.y;
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
