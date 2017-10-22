#include "Plank.h"

Plank::Plank(){
    for (int i = 0; i < sizeof(this->plank_points); i++)
        this->plank_points[i] = new point_t;
	this->end_point = point_Zero;
	this->start_point = point_Zero;
    this->angle = 0;
    this->length = 0;
    this->reward = -200000;
}

Plank::Plank(point_t position, float angle, float time_After_Turn_Start, int num_Iterations){
	Plank();
	this->updatePlank(position, angle, time_After_Turn_Start, num_Iterations);
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

point_t Plank::getPoint(int i){
    if(i==0){
        return this->start_point;
    }
    else if(i==11){
        return this->end_point;
    }
    return this->plank_points[i];
}

bool Plank::willExitGreen(){
    if(this->end_point.y > 20 || this->start_point.y > 20 ){
        return true;
    }
	return false; //Fix this
}

bool Plank::willExitRed(){
	if(this->end_point.y < 0 || this->end_point.x > 20 || this->end_point.x  < 0){
        return true;
    }
    if(this->start_point.y < 0 || this->start_point.x > 20 || this->start_point.x  < 0){
        return true;
    }
    return false; //Fix this
}

void Plank::calculateAllPlankPoints(){
    float step_length = this->length/10;
    float step_x = step_length*cos(this->angle);
    float step_y = step_length*sin(this->angle);

    for (int i = 0; i < 10; i++) {
        this->plank_points[i].x = this->end_point.x + (i + 0.5) * step_x;
        this->plank_points[i].y = this->end_point.x + (i + 0.5) * step_x;
    }
}

float Plank::calculateReward(int n){
    float step_length = this->length/10;
    float reward = 0.0;
    float value = 0.0;

    for (int i = 0; i < n; i++) {
        value = world.getGridValue(this->plank_points[i].x, this->plank_points[i].y);
        reward += value * step_length;
    }
    return reward;
}

void Plank::updatePlank(point_t position, float angle, float time_After_Turn_Start, int num_Iterations){

    this->angle = angle;
    if(time_After_Turn_Start < 2){
        this->end_point.x = position.x;
        this->end_point.y = position.y;
    }
    else {
        this->end_point.x = position.x + (20 - time_After_Turn_Start)*ROBOT_SPEED*cos(angle);
        this->end_point.y = position.y + (20 - time_After_Turn_Start)*ROBOT_SPEED*sin(angle);
    }
    this->start_point.x = this->end_point.x - (20-2)*ROBOT_SPEED*cos(this->angle); // Subtracting 2.5 because of turn time (no translation)
    this->start_point.y = this->end_point.y - (20-2)*ROBOT_SPEED*sin(this->angle);
    

    float dx = this->start_point.x - this->end_point.x;
    float dy = this->start_point.y - this->end_point.y;
    
    this->length = sqrt(dx*dx + dy*dy);

    this->reward = calculateReward(num_Iterations);
}

bool Plank::pointIsOutsideOfPlank(point_t point){

    float tol = 0.001;
	if ((point.x > (this->end_point.x + tol) && point.x > (this->start_point.x + tol)) || 
		(point.x < (this->end_point.x - tol) && point.x < (this->start_point.x - tol)) ||
	    (point.y > (this->end_point.y + tol) && point.y > (this->start_point.y + tol)) || 
	    (point.y < (this->end_point.y - tol) && point.y < (this->start_point.y - tol))) {
	    return true;
	} else {
	    return false;
	}
}

std::ostream& operator<<(std::ostream &strm, const Plank &plank) {

    strm << std::endl << "--- Plank ---" << std::endl
                      << "Endpoint 1: "   << plank.end_point << std::endl
                      << "Endpoint 2: "   << plank.start_point << std::endl
                      << "Reward: "       << plank.reward     << std::endl
                      << "Angle: "        << plank.angle      << std::endl
                      << "Length: "       << plank.length     << std::endl;
    return strm;
};
