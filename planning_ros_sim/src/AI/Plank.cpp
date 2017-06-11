#include "Plank.h"

Plank::Plank(){
	this->endpoint_1 = point_Zero;
	this->endpoint_2 = point_Zero;
    this->angle = 0;
    this->length = 0;
    this->reward = -200000;
}

Plank::Plank(point_t position, float angle, float time_After_Turn, int num_Iterations){
	Plank();
	this->updatePlank(position, angle, time_After_Turn, num_Iterations);
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

    if(i==2){
        return this->endpoint_2;
    }
    return this->endpoint_1;
}
bool Plank::willExitGreen(){

    if(this->endpoint_1.y > 20){
        std::cout << "Plank will exit green using temporary solution" << std::endl;
        return true;
    }
	return false; //Fix this
}
bool Plank::willExitRed(){
    std::cout << "Plank will exit red using temporary solution" << std::endl;
	if(this->endpoint_1.y < 0 || this->endpoint_1.x > 20 || this->endpoint_1.x  < 0){
        return true;
    }
    if(this->endpoint_2.y < 0 || this->endpoint_2.x > 20 || this->endpoint_2.x  < 0){
        return true;
    }
    return false; //Fix this
}

float Plank::calculateReward(int n){
    float step_tot = this->length/n;
    float step_x = step_tot*cos(this->angle);
    float step_y = step_tot*sin(this->angle);

    float area = 0.0;  // signed area
    float x = 0.0;
    float y = 0.0;
 
    for (int i = 0; i < n; i++) {
        x = this->endpoint_1.x + (i + 0.5) * step_x;
        y = this->endpoint_1.y + (i + 0.5) * step_y;
        area += world->getGridValue(x, y) * step_tot; // sum up each small rectangle
    }
    return area;
}
void Plank::updatePlank(point_t position, float angle, float time_After_Turn, int num_Iterations){
	

    this->angle = angle;
	this->endpoint_1.x = (20- time_After_Turn)*ROBOT_SPEED*cos(angle) + position.x;
    this->endpoint_1.y = (20- time_After_Turn)*ROBOT_SPEED*sin(angle) + position.y;
    this->endpoint_2.x = (time_After_Turn+2.5)*ROBOT_SPEED*cos(angle) + position.x; // Adding 2.5 because of turn time
    this->endpoint_2.y = (time_After_Turn+2.5)*ROBOT_SPEED*sin(angle) + position.y;
    
    float dx = this->endpoint_2.x - this->endpoint_1.x;
    float dy = this->endpoint_2.y - this->endpoint_1.y;
    
    this->length = sqrt(dx*dx + dy*dy);

    this->reward = calculateReward(num_Iterations);
}

bool Plank::pointIsOutsideOfPlank(point_t point){
	if ((point.x > this->endpoint_1.x && point.x > this->endpoint_2.x) || 
		(point.x < this->endpoint_1.x && point.x < this->endpoint_2.x) ||
	    (point.y > this->endpoint_1.y && point.y > this->endpoint_2.y) || 
	    (point.y < this->endpoint_1.y && point.y < this->endpoint_2.y)) {
	    return true;
	} else {
	    return false;
	}
}

std::ostream& operator<<(std::ostream &strm, const Plank &plank) {

    float angle = fmod(plank.angle,MATH_PI);

    strm << std::endl << "--- Plank ---" << std::endl
    << "Endpoint 1: "   << plank.endpoint_1 << std::endl
    << "Endpoint 2: "   << plank.endpoint_2 << std::endl
    << "Reward: "       << plank.reward     << std::endl
    << "Angle: "        << angle            << std::endl
    << "Length: "       << plank.length     << std::endl;
    return strm;
};












