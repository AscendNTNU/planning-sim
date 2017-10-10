#include "Plank.h"

Plank::Plank(){
	this->endpoint_1 = point_Zero;
	this->endpoint_2 = point_Zero;
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
    if(i==2){
        return this->endpoint_2;
    }
    return this->endpoint_1;
}

bool Plank::willExitGreen(){
    if(this->endpoint_1.y > 20 || this->endpoint_2.y > 20 ){
        return true;
    }
	return false; //Fix this
}

bool Plank::willExitRed(){
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
        area += world.getGridValue(x, y) * step_tot; // sum up each small rectangle
    }
    return area;
}

void Plank::updatePlank(point_t position, float angle, float time_After_Turn_Start, int num_Iterations){

    this->angle = angle;
    if(time_After_Turn_Start < 2){
        this->endpoint_1.x = position.x;
        this->endpoint_1.y = position.y;
    }
    else {
        this->endpoint_1.x = position.x + (20 - time_After_Turn_Start)*ROBOT_SPEED*cos(angle);
        this->endpoint_1.y = position.y + (20 - time_After_Turn_Start)*ROBOT_SPEED*sin(angle);
    }
    this->endpoint_2.x = this->endpoint_1.x - (20-2)*ROBOT_SPEED*cos(this->angle); // Subtracting 2.5 because of turn time (no translation)
    this->endpoint_2.y = this->endpoint_1.y - (20-2)*ROBOT_SPEED*sin(this->angle);
    

    float dx = this->endpoint_2.x - this->endpoint_1.x;
    float dy = this->endpoint_2.y - this->endpoint_1.y;
    
    this->length = sqrt(dx*dx + dy*dy);

    this->reward = calculateReward(num_Iterations);
}

bool Plank::pointIsOutsideOfPlank(point_t point){

    float tol = 0.001;
	if ((point.x > (this->endpoint_1.x + tol) && point.x > (this->endpoint_2.x + tol)) || 
		(point.x < (this->endpoint_1.x - tol) && point.x < (this->endpoint_2.x - tol)) ||
	    (point.y > (this->endpoint_1.y + tol) && point.y > (this->endpoint_2.y + tol)) || 
	    (point.y < (this->endpoint_1.y - tol) && point.y < (this->endpoint_2.y - tol))) {
	    return true;
	} else {
	    return false;
	}
}

std::ostream& operator<<(std::ostream &strm, const Plank &plank) {

    strm << std::endl << "--- Plank ---" << std::endl
                      << "Endpoint 1: "   << plank.endpoint_1 << std::endl
                      << "Endpoint 2: "   << plank.endpoint_2 << std::endl
                      << "Reward: "       << plank.reward     << std::endl
                      << "Angle: "        << plank.angle      << std::endl
                      << "Length: "       << plank.length     << std::endl;
    return strm;
};












