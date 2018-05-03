#include "Drone.h"

Drone::Drone(){
	this->position = point_zero;
	this->orientation = 0;
	this->angle_Of_Motion = 0;
	this->speed = DRONE_SPEED;
}

point_t Drone::getPosition(){
	return this->position;
}

bool Drone::update(observation_t observation){
	
	point_t new_Position = {observation.drone_x, observation.drone_y, observation.drone_z};
	this->prev_Position = this->position;

	this->position = new_Position;
	this->command_Done = observation.drone_cmd_done;
	return true;
}


void Drone::wait(double time){
	//NOT IMPLEMENTED YET
}

double Drone::getDistanceToPoint(point_t point){

	double x_Distance = point.x - this->position.x;
	double y_Distance = point.y - this->position.y;

	return sqrt(pow(x_Distance,2) + pow(y_Distance,2));
}


point_t Drone::getInterceptPoint(Robot robot) {
	
	//Drone* drone = new Drone(); 
	double time_Until_Turn = 20 - robot.getTimeAfterTurn();
	double robot_Ori = robot.plank.getAngle();
	point_t robot_Pos = robot.getPosition();
	double x_d = this->position.x;
	double y_d = this->position.y;
	double time_Since_Turn = robot.getTimeAfterTurn();

	if(time_Since_Turn < 2) {
		robot_Ori = robot_Ori - (MATH_PI/2)*(time_Since_Turn) + MATH_PI;

		this->angle_Of_Motion = atan2(robot_Pos.y-this->position.y, robot_Pos.x-this->position.x);
		x_d = this->position.x + (2-time_Since_Turn)*this->speed*cos(this->angle_Of_Motion);
		y_d = this->position.y + (2-time_Since_Turn)*this->speed*sin(this->angle_Of_Motion);
	}
	
	//Math to calculate if direct
	double a = robot_Pos.x; double b = robot.getSpeed(); double c = robot_Ori; double d = robot_Pos.y; double e = x_d; double f = y_d; double g = this->speed;
	double ta =(-sqrt(pow(b,2)*pow(-2*a*cos(c) - 2*d*sin(c) + 2*e*cos(c) + 2*f*sin(c),2) - 4*(-pow(a,2) + 2*a*e - pow(d,2) + 2*d*f - pow(e,2) - pow(f,2))*(-pow(b,2)*pow(sin(c),2) - pow(b,2)*pow(cos(c),2) + pow(g,2))) - b*(-2*a*cos(c) - 2*d*sin(c) + 2*e*cos(c) + 2*f*sin(c)))/(2*(-pow(b,2)*pow(sin(c),2) - pow(b,2)*pow(cos(c),2) + pow(g,2)));
	double tb = (sqrt(pow(b,2)*pow(-2*a*cos(c) - 2*d*sin(c) + 2*e*cos(c) + 2*f*sin(c),2) - 4*(-pow(a,2) + 2*a*e - pow(d,2) + 2*d*f - pow(e,2) - pow(f,2))*(-pow(b,2)*pow(sin(c),2) - pow(b,2)*pow(cos(c),2) + pow(g,2))) - b*(-2*a*cos(c) - 2*d*sin(c) + 2*e*cos(c) + 2*f*sin(c)))/(2*(-pow(b,2)*pow(sin(c),2) - pow(b,2)*pow(cos(c),2) + pow(g,2)));

	double t1 = (std::max)(ta, tb);
	double t2 = 0;

	double x_bf = 0;
	double y_bf = 0;

	if(t1 > (time_Until_Turn+2))
	{
		t1 = time_Until_Turn;
		double x_b1 = robot_Pos.x +time_Until_Turn*robot.getSpeed()*cos(robot_Ori);
		double y_b1 = robot_Pos.y +time_Until_Turn*robot.getSpeed()*sin(robot_Ori);
		double angleDrone1 = atan2(y_b1-y_d, x_b1-x_d);

		double a = x_b1; double b = robot.getSpeed(); double c = robot_Ori+MATH_PI; double d = y_b1; double e = x_d + (time_Until_Turn+2)*this->speed*cos(angleDrone1); double f = x_d + (time_Until_Turn+2)*this->speed*sin(angleDrone1); double g = this->speed;
		ta =(-sqrt(pow(b,2)*pow(-2*a*cos(c) - 2*d*sin(c) + 2*e*cos(c) + 2*f*sin(c),2) - 4*(-pow(a,2) + 2*a*e - pow(d,2) + 2*d*f - pow(e,2) - pow(f,2))*(-pow(b,2)*pow(sin(c),2) - pow(b,2)*pow(cos(c),2) + pow(g,2))) - b*(-2*a*cos(c) - 2*d*sin(c) + 2*e*cos(c) + 2*f*sin(c)))/(2*(-pow(b,2)*pow(sin(c),2) - pow(b,2)*pow(cos(c),2) + pow(g,2)));
		tb = (sqrt(pow(b,2)*pow(-2*a*cos(c) - 2*d*sin(c) + 2*e*cos(c) + 2*f*sin(c),2) - 4*(-pow(a,2) + 2*a*e - pow(d,2) + 2*d*f - pow(e,2) - pow(f,2))*(-pow(b,2)*pow(sin(c),2) - pow(b,2)*pow(cos(c),2) + pow(g,2))) - b*(-2*a*cos(c) - 2*d*sin(c) + 2*e*cos(c) + 2*f*sin(c)))/(2*(-pow(b,2)*pow(sin(c),2) - pow(b,2)*pow(cos(c),2) + pow(g,2)));
		t2 = (std::max)(ta, tb);

		double x_d1 = e;
		double y_d1 = f;
		
		x_bf = x_b1+t2*robot.getSpeed()*cos(robot_Ori+MATH_PI);
		y_bf = y_b1+t2*robot.getSpeed()*sin(robot_Ori+MATH_PI);

		double angleDrone2 = atan2(y_bf-y_d1, x_bf-x_d1);
	}
	else
	{
		x_bf = robot_Pos.x+t1*robot.getSpeed()*cos(robot_Ori);
		y_bf = robot_Pos.y+t1*robot.getSpeed()*sin(robot_Ori);
		double angleDrone1 = atan2(y_bf-y_d, x_bf-x_d);
	}
	point_t intersection;
	intersection.x = x_bf;
	intersection.y = y_bf;
	double t = t1+t2;
	intersection.travel_Time = t;

	return intersection;
}