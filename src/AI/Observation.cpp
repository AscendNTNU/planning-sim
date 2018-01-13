#include "Observation.h"

Observation::Observation(){
    this->drone = Drone();
    
    for(int i = 0; i<10; i++){
        this->robots[i] = Robot();
    }
    
    for(int i = 0; i<4; i++){
        this->obstacles[i] = Robot();
    }
    this->time_Stamp = 0;
}

Drone Observation::getDrone(){
    return this->drone;
}

Robot Observation::getRobot(int index){
    if(index < 0 || index > 10){
        return Robot(-1);
    }
    return this->robots[index];
}

std::array<Robot,10> Observation::getRobots(){
    return this->robots;
}


Robot Observation::getObstacle(int index){
    if(index < 0 || index > 4){
        return  Robot(-1);
    }
    return this->obstacles[index];
}

std::array<Robot,4> Observation::getObstacles(){
    return this->obstacles;
}

float Observation::getTimeStamp(){
    return this->time_Stamp;
}


float Observation::getStateValue() {
    float state_value;
    int num_robots = 0;
    Robot robot;

    for (int i = 0; i < 10; i++) {
        robot = this->robots[i];
        if (robot.getVisibility()) {
            num_robots += 1;
            state_value += robot.current_Plank.getReward();
        }
    }

    this->state_value = state_value / num_robots;
    return this->state_value;
}


bool Observation::update(observation_t observation, float elapsed_time){
    bool drone_Updated = updateDrone(observation, elapsed_time);
    bool robot_Updated = updateRobot(observation, elapsed_time);
    return (drone_Updated and robot_Updated);
}

bool Observation::updateDrone(observation_t observation, float elapsed_time){

    this->time_Stamp = elapsed_time;
    this->drone.update(observation);
    return true;
}

bool Observation::updateRobot(observation_t observation, float elapsed_time){
    point_t position = point_Zero;
    this->time_Stamp = elapsed_time;
    for(int i = 0; i < 10; i++){ // should loop through lenght of observed robots not 10.
        position = (point_t){.x = observation.robot_x[i], .y = observation.robot_y[i]};
        this->robots[i].update(i, position, observation.robot_q[i], this->time_Stamp);
    }
    for(int i = 0; i < 4; i++){
        position = (point_t){.x = 0, .y = 0};
        this->obstacles[i].update(i, position, 0, this->time_Stamp);
    }
    return true;
}
