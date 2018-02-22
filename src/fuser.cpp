#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Bool.h"


void tracker_chatterCallback(ascend_msgs::DetectedRobotsGlobalPositions::ConstPtr msg){
    observation_t robotObs = observation_Empty;
    for(int i = 0; i < 10; i++) {
    	if(i < (int)msg->count){
            robotObs.robot_x[i] = msg->global_robot_position[i].x;
            robotObs.robot_y[i] = msg->global_robot_position[i].y;
            robotObs.robot_q[i] = msg->direction[i];
            robotObs.robot_visible[i] = true;
        }
        else{
            robotObs.robot_visible[i] = false;
        }
    }
    elapsed_time = 10;
    ai_controller.observation.updateRobot(robotObs, elapsed_time);   
}


int main(int argc, char **argv)
{

}