
// %Tag(FULLTEXT)%
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Pose2D.h"
#include "../../../devel/include/planning_ros_sim/groundRobotList.h"
#include "../../../devel/include/planning_ros_sim/groundRobot.h"
#include "../../../devel/include/planning_ros_sim/droneCmd.h"
#define SIM_IMPLEMENTATION
#include "ai-sim/sim.h"
#include <stdio.h>

 planning_ros_sim::groundRobotList GroundRobots;
 geometry_msgs::Pose2D Drone;


void groundRobot_chatterCallback(planning_ros_sim::groundRobotList msg)
{for (int i = 0; i < 10; i++ ){
	//This works. Is receiving the list of all groundrobots(not with stick) with x, y and theta.
	GroundRobots.groundRobot[i].x = msg.groundRobot[i].x;
	GroundRobots.groundRobot[i].y = msg.groundRobot[i].y;
	GroundRobots.groundRobot[i].theta = msg.groundRobot[i].theta;
	std::cout << "Ground robot " << i << ": "<< msg.groundRobot[i].x<< "x position. " << std::endl;

}
}

void drone_chatterCallback(geometry_msgs::Pose2D msg)
{

	Drone.x = msg.x;
	Drone.y = msg.y;
	std::cout << "Drone position, x : " << Drone.x << " y : "<<Drone.y <<std::endl;

};


/*
enum sim_CommandType
{
    sim_CommandType_NoCommand = 0,   // continue doing whatever you are doing
    sim_CommandType_LandOnTopOf,     // trigger one 45 deg turn of robot (i)
    sim_CommandType_LandInFrontOf,   // trigger one 180 deg turn of robot (i)
    sim_CommandType_Track,           // follow robot (i) at a constant height
    sim_CommandType_Search,          // ascend to 3 meters and go to (x, y)
    sim_CommandType_Debug
};

struct sim_Command
{
    sim_CommandType type;
    float x;
    float y;
    int i;
    int reward;
    float heatmap[pixels_each_meter*pixels_each_meter*20*20];

};*/

planning_ros_sim::droneCmd drone_action(planning_ros_sim::droneCmd drone_pos)
{
	drone_pos.x = 1;
	drone_pos.y =1 ;
	drone_pos.z = 1;
	sim_Command command;
    command.type = sim_CommandType_LandInFrontOf;
	drone_pos.cmd = command.type;
	//std::cout << "inni drone action" <<std::endl;
	return drone_pos;
	//droneCmd_pub.publish(drone_pos);

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "planning");

  ros::NodeHandle l;
  ros::NodeHandle m;
  ros::NodeHandle n;

  ros::Subscriber groundRobot_sub = l.subscribe("groundrobot_chatter", 1000, groundRobot_chatterCallback);
  ros::Subscriber drone_sub = m.subscribe("drone_chatter", 1000, drone_chatterCallback);
  ros::Publisher droneCmd_pub = n.advertise<planning_ros_sim::droneCmd>("drone_cmd_chatter", 1000);


  planning_ros_sim::droneCmd drone_pos;
  sim_Command command;
  while (ros::ok()){
	drone_pos = drone_action(drone_pos);

  droneCmd_pub.publish(drone_pos);

	ros::spinOnce();
  }


  return 0;
}
