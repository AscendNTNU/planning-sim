#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Pose2D.h"
#include <vector>
#include "../../../devel/include/planning_ros_sim/groundRobotList.h"
#include "../../../devel/include/planning_ros_sim/groundRobot.h"
#include "../../../devel/include/planning_ros_sim/droneCmd.h"
#define SIM_IMPLEMENTATION
#define SIM_CLIENT_CODE
#include "ai-sim/sim.h"
#include "ai-sim/gui.h"
#include <stdio.h>
#include <sstream>
using namespace std;

void droneCmd_chatterCallback(planning_ros_sim::droneCmd droneCmd_msg)
{
  sim_Command command;
  command.x = droneCmd_msg.x;
  command.y = droneCmd_msg.y;
  command.type = (sim_CommandType) droneCmd_msg.cmd;
  sim_send_cmd(&command);

}


int main(int argc, char **argv)
{
  sim_init_msgs(true);
  sim_State state;
  bool running = true;

  sim_Command cmd;
  cmd.type = sim_CommandType_NoCommand;
  cmd.x = 0;
  cmd.y = 0;
  cmd.i = 0;

  ros::init(argc, argv, "perception_control");
  ros::NodeHandle l;
  ros::NodeHandle m;
  ros::NodeHandle n;
  planning_ros_sim::groundRobotList groundrobot_msg;
  geometry_msgs::Pose2D drone_msg;
  ros::Publisher ground_robots_pub = l.advertise<planning_ros_sim::groundRobotList>("groundrobot_chatter", 1000);
  ros::Publisher drone_pub = m.advertise<geometry_msgs::Pose2D>("drone_chatter", 1000);
  ros::Subscriber droneCmd_sub = n.subscribe("drone_cmd_chatter", 1000, droneCmd_chatterCallback);
  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    std::cout << "kommer hit " <<std::endl;
    sim_recv_state(&state);
    sim_Observed_State obs_state = sim_observe_state(state);

  	for (int n = 0; n<10 ; n++ )
  	{
  		groundrobot_msg.groundRobot[n].x = obs_state.target_x[n];
  		groundrobot_msg.groundRobot[n].y = obs_state.target_y[n];
  		groundrobot_msg.groundRobot[n].theta = obs_state.target_q[n];
  	}
    drone_msg.x = obs_state.drone_x;
    drone_msg.y = obs_state.drone_y;
    ground_robots_pub.publish(groundrobot_msg);
    drone_pub.publish(drone_msg);
    ros::spinOnce();
  }

  return 0;
}
