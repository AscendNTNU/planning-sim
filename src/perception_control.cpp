#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/Pose2D.h"
#include "planning_ros_sim/groundRobotList.h"
#include "planning_ros_sim/groundRobot.h"
#include "planning_ros_sim/droneCmd.h"
#define SIM_IMPLEMENTATION
#define SIM_CLIENT_CODE
#include "ai-sim/sim.h"
#include "ai-sim/gui.h"
#include "AI/structs.h"
#include <stdio.h>

sim_CommandType to_Sim_ActionType(int action){
  switch(action){

    case land_On_Top_Of:
      std::cout << "command is on top" << std::endl;
      return sim_CommandType_LandOnTopOf;
    case land_In_Front_Of:
      std::cout << "command is in front" << std::endl;
      return sim_CommandType_LandInFrontOf;
    case land_At_Point:
      return sim_CommandType_NoCommand;
    case search:
      std::cout << "command is search" << std::endl;
      return sim_CommandType_Search;
    default:
      return sim_CommandType_NoCommand;
  }
}

void droneCmd_chatterCallback(planning_ros_sim::droneCmd droneCmd_msg)
{
  sim_Command command;
  command.x = droneCmd_msg.x;
  command.y = droneCmd_msg.y;
  command.i = droneCmd_msg.target_id;
  command.type = to_Sim_ActionType(droneCmd_msg.cmd);
  command.reward = droneCmd_msg.reward;
  std::cout<< "Sending drone command " << command.i << std::endl;
  sim_send_cmd(&command);
}


int main(int argc, char **argv)
{
  sim_init_msgs(true);
  sim_Observed_State state;
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
  ros::NodeHandle o;
  ros::NodeHandle command_done_node;

  planning_ros_sim::groundRobotList groundrobot_msg;
  geometry_msgs::Pose2D drone_msg;
  std_msgs::Float32 time_msg; 
  std_msgs::Bool command_done_msg;

  ros::Publisher ground_robots_pub = l.advertise<planning_ros_sim::groundRobotList>("groundrobot_chatter", 100);
  ros::Publisher drone_pub = m.advertise<geometry_msgs::Pose2D>("drone_chatter", 100);
  ros::Subscriber droneCmd_sub = n.subscribe("drone_cmd_chatter", 100, droneCmd_chatterCallback);
  ros::Publisher elapsed_time_pub = o.advertise<std_msgs::Float32>("time_chatter",100);
  ros::Publisher command_done_pub = command_done_node.advertise<std_msgs::Bool>("command_done_chatter", 100);
  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    sim_recv_state(&state);
    sim_Observed_State obs_state = state;

    for (int n = 0; n<10; n++){
      groundrobot_msg.groundRobot[n].x = obs_state.target_x[n];
      groundrobot_msg.groundRobot[n].y = obs_state.target_y[n];
      groundrobot_msg.groundRobot[n].theta = obs_state.target_q[n];
      groundrobot_msg.groundRobot[n].visible = obs_state.target_in_view[n];
    }

    drone_msg.x = obs_state.drone_x;
    drone_msg.y = obs_state.drone_y;

    command_done_msg.data = obs_state.drone_cmd_done;
    command_done_pub.publish(command_done_msg);
    ground_robots_pub.publish(groundrobot_msg);
    drone_pub.publish(drone_msg);

    time_msg.data = obs_state.elapsed_time;
    elapsed_time_pub.publish(time_msg);
    ros::spinOnce();
  }

  return 0;
}
