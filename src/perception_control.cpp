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
#include <actionlib/server/simple_action_server.h>
#include <ascend_msgs/ControlFSMAction.h>

//Typedefs
using ActionServerType = actionlib::SimpleActionServer<ascend_msgs::ControlFSMAction>;
using GoalType = ascend_msgs::ControlFSMGoal;


sim_Command action_ROS2Sim(GoalType goal){
  sim_Command command;

  switch(goal.cmd){

    case ascend_msgs::ControlFSMGoal::GO_TO_XYZ:
      // What happens here
      break;
    case ascend_msgs::ControlFSMGoal::LAND_ON_TOP_OF:
      command.type = sim_CommandType_LandOnTopOf;
      break;
    case ascend_msgs::ControlFSMGoal::LAND_AT_POINT:
      command.type = sim_CommandType_LandInFrontOf; 
      // How do we know if we should send LandInFrontOf of LandAtPoint?
      break;
    case ascend_msgs::ControlFSMGoal::SEARCH:
      command.type = sim_CommandType_Search;
      break;
    default:
      command.type = sim_CommandType_NoCommand;
      break;
  }

  command.x = goal.x;
  command.y = goal.y;
  command.i = goal.target_id;
  command.reward = goal.reward;
  return command;
}

//Accepts new goal from client when recieved
void newGoalCB(ActionServerType* server) {
  //Check there really is a new one available
  if(!server->isNewGoalAvailable()) return;
  //Accept the new goal
  ascend_msgs::ControlFSMGoal goal = *server->acceptNewGoal();
  //Check that the client hasn't cancelled the request already calls
  if(server->isPreemptRequested()) {
    //Goal is already stopped by client
    // Should we send no_command to sim?
    return;
  }
  //Read the goal and do something with it!
  sim_Command command = action_ROS2Sim(goal);
  sim_send_cmd(&command);
}

//Terminates current goal when requested.
void preemptCB(ActionServerType* server) {
  //Abort whatever you are doing first!
  // Should we send no_command to sim?
  ROS_WARN("Preempted!");
}

int main(int argc, char **argv)
{
  // Initialize sim-messages
  sim_init_msgs(true);
  sim_Observed_State state;
  sim_Command cmd;
  cmd.type = sim_CommandType_NoCommand;
  cmd.x = 0;
  cmd.y = 0;
  cmd.i = 0;

  // Initialize ros-messages
  ros::init(argc, argv, "perception_control");
  ros::NodeHandle nh;
  planning_ros_sim::groundRobotList groundrobot_msg;
  geometry_msgs::Pose2D drone_msg;
  std_msgs::Float32 time_msg; 

  // Define publishers
  ros::Publisher ground_robots_pub = nh.advertise<planning_ros_sim::groundRobotList>("groundrobot_chatter", 100);
  ros::Publisher drone_pub = nh.advertise<geometry_msgs::Pose2D>("drone_chatter", 100);
  ros::Publisher elapsed_time_pub = nh.advertise<std_msgs::Float32>("time_chatter",100);
  ros::Publisher command_done_pub = nh.advertise<std_msgs::Bool>("command_done_chatter", 100);

  // Define action server
  ActionServerType server(nh, "control_action_server", false);
  server.registerGoalCallback(boost::bind(newGoalCB, &server));
  server.registerPreemptCallback(boost::bind(preemptCB, &server));
  server.start();

  ros::Rate rate(30.0);
  while (ros::ok()) {
    ros::spinOnce();

    // Collect new observation
    sim_recv_state(&state);

    // Generate messages from observation
    drone_msg.x = state.drone_x;
    drone_msg.y = state.drone_y;
    for (int n = 0; n<10; n++) { // Will it always be 10 targets in the observed state? What if not?
      groundrobot_msg.groundRobot[n].x = state.target_x[n];
      groundrobot_msg.groundRobot[n].y = state.target_y[n];
      groundrobot_msg.groundRobot[n].theta = state.target_q[n];
      groundrobot_msg.groundRobot[n].visible = obs_state.target_in_view[n];
    }

    // Check if command done in sim and that
    // there exists an active goal and it is completed
    if(state.drone_cmd_done && server.isActive()) {
      server.setSucceeded();
    }

    ground_robots_pub.publish(groundrobot_msg);
    drone_pub.publish(drone_msg);

    time_msg.data = state.elapsed_time;
    elapsed_time_pub.publish(time_msg);

    rate.sleep();
  }
  return 0;
}
