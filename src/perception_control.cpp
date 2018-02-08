#include "ros/ros.h"

#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/Pose2D.h"
#include <ascend_msgs/DetectedRobotsGlobalPositions.h>
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
      command.type = sim_CommandType_Search;
      break;
    case ascend_msgs::ControlFSMGoal::LAND_ON_TOP_OF:
      command.type = sim_CommandType_LandOnTopOf;
      break;
    case ascend_msgs::ControlFSMGoal::LAND_AT_POINT:
      command.type = sim_CommandType_LandInFrontOf; 
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
  ros::NodeHandle node;



  geometry_msgs::Pose2D drone_msg;
  std_msgs::Float32 time_msg; 


  ros::Publisher ground_robots_pub = node.advertise<ascend_msgs::DetectedRobotsGlobalPositions>("globalGroundRobotPosition", 100);
  ros::Publisher drone_pub = node.advertise<geometry_msgs::Pose2D>("drone_chatter", 100);
  ros::Publisher elapsed_time_pub = node.advertise<std_msgs::Float32>("time_chatter",100);
  ros::Rate loop_rate(10);

  // Define action server
  ActionServerType server(node, "control_action_server", false);
  server.registerGoalCallback(boost::bind(newGoalCB, &server));
  server.registerPreemptCallback(boost::bind(preemptCB, &server));
  server.start();

  ros::Rate rate(30.0);
  while (ros::ok()) {
    ros::spinOnce();

    // Collect new observation
    sim_recv_state(&state);
    sim_Observed_State obs_state = state;
    
    ascend_msgs::DetectedRobotsGlobalPositions groundrobot_msg;
    groundrobot_msg.count = 10;
    for (int n = 0; n<10; n++){
      geometry_msgs::Point32 robot_position;
      std_msgs::Float32 direction;
      robot_position.x = obs_state.target_x[n];
      robot_position.y = obs_state.target_y[n];
      direction.data = obs_state.target_q[n];
      groundrobot_msg.global_robot_position.push_back(robot_position);
      groundrobot_msg.direction.push_back(obs_state.target_q[n]);
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
