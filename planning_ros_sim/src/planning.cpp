#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Pose2D.h"
#include "../../../devel/include/planning_ros_sim/groundRobotList.h"
#include "../../../devel/include/planning_ros_sim/groundRobot.h"
#include "../../../devel/include/planning_ros_sim/droneCmd.h"
#include <actionlib/client/simple_action_client.h>

#include <stdio.h>
#include "AI/AI.h"
#include "AI/structs.h"



using namespace std;

const float SIMILARITY_THRESHOLD = 10;

planning_ros_sim::groundRobotList GroundRobots;
geometry_msgs::Pose2D Drone;

AI* ai = new AI();

//This must correspond to sim.h
 enum sim_CommandType
 {
     sim_CommandType_NoCommand = 0,   // continue doing whatever you are doing
     sim_CommandType_LandOnTopOf,     // trigger one 45 deg turn of robot (i)
     sim_CommandType_LandInFrontOf,   // trigger one 180 deg turn of robot (i)
     sim_CommandType_Track,           // follow robot (i) at a constant height
     sim_CommandType_Search,          // ascend to 3 meters and go to (x, y)
     sim_CommandType_Land,
     sim_CommandType_Debug
 };

void groundRobot_chatterCallback(const planning_ros_sim::groundRobotList &msg)
{
  ai->robot_update(msg);
}

void drone_chatterCallback(geometry_msgs::Pose2D msg)
{
	
  ai->drone_update(msg)

};

planning_ros_sim::droneCmd drone_action(planning_ros_sim::droneCmd drone_action)
{
	drone_pos.x = 10;
	drone_pos.y = 10;
	drone_pos.z = 1;
	sim_Command command;
  command.type = sim_CommandType_LandInFrontOf;
	drone_pos.cmd = command.type;
  return drone_pos;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "planning");
  ros::NodeHandle ground_robot_node;
  ros::NodeHandle drone_node;
  ros::NodeHandle command_node;

  ros::Subscriber ground_robot_sub = ground_robot_node.subscribe("groundrobot_chatter", 1000, groundRobot_chatterCallback);
  ros::Subscriber drone_sub = drone_node.subscribe("drone_chatter", 1000, drone_chatterCallback);
  ros::Publisher command_pub = command_node.advertise<planning_ros_sim::droneCmd>("drone_cmd_chatter", 1000);

  planning_ros_sim::droneCmd drone_action;
  sim_Command command;
  
  target_id = -1;

  action_t current_action;
  std::stack<action_t> current_action_stack;
  std::stack<action_t> updated_action_stack;

  while (ros::ok()){

    if(action_done){

      //If we've finished our stack get a new one!
      if(chosen_action_stack.empty()){
        current_action_stack = ai->getBestGeneralActionStack();
      }
 
      //If we are waiting on the ground robot(ie the robot isn't
      //nearby our landing location) we might aswell update our
      //where_to_act on our current observations.
      if(!nearby(current_action_stack.where_to_act, target)){
        current_action_stack = updated_action_stack;
        current_action = current_action_stack.pop(); 
      }

      drone_action = drone_action(current_action);
      command_pub.publish(drone_action);

    }
    //returns a stack of the best actions based on the current observation
    updated_action_stack = ai->getBestActionStack(target_id);

    //If the action we are currently doing is significantly different
    //from the best possible action, abort.
    if(similarity(current_action ,updated_target_actions.top())  > SIMILARITY_THRESHOLD){
      current_action_stack = ai->getBestGeneralActionStack();
      cancle_actionlib_action();
    }

    ros::spinOnce();
  }

  return 0;
}
