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
  //Probably need a msg to observation_t function
  ai->update(msg);
}

void drone_chatterCallback(geometry_msgs::Pose2D msg)
{
  //Probably need a msg to observation_t function
  ai->update(msg)
}

planning_ros_sim::droneCmd drone_action(planning_ros_sim::droneCmd drone_action)
{
	drone_pos.x = 10;
	drone_pos.y = 10;
	drone_pos.z = 1;
	action_t command;  
  return drone_pos;
}

bool is_nearby(point_t currentWhereToAct, point_t target) {
	double x1 = currentWhereToAct.x;
	double y1 = currentWhereToAct.y;
	double x2 = target.x;
	double y2 = target.y;
	
	double dist = pow(pow(x2-x1,2) + pow(y2-y1,2), .5);
	return dist < SIMILARITY_THRESHOLD;
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
  
  int target_id = -1;
  Robot target = AI->State->getRobot(1); //Maybe best to have some error handling in case of no robot chosen

  action_t current_action;
  std::stack<action_t> current_action_stack;
  std::stack<action_t> updated_action_stack;

  while (ros::ok()){

    if(action_done){

      //If we've finished our stack get a new one!
      if(chosen_action_stack.empty()){
        current_action_stack = ai->getBestGeneralActionStack();
        target_id = current_action_stack.top().target;
      }
 
      //If we are waiting on the ground robot(ie the robot isn't
      //nearby our landing location) we might aswell update our
      //where_to_act on our current observations.
      if(!is_nearby(current_action.where_to_act, AI->State->getRobot(target_id).getPosition())){
        current_action_stack = updated_action_stack;
      }

      current_action = current_action_stack.pop(); 
      drone_action = drone_action(current_action);
      command_pub.publish(drone_action);

    }
    //returns a stack of the best actions based on the current observation
    updated_action_stack = ai->getBestActionStack(target_id);

    //If the action we are currently doing is significantly different
    //from the best possible action, abort.
    if(similarity(current_action ,updated_action_stack.top())  > SIMILARITY_THRESHOLD){
      current_action_stack = ai->getBestGeneralActionStack();
      cancle_actionlib_action();
    }

    ros::spinOnce();
  }

  return 0;
}
