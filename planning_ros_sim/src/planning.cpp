
// %Tag(FULLTEXT)%
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Pose2D.h"
#include "../../../devel/include/planning_ros_sim/groundRobotList.h"
#include "../../../devel/include/planning_ros_sim/groundRobot.h"
#include "../../../devel/include/planning_ros_sim/droneCmd.h"
#include <stdio.h>
#include "AI/AI.h"
#include "AI/structs.h"

using namespace std;

planning_ros_sim::groundRobotList GroundRobots;
geometry_msgs::Pose2D Drone;

AI ai = new AI()

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
	drone_pos.y =10;
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
  
  target_id = None
  std::stack<action_t> current_action_stack = ai->getActionArrayBestTarget()
  std::stack<action_t> updated_target_actions = ai->getActionArrayBestTarget()
  std::stack<action_t> updated_general_actions = ai->getActionArrayBestTarget()



  //Update observationsobservation_t observation = this->updateObservation();
  //Send search command

  //While action not done wait

  //Wait for correct time to act -> here we should adjust based on what we've seen

  //Observe

  while(observation.elapsed_time-action_Start <= action.when_To_Act){
    if(observed_state.target_x[action.target] > 20 || observed_state.target_x[action.target] < 0 ||
      observed_state.target_y[action.target] > 20 || observed_state.target_y[action.target] < 0) {
      std::cout << "Target removed before we could do action. Choose target again." << std::endl;
      return false;
    }
    //Observe
  }

  //If target is not turning act, if not dont do anything(most likely a stupid action if timer is wrong)
  // if(action.target.isMoving()){
  //    Send Land command

  //While action not done wait

  action_t current_action;
  std::stack<action_t> current_action_stack;
  std::stack<action_t> updated_action_stack;

  while (ros::ok()){

    //return a sequence of actions which is restricted to only target_id
    updated_action_stack = ai->getBestActionStack(target_id);

    if(similarity(current_action , updated_target_actions.top())  > threshold){
      current_action_stack = ai->getBestGeneralActionStack();
      //Cancle any current actions
    }

    if(action_done){
      if(chosen_action_stack.empty()){
        current_action_stack = ai->getBestGeneralActionStack();
      }

      current_action = current_action_stack.pop();

      if(!nearby(current_action_stack.where_to_act, target)){
        current_action_stack = updated_action_stack
        current_action = current_action_stack.pop(); 
      }
    } 
    ros::spinOnce();
  }
  return 0;
}
