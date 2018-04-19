#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Point32.h"
#include "planning_ros_sim/groundRobotList.h"
#include "planning_ros_sim/groundRobot.h"
#include "planning_ros_sim/droneCmd.h"
#include "ascend_msgs/AIWorldObservation.h"
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
sim_Observed_State state;

int mapAIIndexToSimIndex(sim_Command command){
    float nearest_distance = 10000;
    float target = 0;
    point_t command_position = point_zero;

    for(int i=0; i<Num_Targets; i++){
        float x_Distance = command.x - state.target_x[i];
        float y_Distance = command.y - state.target_y[i];
        float distance = sqrt(pow(x_Distance,2) + pow(y_Distance,2));
        if(distance < nearest_distance){
            nearest_distance = distance;
            target = i;
        }
    }
    return target;
}

sim_Command action_ROS2Sim(GoalType goal) {
  sim_Command command;
  command.x = goal.dx + state.drone_x;
  command.y = goal.dy + state.drone_y;
  command.i = mapAIIndexToSimIndex(command);
  command.reward = goal.reward;

    switch(goal.cmd){

    case ascend_msgs::ControlFSMGoal::GO_TO_XYZ:
      command.type = sim_CommandType_Search;
      break;
    case ascend_msgs::ControlFSMGoal::LAND_ON_TOP_OF:
      command.type = sim_CommandType_LandOnTopOf;
      
      break;
    case ascend_msgs::ControlFSMGoal::LAND_AT_POINT:
      command.type = sim_CommandType_Land;
      break;
    case ascend_msgs::ControlFSMGoal::SEARCH:
      command.type = sim_CommandType_Search;
      break;
    case ascend_msgs::ControlFSMGoal::TAKEOFF:
      command.type = sim_CommandType_TakeOff;
      break;
    default:
      command.type = sim_CommandType_NoCommand;
      break;
  }

  return command;
}

//Accepts new goal from client when recieved
void newGoalCB(ActionServerType* server){
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
void preemptCB(ActionServerType* server){
    //Abort whatever you are doing first!
    // Should we send no_command to sim?
    ROS_WARN("Preempted!");
}

int main(int argc, char **argv)
{
  // Initialize sim-messages
  sim_init_msgs(true);
  //sim_Observed_State state; // Made state global as it is used in a callback function
  sim_Command cmd;
  cmd.type = sim_CommandType_NoCommand;
  cmd.x = 0;
  cmd.y = 0;
  cmd.i = 0;

    // Initialize ros-messages
    ros::init(argc, argv, "perception_control");
    ros::NodeHandle nh;

    // Define publishers
    ros::Publisher ai_sim_pub = nh.advertise<ascend_msgs::AIWorldObservation>("/ai/sim", 1);
    
    // Define action server
    ActionServerType server(nh, "control_action_server", false);
    server.registerGoalCallback(boost::bind(newGoalCB, &server));
    server.registerPreemptCallback(boost::bind(preemptCB, &server));
    server.start();

    ros::Rate rate(20.0);
    while (ros::ok()){
        ros::spinOnce();
        // Collect new observation
        sim_recv_state(&state);


        ascend_msgs::AIWorldObservation observation;
        observation.elapsed_time = state.elapsed_time;

        observation.drone_position.x = state.drone_x;
        observation.drone_position.y = state.drone_y;
        observation.drone_position.z = state.drone_z;

        for(int i = 0; i < Num_Targets; i++) {
            ascend_msgs::GRState robot;
            robot.x = state.target_x[i];
            robot.y = state.target_y[i];
            robot.theta = state.target_q[i];
            robot.visible = true;
            // robot.visible = state.target_in_view[i];
            observation.ground_robots[i] = robot;
        }

        for(int i = 0; i < Num_Obstacles; i++) {
            ascend_msgs::GRState robot;
            robot.x = state.obstacle_x[i];
            robot.y = state.obstacle_y[i];
            robot.theta = state.obstacle_q[i];
            robot.visible = true;
            observation.obstacle_robots[i] = robot;
        }

        // Check if command done in sim and that
        // there exists an active goal and it is completed
        if(state.drone_cmd_done && server.isActive()) {
            server.setSucceeded();
        }
        observation.header.seq = 0;
        ai_sim_pub.publish(observation);
        rate.sleep();
    }
    return 0;
}
