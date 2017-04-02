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
  int i = 0;
  if (i == 0){
    command.type =  sim_CommandType_Search;
    command.x = 10;
    command.y = 10;
    i = 1;
  } else {
    command.type = sim_CommandType_NoCommand;
}
  //cout << "run gui" << "command" << command.type <<"x"<< command.x << "y"<<command.y<< endl;
  std::cout << "Should send command nowww" << std::endl;
  sim_send_cmd(&command);
  //std::cout << " Drone cmd x "<< droneCmd_msg.x <<std::endl;

}



int main(int argc, char **argv)
{
  sim_init_msgs(true);
  sim_State state;
  bool running = true;
  //sim_State sim = sim_init(12345);
  //sim_Observed_State state = sim_observe_state(sim);

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
  if (not ground_robots_pub){
    std::cout << "ground_robots_pub error"<<std::endl;
  }
  ros::Publisher drone_pub = m.advertise<geometry_msgs::Pose2D>("drone_chatter", 1000);
  if (not drone_pub){
    std::cout << "Drone_pub error"<<std::endl;
  }

  ros::Subscriber droneCmd_sub = n.subscribe("drone_cmd_chatter", 1000, droneCmd_chatterCallback);



  ros::Rate loop_rate(10);


  while (ros::ok())
  {
    std::cout << "kommer hit " <<std::endl;
    sim_recv_state(&state);
    //sim_recv_state returner en bool, men fyller adressa til state med info om staten,
    //er det good å gi sim_observe_state barestate objektet? Den skal ta inn en sim_state. er state en sånn?

    printf("Recv state %.2f\n", state.elapsed_time);
    sim_Observed_State obs_state = sim_observe_state(state);
    for(int o = 0; o < 10; o++){
      cout<< "robot id for state:"<< o << " x_pos: " << state.robots[o].x<<endl;
    }

    /*
    struct sim_Observed_State
    {
        float elapsed_time;
        float drone_x;
        float drone_y;
        bool  drone_cmd_done;

        bool  target_in_view[Num_Targets];
        bool  target_reversing[Num_Targets];
        bool  target_removed[Num_Targets];
        float target_reward[Num_Targets];

        float target_x[Num_Targets];
        float target_y[Num_Targets];
        float target_q[Num_Targets];

        float obstacle_x[Num_Obstacles];
        float obstacle_y[Num_Obstacles];
        float obstacle_q[Num_Obstacles];
    }; */
    //std::cout << "in the while loop" <<std::endl;

    /*for ( int i=0; i<1000; i++ )
  	{
  	sim = sim_tick(sim, cmd);
    }

  	state = sim_observe_state(sim);*/

  	for (int n = 0; n<10 ; n++ )
  	{
  		groundrobot_msg.groundRobot[n].x = obs_state.target_x[n];
  		groundrobot_msg.groundRobot[n].y = obs_state.target_y[n];
  		groundrobot_msg.groundRobot[n].theta = obs_state.target_q[n];


  	}
    drone_msg.x = obs_state.drone_x;
    drone_msg.y = obs_state.drone_y;

    std::cout << groundrobot_msg.groundRobot[0].x <<std::endl;

    ground_robots_pub.publish(groundrobot_msg);
    drone_pub.publish(drone_msg);
    ros::spinOnce();
  }

  return 0;
}
