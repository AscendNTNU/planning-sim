//ROS includes
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/Pose2D.h"
#include "planning_ros_sim/groundRobotList.h"
#include "planning_ros_sim/groundRobot.h"
#include "planning_ros_sim/droneCmd.h"
#include <actionlib/client/simple_action_client.h>

//Code specific includes
#include <stdio.h>
#include "AI/AI.h"
#include "AI/structs.h"

const float SIMILARITY_THRESHOLD = 1;

float elapsed_time = 0;
bool action_done = true;

planning_ros_sim::groundRobotList GroundRobots;
geometry_msgs::Pose2D Drone;

AI ai = AI();
World world = World(0);


//Callback functions
void time_chatterCallback(std_msgs::Float32 msg){
    elapsed_time = (float)msg.data;
}


void groundRobot_chatterCallback(const planning_ros_sim::groundRobotList &msg)
{
    observation_t robotObs;
    for(int i = 0; i < 10; i++) {
          robotObs.robot_x[i] = msg.groundRobot[i].x;
          robotObs.robot_y[i] = msg.groundRobot[i].y;
          robotObs.robot_q[i] = msg.groundRobot[i].theta;
    }
    ai.updateRobot(robotObs, elapsed_time);
}

void drone_chatterCallback(geometry_msgs::Pose2D msg)
{
    observation_t droneObs;
    droneObs.drone_x = msg.x;
    droneObs.drone_y = msg.y;
    ai.updateDrone(droneObs, elapsed_time);
}

void command_done_chatterCallback(std_msgs::Bool msg){
    action_done = (bool)msg.data;
}

//helper functions
//Converts AI's drone command to the correct ROS message type
planning_ros_sim::droneCmd to_ROS_Command(action_t action)
{
    planning_ros_sim::droneCmd command;
    command.x = action.where_To_Act.x;
    command.y = action.where_To_Act.x;
    command.z = 0;
    command.cmd = (int)action.type;
    command.target_id = action.target;
    command.reward = action.reward;

    return command;
}

//Checks if a point is near another point and returns true if it is below the SIMILARITY_THRESHOLD
bool is_nearby(point_t current_Where_To_Act, point_t target) {
	  double x1 = current_Where_To_Act.x;
	  double y1 = current_Where_To_Act.y;
	  double x2 = target.x;
	  double y2 = target.y;
	  
	  double dist = pow(pow(x2-x1,2) + pow(y2-y1,2), .5);
	  return dist < SIMILARITY_THRESHOLD;
}

//Checks similarity between actions, currently just checks if they are identical or not
float similarity(action_t action1 ,action_t action2){
    if(is_nearby(action1.where_To_Act, action2.where_To_Act)){
        return 1;
    }
    return 0;
}

//main
int main(int argc, char **argv)
{
    //Ros initialization
    ros::init(argc, argv, "planning");
    ros::NodeHandle ground_robot_node;
    ros::NodeHandle drone_node;
    ros::NodeHandle command_node;
    ros::NodeHandle time_node;
    ros::NodeHandle command_done_node;

    ros::Subscriber time_sub = time_node.subscribe("time_chatter", 1000, time_chatterCallback);
    ros::Subscriber ground_robot_sub = ground_robot_node.subscribe("groundrobot_chatter", 1000, groundRobot_chatterCallback);
    ros::Subscriber drone_sub = drone_node.subscribe("drone_chatter", 1000, drone_chatterCallback);
    ros::Subscriber command_done_sub = command_done_node.subscribe("command_done_chatter", 100, command_done_chatterCallback);
    
    ros::Publisher command_pub = command_node.advertise<planning_ros_sim::droneCmd>("drone_cmd_chatter", 1000);

    planning_ros_sim::droneCmd drone_action;
    
    //Initialization of variables
    int target_id = 0;
    Robot target = ai.state.getRobot(1);
    action_t current_action;
    std::stack<action_t> current_action_stack;
    world.startTimer();

    //Wait for real data before starting AI algorithm
    while (ros::ok() and ai.state.getRobot(0).getPosition().x == 0){
        ros::Duration(0.2).sleep();
        ros::spinOnce();
    }

    //AI algorithm
    while (ros::ok()){
        ros::Duration(0.4).sleep();
        ros::spinOnce();

        if(action_done){
            target = ai.state.getRobot(target_id);
            
            //If the stack of actions is empty get a new one!
            if(current_action_stack.empty()){
                current_action_stack = ai.getBestGeneralActionStack();
                target_id = current_action_stack.top().target;
                target = ai.state.getRobot(target_id);
            }

            //If we are waiting on the ground robot(ie the robot isn't
            //nearby our landing location) we might aswell update our
            //where_to_act on our current observations.
            else if(current_action_stack.top().type != search && !is_nearby(current_action_stack.top().where_To_Act, target.getPosition())){
                current_action_stack.push(ai.getBestActionStack(target).top());

            }

            //Send next action to drone
            current_action = current_action_stack.top();
            drone_action = to_ROS_Command(current_action);
            command_pub.publish(drone_action);
            current_action_stack.pop();
        }
    }
    return 0;
}
