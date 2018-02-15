#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/Pose2D.h"
#include "planning_ros_sim/groundRobotList.h"
#include "planning_ros_sim/groundRobot.h"
#include "planning_ros_sim/droneCmd.h"
#include <actionlib/client/simple_action_client.h>
#include <stdio.h>
#include "AI/AIController.h"
#include "AI/structs.h"
#include "AI/StateMachine/PlanningFSM.h"

planning_ros_sim::groundRobotList GroundRobots;
geometry_msgs::Pose2D Drone;

float elapsed_time = 0;
bool action_done = true;
World world = World(0);
PlanningFSM fsm;

void time_chatterCallback(std_msgs::Float32 msg) {
    elapsed_time = (float)msg.data;
}

void groundRobot_chatterCallback(const planning_ros_sim::groundRobotList &msg) {
    observation_t robotObs = observation_Empty;
    for(int i = 0; i < 10; i++) {
            robotObs.robot_x[i] = msg.groundRobot[i].x;
            robotObs.robot_y[i] = msg.groundRobot[i].y;
            robotObs.robot_q[i] = msg.groundRobot[i].theta;
            robotObs.robot_visible[i] = msg.groundRobot[i].visible;
    }

    Observation obs = fsm.getObservation();
    obs.updateRobot(robotObs, elapsed_time);
    fsm.updateObservation(obs);
    //ai_controller.observation.updateRobot(robotObs, elapsed_time);

}

void drone_chatterCallback(geometry_msgs::Pose2D msg) {
    observation_t droneObs = observation_Empty;
    droneObs.drone_x = msg.x;
    droneObs.drone_y = msg.y;
    
    Observation obs = fsm.getObservation();
    obs.updateDrone(droneObs, elapsed_time);
    fsm.updateObservation(obs);
    //ai_controller.observation.updateDrone(droneObs, elapsed_time);
}

void command_done_chatterCallback(std_msgs::Bool msg) {
    action_done = (bool)msg.data;
}

planning_ros_sim::droneCmd to_ROS_Command(action_t action) {
    planning_ros_sim::droneCmd command;
    command.x = action.where_To_Act.x;
    command.y = action.where_To_Act.y;
    command.z = 0;
    command.cmd = (int)action.type;
    command.target_id = action.target;
    command.reward = action.reward;

    return command;
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "planning");
    ros::NodeHandle nh;

    ros::Subscriber time_sub = nh.subscribe("time_chatter", 1000, time_chatterCallback);
    ros::Subscriber ground_robot_sub = nh.subscribe("groundrobot_chatter", 1000, groundRobot_chatterCallback);
    ros::Subscriber drone_sub = nh.subscribe("drone_chatter", 1000, drone_chatterCallback);
    ros::Subscriber command_done_sub = nh.subscribe("command_done_chatter", 100, command_done_chatterCallback);
    
    ros::Publisher command_pub = nh.advertise<planning_ros_sim::droneCmd>("drone_cmd_chatter", 1000);
    
    while (ros::ok()) {
        ros::Duration(0.4).sleep();
        ros::spinOnce();

        if(action_done) {

            fsm.loopCurrentState();
            action_t action = fsm.getCurrentAction();

            if(action.type != no_Command){
                printf("sending command\n");
                planning_ros_sim::droneCmd drone_action = to_ROS_Command(action);
                command_pub.publish(drone_action);
            }
        }
    }
}
