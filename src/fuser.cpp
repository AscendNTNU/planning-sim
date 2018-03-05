#include <array>
#include <vector>

#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Time.h"

#include "ascend_msgs/DetectedRobotsGlobalPositions.h"
#include "ascend_msgs/AIWorldObservation.h"

#include "AI/Robot.h"
#include "AI/World.h"

World world = World(0);

std::array<Robot, 10> robots;
std::vector<Robot> new_robots;
std::array<Robot, 4> obstacle_robots;
std::vector<Robot> new_obstacle_robots;
ros::Time start_time;
float elapsed_time = -1; // This is set by a callback if we are using ai-sim
bool using_sim = false;


float TIMEOUT_OBSERVATION = 5;

point_t drone_position = point_zero;

//Can only handle 10 robots in one message.
void groundRobotCallback(ascend_msgs::DetectedRobotsGlobalPositions::ConstPtr msg){
    for(int i = 0; i < (int)msg->count; i++) {
        Robot robot;

        point_t position;
        position.x = msg->global_robot_position[i].x;
        position.y = msg->global_robot_position[i].y;

        float q = msg->direction[i];
        float time = msg->header.stamp.sec-start_time.sec;
        bool visible = true;

        robot.update(i, position, q , time, visible);
        new_robots.push_back(robot);
    }
}

void startTimeCallback(std_msgs::Time::ConstPtr msg){
    start_time = msg->data;
}

void dronePositionCallback(geometry_msgs::PoseStamped::ConstPtr msg){
    drone_position.x = msg->pose.position.x;
    drone_position.y = msg->pose.position.y;
    drone_position.z = msg->pose.position.z;
}

void aiSimCallback(ascend_msgs::AIWorldObservation::ConstPtr obs){

    elapsed_time = obs->elapsed_time;

    drone_position.x = obs->drone_position.x;
    drone_position.y = obs->drone_position.y;
    drone_position.z = obs->drone_position.z;

    int i = 0;
    for(auto it = obs->ground_robots.begin(); it != obs->ground_robots.end(); it++, i++) {
        Robot robot;

        point_t position;
        position.x = it->x;
        position.y = it->y;

        float q = it->theta;
        float time = obs->header.stamp.sec - start_time.sec;
        bool visible = it->visible;

        robot.update(i, position, q , time, visible);
        new_robots.push_back(robot);
    }
}

double distanceBetweenRobots(Robot r1, Robot r2) {
    return sqrt(pow(r1.getPosition().x - r2.getPosition().x,2)+pow(r1.getPosition().y - r2.getPosition().y,2));
}

int nearestNeighbor(Robot robot) {
    double min_distance = 1000000;
    int index = -1;
    for(auto it = robots.begin(); it != robots.end(); it++){
        Robot robot = *it;
        int counter = 0;
        if(distanceBetweenRobots(robot, robots[counter]) < min_distance) {
            min_distance = distanceBetweenRobots(robot, robots[counter]);
            index = counter;
        }
        counter++;
    }
    return index;
}

void updateRobot(Robot new_robot){
    int nearest_robot_index = nearestNeighbor(new_robot);
    robots[nearest_robot_index].update(new_robot);
}

float calcCurrentTime(float seconds){
    if(elapsed_time == -1){
        return seconds-start_time.sec; 
    }
    return elapsed_time;
}

int main(int argc, char **argv){

    int counter = 0;

    for(auto it = robots.begin(); it != robots.end(); it++){
        *it = Robot(counter);
        counter++;
    }

    // Initialize ros-messages
    ros::init(argc, argv, "fuser");
    ros::NodeHandle node;
    // geometry_msgs::Pose2D drone_msg;
    std_msgs::Float32 time_msg; 

    // ros::Publisher ground_robots_pub = node.advertise<ascend_msgs::AIWorldObservation>("AIWorldObservation", 1);
    ros::Subscriber tracker_sub = node.subscribe("globalGroundRobotPosition", 100, groundRobotCallback);
    ros::Subscriber start_time_sub = node.subscribe("/time_chatter/start_time", 1, startTimeCallback);
    ros::Subscriber drone_sub = node.subscribe("/mavros/local_position_pose", 1, dronePositionCallback);
    ros::Subscriber sim_sub = node.subscribe("/ai/sim", 1, aiSimCallback);

    ros::Publisher observation_pub = node.advertise<ascend_msgs::AIWorldObservation>("AIWorldObservation", 1);

    ros::Rate rate(30.0);

    while (ros::ok()) {
        ros::spinOnce();

        for(auto it = new_robots.begin(); it != new_robots.end(); it++){
            updateRobot(*it);
        }

        for(auto it = new_obstacle_robots.begin(); it != new_obstacle_robots.end(); it++){
            // updateRobot(*it);
        }

        ascend_msgs::AIWorldObservation observation;
        float current_time = calcCurrentTime(ros::Time::now().sec);
        observation.elapsed_time = current_time;

        for(auto it = robots.begin(); it != robots.end(); it++){

            if(observation.elapsed_time - it->getTimeLastSeen() > TIMEOUT_OBSERVATION){
                it->setVisible(false);
            }

            ascend_msgs::GRState robot;

            robot.header = observation.header;

            point_t position = it->getPosition();
            robot.x = position.x;
            robot.y = position.y;
            robot.theta = it->getOrientation();
            robot.visible = it->getVisible();
            observation.ground_robots[it->getIndex()] = robot;
        }

        // for(auto it = obstacle_robots.begin(); it != obstacle_robots.end(); it++){

        //     if(observation.elapsed_time - it->getTimeLastSeen() > TIMEOUT_OBSERVATION){
        //         it->setVisible(false);
        //     }

        //     ascend_msgs::GRState robot;

        //     robot.header = observation.header;

        //     point_t position = it->getPosition();
        //     robot.x = position.x;
        //     robot.y = position.y;
        //     robot.theta = it->getOrientation();
        //     robot.visible = it->getVisible();
        //     observation.obstacle_robots[it->getIndex()] = robot;
        // }

        geometry_msgs::Point32 drone;
        drone.x = drone_position.x;
        drone.y = drone_position.y;
        drone.z = drone_position.z;
        observation.drone_position = drone;

        observation_pub.publish(observation);
        rate.sleep();
    }

    return 0;

}
