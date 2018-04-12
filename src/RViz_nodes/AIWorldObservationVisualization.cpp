#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <ascend_msgs/AIWorldObservation.h>

#include <vector>
#include <cmath>

typedef struct struct_detected_robot
{
	float x,y;
	float angle;
	int color;
}DetectedRobot;

typedef struct struct_rotation_quaternion
{
	float x,y,z,w;
}RotationQuaternion;

std::vector<DetectedRobot> ground_robots_g;
std::vector<DetectedRobot> fused_robots_g;
std::vector<DetectedRobot> ground_obstacles_g;
std::vector<DetectedRobot> fused_obstacles_g;

RotationQuaternion eulerAnglesToQuaternion(double pitch, double roll, double yaw){
	RotationQuaternion q;
        // Abbreviations for the various angular functions
	double cy = cos(yaw * 0.5);
	double sy = sin(yaw * 0.5);
	double cr = cos(roll * 0.5);
	double sr = sin(roll * 0.5);
	double cp = cos(pitch * 0.5);
	double sp = sin(pitch * 0.5);

	q.w = cy * cr * cp + sy * sr * sp;
	q.x = cy * sr * cp - sy * cr * sp;
	q.y = cy * cr * sp + sy * sr * cp;
	q.z = sy * cr * cp - cy * sr * sp;
	return q;
}

void ObservationCallback(const ascend_msgs::AIWorldObservation::ConstPtr observation){


	for(int i=0; i<10; i++){
		float x_pos = observation->ground_robots.at(i).x;
		float y_pos = observation->ground_robots.at(i).y;
		float angle = observation->ground_robots.at(i).theta;
		int color = 1;
		DetectedRobot robot = {x_pos, y_pos, angle, color};
		ground_robots_g.push_back(robot);

	}

	for(int i=0; i<4; i++){
		float x_pos = observation->obstacle_robots.at(i).x;
		float y_pos = observation->obstacle_robots.at(i).y;
		float angle = observation->obstacle_robots.at(i).theta;
		int color = 3;
		DetectedRobot robot = {x_pos, y_pos, angle, color};
		ground_obstacles_g.push_back(robot);
	}
}

void FusedCallback(const ascend_msgs::AIWorldObservation::ConstPtr observation){
	for(int i=0; i<10; i++){
		if(observation->ground_robots.at(i).visible){
			float x_pos = observation->ground_robots.at(i).x;
			float y_pos = observation->ground_robots.at(i).y;
			float angle = observation->ground_robots.at(i).theta;
			int color = 2;
			DetectedRobot robot = {x_pos, y_pos, angle, color};
			fused_robots_g.push_back(robot);
		}
	}
	for(int i=0; i<4; i++){
		float x_pos = observation->obstacle_robots.at(i).x;
		float y_pos = observation->obstacle_robots.at(i).y;
		float angle = observation->obstacle_robots.at(i).theta;
		int color = 4;
		DetectedRobot robot = {x_pos, y_pos, angle, color};
		fused_obstacles_g.push_back(robot);
	}
}

void createRobotMarker(DetectedRobot& robot, visualization_msgs::Marker &robot_marker, visualization_msgs::Marker &direction_marker, int id){
	robot_marker.pose.position.x = robot.x;
	robot_marker.pose.position.y = robot.y;
	robot_marker.pose.position.z = 0.0;
	robot_marker.id = id;
				
	//Find position and orientation of the marker
	RotationQuaternion direction_quaternion = eulerAnglesToQuaternion(0.0, 0.0, robot.angle);
	direction_marker.pose.orientation.x = direction_quaternion.x;
	direction_marker.pose.orientation.y = direction_quaternion.y;
	direction_marker.pose.orientation.z = direction_quaternion.z;
	direction_marker.pose.orientation.w = direction_quaternion.w;
	direction_marker.pose.position.x = robot.x + (0.1 * cos(robot.angle)); //Half the size of the circle
	direction_marker.pose.position.y = robot.y + (0.1 * sin(robot.angle)); //Half the size of the circle
	direction_marker.pose.position.z = 0.025; //It should be placed higher than the circle.
	direction_marker.id = id+1;
}

void createRobotMarker(DetectedRobot& robot, visualization_msgs::Marker &robot_marker, int id){
	robot_marker.pose.position.x = robot.x;
	robot_marker.pose.position.y = robot.y;
	robot_marker.pose.position.z = 0.0;
	robot_marker.id = id;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "AIWorldObservationVisualizer");
	ros::NodeHandle n;
	ros::Publisher sim_pub = n.advertise<visualization_msgs::MarkerArray>("/ai/SimVisualizer", 10);
	ros::Publisher fuser_pub = n.advertise<visualization_msgs::MarkerArray>("/ai/FuserVisualizer", 10);

	ros::Subscriber fuser_sub = n.subscribe("AIWorldObservation", 1, FusedCallback);
	ros::Subscriber sim_sub = n.subscribe("/ai/sim", 1, ObservationCallback);


	int ros_rate = 10;
	ros::Rate rate(ros_rate);


	visualization_msgs::Marker robot_marker;
	visualization_msgs::Marker direction_marker;

		
	//Setup params for the robot marker
	// Set the frame ID and timestamp.  See the TF tutorials for information on these.
    robot_marker.header.frame_id = "/map";
    

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    robot_marker.ns = "basic_shapes";

    //COMMENT: double ;;
    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    robot_marker.type = visualization_msgs::Marker::CYLINDER;

    // Set the marker action.  Options are ADD and DELETE
    robot_marker.action = visualization_msgs::Marker::ADD;

    //COMMENT: Future suggestion: Set these values in a configuration file.
    robot_marker.scale.x = 0.2;
    robot_marker.scale.y = 0.2;
    robot_marker.scale.z = 0.05;
    robot_marker.color.a = 1.0;
    robot_marker.lifetime = ros::Duration(ros_rate);
	
	//Now setup the params for the direction marker the same way as the robot marker
	direction_marker.header.frame_id = "/map";
	
	direction_marker.ns = "basic_shapes";
	direction_marker.type = visualization_msgs::Marker::CYLINDER;
	direction_marker.action = visualization_msgs::Marker::ADD;
	direction_marker.scale.x = 0.1;
	direction_marker.scale.y = 0.01;
	direction_marker.scale.z = 0.05;
	direction_marker.color.a = 1.0;
	direction_marker.color.r = 1.0;
	direction_marker.color.g = 1.0;
	direction_marker.color.b = 0.0;
	direction_marker.lifetime = ros::Duration(ros_rate);

	while(ros::ok())
	{
		robot_marker.header.stamp = ros::Time::now();
		direction_marker.header.stamp = ros::Time::now();

	    visualization_msgs::MarkerArray marker_array_fuser;
		
		for(int i=0; i<fused_robots_g.size(); i++){
			//First reset the previously added markers
		    robot_marker.color.r = 1.0;
		    robot_marker.color.g = 0.0;
		    robot_marker.color.b = 0.0;

			DetectedRobot& robot = fused_robots_g.at(i);
			createRobotMarker(robot, robot_marker, direction_marker, 2*i);

			marker_array_fuser.markers.push_back(robot_marker);
			marker_array_fuser.markers.push_back(direction_marker);
		}

		for(int i=0; i<fused_obstacles_g.size(); i++){
			//First reset the previously added markers
			robot_marker.color.r = 0.5;
			robot_marker.color.g = 0.5;
			robot_marker.color.b = 1.0;

			DetectedRobot& robot = fused_obstacles_g.at(i);
			createRobotMarker(robot, robot_marker, 2*fused_robots_g.size()+i);

			marker_array_fuser.markers.push_back(robot_marker);		
		}

		fuser_pub.publish(marker_array_fuser);

		visualization_msgs::MarkerArray marker_array;

		for(int i=0; i<ground_robots_g.size(); i++)
		{
			//First reset the previously added markers
			robot_marker.color.r = 0.0;
			robot_marker.color.g = 1.0;
			robot_marker.color.b = 0.0;

			DetectedRobot& robot = ground_robots_g.at(i);

			createRobotMarker(robot, robot_marker, direction_marker, 2*i);
			marker_array.markers.push_back(robot_marker);
			marker_array.markers.push_back(direction_marker);
		}

		for(int i=0; i<ground_obstacles_g.size(); i++){
			//First reset the previously added markers
		    robot_marker.color.r = 1.0;
		    robot_marker.color.g = 1.0;
		    robot_marker.color.b = 1.0;

			DetectedRobot& robot = ground_obstacles_g.at(i);
			createRobotMarker(robot, robot_marker, 2*ground_robots_g.size()+i);

			marker_array.markers.push_back(robot_marker);		
		}
		
		sim_pub.publish(marker_array);
		
		
		
		fused_robots_g.clear();
		fused_obstacles_g.clear();
		ground_robots_g.clear();
		ground_obstacles_g.clear();
	
		ros::spinOnce();
        rate.sleep();
	}

	//With a fixed rate, publish what the drone "sees"
}