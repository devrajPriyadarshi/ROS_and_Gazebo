#include <stdio.h>
#include <iostream>

#include <Eigen/Core>
#include <mav_msgs/default_topics.h>
#include <mav_msgs/conversions.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>


int main(int argc, char** argv) {

	ros::init(argc, argv, "waypoint");
	ros::NodeHandle nh("");

	std::string mavName;
	nh.getParam("/mavName", mavName);
	ros::Publisher trajectory_pub = nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>("/" + mavName + +"/command/trajectory", 10);

	double x,y,z,des_yaw;

	printf("\nEnter Desired State :\n");
	printf("X :");
	scanf("%lf", &x);
	printf("Y :");
	scanf("%lf", &y);
	printf("Z :");
	scanf("%lf", &z);
	printf("YAW(degrees b/w -pi to pi) :");
	scanf("%lf", &des_yaw);

	des_yaw = (des_yaw*3.14159265)/180.0000;

	ros::Rate loop_rate(100);

	trajectory_msgs::MultiDOFJointTrajectory trajectory_msg;
	
	Eigen::Vector3d desired_position(x, y, z);

	mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(desired_position, des_yaw, &trajectory_msg);

	while (trajectory_pub.getNumSubscribers() == 0 && ros::ok()) {
		ROS_INFO("There is no subscriber available, trying again in 1 second.");
		ros::Duration(1.0).sleep();
	}

	ROS_INFO("Publishing waypoint on namespace : [%f, %f, %f].",
           x,
           y,
           z);

	while(ros::ok()){
		trajectory_pub.publish(trajectory_msg);
		loop_rate.sleep();
	}
	return 0;

 }