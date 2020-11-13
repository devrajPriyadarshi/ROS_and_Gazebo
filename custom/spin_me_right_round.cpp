#include <stdio.h>
#include <iostream>
#include <cmath>

#include <Eigen/Core>
#include <mav_msgs/default_topics.h>
#include <mav_msgs/common.h>
#include <mav_msgs/conversions.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

#define s(x) std::sin(x)
#define c(x) std::cos(x)

int main(int argc, char** argv) {

	ros::init(argc, argv, "spin_me_right_round");
	ros::NodeHandle nh("");

	std::string mav;
	nh.getParam("/mavName", mav);

	ros::Publisher trajectory_pub = nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>("/" + mav + "/command/trajectory", 10);

	double x,y,z,des_yaw;

	double R = 2;
	double omega = 0.9;

	ros::Rate loop_rate(100);

	while (trajectory_pub.getNumSubscribers() == 0 && ros::ok()) {
		ROS_INFO("There is no subscriber available, trying again in 1 second.");
		ros::Duration(1.0).sleep();
	}

	double To = ros::Time::now().toSec();

	///////////THIS NEVER WORKED
	while(ros::ok()){

		double t = ros::Time::now().toSec() - To;


		x = R * s( omega * t );
		y = R * c( omega * t ) - R;
		z = 1;
		des_yaw = mav_msgs::getShortestYawDistance( ( -1 * omega * t ), 0);
	
		Eigen::Vector3d pos(x, y, z);
		Eigen::Vector3d vel(omega * R * c( omega * t ), -omega * R * s( omega * t ), 0);
		Eigen::Vector3d acc(-omega*omega*R * s( omega * t ), -omega*omega*R * c( omega * t ), 0);

		trajectory_msgs::MultiDOFJointTrajectory trajectory_msg;
		
		mav_msgs::EigenTrajectoryPoint point;
		
		point.position_W =  pos;
		point.velocity_W = vel;
		point.acceleration_W = acc;
		point.setFromYaw(des_yaw);
		point.setFromYawRate(-omega);

		mav_msgs::msgMultiDofJointTrajectoryFromEigen( point, &trajectory_msg );

		trajectory_pub.publish(trajectory_msg);
		loop_rate.sleep();
	}
	return 0;

 }