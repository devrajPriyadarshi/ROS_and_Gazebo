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

	ros::init(argc, argv, "traj3d");
	ros::NodeHandle nh("");

	std::string mav;
	nh.getParam("/mavName", mav);

	ros::Publisher trajectory_pub = nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>("/" + mav + "/command/trajectory", 10);

	ros::Rate loop_rate(100);

	while (trajectory_pub.getNumSubscribers() == 0 && ros::ok()) {
		ROS_INFO("There is no subscriber available, trying again in 1 second.");
		ros::Duration(1.0).sleep();
	}
	

	double t_max = 4;

	Eigen::Vector3d initial_pos( 0, 0, 1);

	Eigen::Vector3d acc;
	Eigen::Vector3d vel;
	Eigen::Vector3d pos;
	double des_yaw;
	double des_yaw_rate;

	double p,v,a;

	double To = ros::Time::now().toSec();

	while(ros::ok()){

		double t = ros::Time::now().toSec() - To;
		

		t = std::max( 0.0, std::min( t, t_max));

		t = t/t_max;
		

		p = 10*t*t*t - 15*t*t*t*t + 6*t*t*t*t*t;
		v = (30/t_max)*t*t - (60/t_max)*t*t*t + (30/t_max)*t*t*t*t;
		a = (60/(t_max*t_max))*t - (180/(t_max*t_max))*t*t + (120/(t_max*t_max))*t*t*t;
		

		pos = Eigen::Vector3d(p,p,p);
		vel = Eigen::Vector3d(v,v,v);
		acc = Eigen::Vector3d(a,a,a);
		des_yaw = p;
		des_yaw_rate = v;


		trajectory_msgs::MultiDOFJointTrajectory trajectory_msg;
		
		mav_msgs::EigenTrajectoryPoint point;
		
		point.position_W = initial_pos + pos;
		point.velocity_W = vel;
		point.acceleration_W = acc;
		point.setFromYaw(des_yaw);
		point.setFromYawRate(des_yaw_rate);

		mav_msgs::msgMultiDofJointTrajectoryFromEigen( point, &trajectory_msg );

		trajectory_pub.publish(trajectory_msg);
		loop_rate.sleep();
	}
	return 0;

 }

