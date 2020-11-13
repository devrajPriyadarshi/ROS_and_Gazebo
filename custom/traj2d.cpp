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

	ros::init(argc, argv, "traj2d");
	ros::NodeHandle nh("");

	std::string mav;
	nh.getParam("/mavName", mav);

	ros::Publisher trajectory_pub = nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>("/" + mav + "/command/trajectory", 10);

	double x,y,z,des_yaw;

	ros::Rate loop_rate(100);

	while (trajectory_pub.getNumSubscribers() == 0 && ros::ok()) {
		ROS_INFO("There is no subscriber available, trying again in 1 second.");
		ros::Duration(1.0).sleep();
	}
	
	double T = 3; //secs

	
	double a_max = 2;
	double v_max = 2;
	Eigen::Vector3d initial_pos( 0, 0, 1);

	double dt;
	Eigen::Vector3d acc;
	Eigen::Vector3d vel;
	Eigen::Vector3d pos;

	double To = ros::Time::now().toSec();

	while(ros::ok()){

		double t = ros::Time::now().toSec() - To;
		

		if( t <= v_max/a_max){
			dt = t;
			acc = Eigen::Vector3d( a_max, 0, 0);
			vel = acc*dt;
			pos = 0.5 * acc * (dt*dt);
		}
		else if( t <= 2*v_max/a_max){
			dt = t - v_max/a_max;
			acc = Eigen::Vector3d( 0, 0, 0);
			vel = Eigen::Vector3d( v_max, 0, 0);
			pos = Eigen::Vector3d(0.5*v_max*v_max/a_max, 0, 0) + Eigen::Vector3d( v_max*dt, 0, 0); 
		}
		else if(t <= 3*v_max/a_max){
			dt = t - 2*v_max/a_max;
			acc = Eigen::Vector3d( -a_max, 0, 0);
			vel = Eigen::Vector3d( v_max, 0, 0) + acc*dt;
			pos = Eigen::Vector3d(1.5*v_max*v_max/a_max, 0, 0) + Eigen::Vector3d(v_max *dt, 0, 0 ) + 0.5*acc*(dt*dt);
		}
		else{
			acc = Eigen::Vector3d( 0, 0, 0);
			vel = Eigen::Vector3d( 0, 0, 0);
			pos = Eigen::Vector3d( 2*v_max*v_max/a_max, 0, 0);
		}
				

		des_yaw = 0;

		trajectory_msgs::MultiDOFJointTrajectory trajectory_msg;
		
		mav_msgs::EigenTrajectoryPoint point;
		
		point.position_W = initial_pos + pos;
		point.velocity_W = vel;
		point.acceleration_W = acc;
		point.setFromYaw(des_yaw);

		mav_msgs::msgMultiDofJointTrajectoryFromEigen( point, &trajectory_msg );

		trajectory_pub.publish(trajectory_msg);
		loop_rate.sleep();
	}
	return 0;

 }
