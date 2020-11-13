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
#define p(x,y) std::pow(x,y)
#define pi 	 3.14159

int main(int argc, char** argv) {

	ros::init(argc, argv, "traj_helix");
	ros::NodeHandle nh("");

	std::string mav;
	nh.getParam("/mavName", mav);

	ros::Publisher trajectory_pub = nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>("/" + mav + "/command/trajectory", 10);

	ros::Rate loop_rate(100);

	while (trajectory_pub.getNumSubscribers() == 0 && ros::ok()) {
		ROS_INFO("There is no subscriber available, trying again in 1 second.");
		ros::Duration(1.0).sleep();
	}
	

	double T = 8 ; //finishing time 10

	Eigen::Vector3d initial_pos( 0, 0, 1);

	Eigen::Vector3d acc;
	Eigen::Vector3d vel;
	Eigen::Vector3d pos;
	double x, y, z, xd, yd, zd, xdd, ydd, zdd;
	double des_yaw;
	double des_yaw_rate;

	double p,v,a;

	
	double To = ros::Time::now().toSec();
	double z_max = 2.5;
	double r = 2;


	while(ros::ok()){

		double t = ros::Time::now().toSec() - To;
		

		if(t >= T){

			x = c(2*pi)*r;
			y = s(2*pi)*r;
			z = z_max;
			pos = Eigen::Vector3d(x,y,z);
			vel = Eigen::Vector3d(0,0,0);
			acc = Eigen::Vector3d(0,0,0);
		
		}
		else{

			double t0 = 0;
			double tf = T;

			Eigen::MatrixXd M( 6, 6);
			M <<	1		,	p(t0,1),	p(t0,2),	p(t0,3),	p(t0,4),	p(t0,5),
					0,			1,		  2*p(t0,1),  3*p(t0,2),  4*p(t0,3),  5*p(t0,4),
					0,			0,			2,		  6*p(t0,1), 12*p(t0,2), 20*p(t0,3),
					1		,	p(tf,1),	p(tf,2),	p(tf,3),	p(tf,4),	p(tf,5),
					0,			1,		  2*p(tf,1),  3*p(tf,2),  4*p(tf,3),  5*p(tf,4),
					0,			0,			2,		  6*p(tf,1), 12*p(tf,2), 20*p(tf,3);

			Eigen::MatrixXd b(6,2);
			b<<		0,		0,
					0,		0,
					0,		0,
					2*pi, 	z_max,
					0,		0,
					0,		0;

			Eigen::MatrixXd a(6,2);
			a = (M.inverse())*b; // implies left division of M by b

			Eigen::MatrixXd out(1,2);
			Eigen::MatrixXd outd(1,2);
			Eigen::MatrixXd outdd(1,2);

			for(int i = 0; i < 2; i ++){
				out(0,i) = 0.0;
				outd(0,i) = 0.0;
				outdd(0,i) = 0.0;
			}

			for(int i = 0; i < 6; i++){
				out(0,0) += a(i,0)*p(t,i);
				out(0,1) += a(i,1)*p(t,i);
			}
			for(int i = 1; i < 6; i++){
				outd(0,0) += i*a(i,0)*p(t,i-1);
				outd(0,1) += i*a(i,1)*p(t,i-1);
			}
			for(int i = 2; i < 6; i++){
				outdd(0,0) += (i-1)*i*a(i,0)*p(t,i-2);
				outdd(0,1) += (i-1)*i*a(i,1)*p(t,i-2);
			}


			double beta = out(0,0);
			double betad = outd(0,0);
			double betadd = outdd(0,0);

			z = out(0,1);
			zd = outd(0,1);
			zdd = outdd(0,1);

			//position
			x = c(beta)*r;
			y = s(beta)*r;
			pos = Eigen::Vector3d(x,y,z);
			//velocity
			xd = -y*betad;
			yd =  x*betad;
			vel = Eigen::Vector3d(xd,yd,zd);
			//acceletation
			xdd = -x*betad*betad - y*betadd;
			ydd = -y*betad*betad + x*betadd;
			acc = Eigen::Vector3d(xdd,ydd,zdd);

		}


		des_yaw = 0;
		des_yaw_rate = 0;


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

