#include <stdio.h>
#include <iostream>
#include <cmath>
#include <vector>
#include <string>
#include <Eigen/Core>
#include <Eigen/Dense>

#include <ros/ros.h>

#include "include_file2.h"

#define s(x) std::sin(x)
#define c(x) std::cos(x)
#define PRINT_MAT(X) std::cout << #X << ":\n" << X << std::endl << std::endl
#define printVector3(x) for(int i = 0 ; i < 3; i++){std::cout<<x(i)<<" ";} std::cout<<"\n";

double abs_gravity_z;

// the euler angles and angular velocities are defined wrt body frame always
// the position, velocity, and acceleration are defined wrt world frame;
Mav mav;
State currState;
State desState;
State prevState;

/////////////////////////////////////////////////////////////////////

void OdometryCallback(const nav_msgs::Odometry::ConstPtr& msg){

	Eigen::Vector3d posB;
	Eigen::Vector3d eulB;
	Eigen::Vector3d velB;
	Eigen::Vector3d omgB;
	Eigen::Quaternion<double> quatB;

	posB = mav_msgs::vector3FromPointMsg(msg->pose.pose.position);
	quatB = mav_msgs::quaternionFromMsg(msg->pose.pose.orientation);
	velB = mav_msgs::vector3FromMsg(msg->twist.twist.linear);
	omgB = mav_msgs::vector3FromMsg(msg->twist.twist.angular);
	
	mav_msgs::getEulerAnglesFromQuaternion(quatB, &eulB);

	currState.position = posB;
	currState.velocity = wRb(eulB(0), eulB(1), eulB(2))*velB;//wRb converts from body to world
	currState.euler_angle = eulB;
	currState.angular_velocity = omgB;

}

void TrajectoryCallback(const trajectory_msgs::MultiDOFJointTrajectory::ConstPtr& msg){

	Eigen::Vector3d des_posW, des_velW, des_accW, des_jerkW, des_snapW, des_eulW, des_omgW;
	Eigen::Quaternion<double> des_quatW;

	mav_msgs::EigenTrajectoryPoint traj_reference;
	mav_msgs::eigenTrajectoryPointFromMsg(msg->points.front(), &traj_reference);

	des_posW = traj_reference.position_W;
	des_velW = traj_reference.velocity_W;
	des_accW = traj_reference.acceleration_W;
	des_jerkW = traj_reference.jerk_W;
	des_snapW = traj_reference.snap_W;

	des_quatW = traj_reference.orientation_W_B;
	mav_msgs::getEulerAnglesFromQuaternion(des_quatW, &des_eulW);
	des_omgW = traj_reference.angular_velocity_W;

	desState.position = des_posW;
	desState.velocity = des_velW;
	desState.acceleration = des_accW;
	desState.euler_angle = des_eulW;
	desState.angular_velocity = des_omgW;
}

//////////////////////////////////////////////////////////////////////////////////////////

void InitialiseRotorMatrix(){

	Rotors* r = mav.parameters.rotors;

	Eigen::MatrixXd inp_r( mav.parameters.no_rotors, 4);
	Eigen::MatrixXd r_inp( 4, mav.parameters.no_rotors);

	for(int i = 0; i < mav.parameters.no_rotors ; i++){

		r_inp(0,i) = 1;
		r_inp(1,i) = (r[i].arm_length)*(std::sin(r[i].rotor_angle));
		r_inp(2,i) = -1*(r[i].arm_length)*(std::cos(r[i].rotor_angle));
		r_inp(3,i) =  -1*(r[i].direction)*(r[i].moment_constant);
	}

	inp_r = (( r_inp.transpose() )*(( r_inp*(r_inp.transpose()) ).inverse()) )/(r[0].force_constant);
	//A left-inverse of a matrix r

	mav.inp_to_rotor_matrix = inp_r;

}

Eigen::MatrixXd InputToRotorSpeed( Eigen::Vector4d U){
	//U = U_thrust, Uroll, Upitch, Uyaw
	Eigen::MatrixXd rotor_speed2( mav.parameters.no_rotors, 1);
	Eigen::MatrixXd rotor_speed( mav.parameters.no_rotors, 1);

	rotor_speed2 = mav.inp_to_rotor_matrix*U;

	rotor_speed = rotor_speed2.array().sqrt();

	return rotor_speed;

}

///////////////////////////////////////////////////////////////////////////////////////
////////////////////////////HERE STARTS A PID CONTROLLER///////////////////////////////////




///////////////////////////////////////////////////////////////////////////////////////

int main(int argc, char **argv){

	
	//ros::init(argc, argv, "Controller_NAME");
	ros::init(argc, argv, "lqr_Controller");

	ros::NodeHandle n;


	initialise_params(n, mav, abs_gravity_z);


	//ADD CONTROLER PARAMETERS here
	//..........//


	abs_gravity_z = abs_gravity_z < 0 ? -1*abs_gravity_z : abs_gravity_z;


	InitialiseRotorMatrix();
	prevState.position(2) = 1.0;


	ros::Subscriber subOdom = n.subscribe("/" + mav.parameters.name +"/odometry_sensor1/odometry", 10, OdometryCallback);
	ros::Subscriber subTraj = n.subscribe("/" + mav.parameters.name +"/command/trajectory", 10, TrajectoryCallback);
	ros::Publisher pubActuators = n.advertise<mav_msgs::Actuators>("/" + mav.parameters.name +"/command/motor_speed", 1000);
	
	
	mav_msgs::Actuators msg;

	ros::Rate loop_rate(100);

	Eigen::MatrixXd rs( mav.parameters.no_rotors, 1);


	while(ros::ok()){

		ros::spinOnce();


		if(subTraj.getNumPublishers() == 0){
			desState = prevState;			
		}
		else{
			prevState = desState;
		}


		Eigen::Vector4d input_U;

		//input_U = CONTROLLER();


		rs = InputToRotorSpeed( input_U );


		for(int i = 0; i < mav.parameters.no_rotors; i++){
			if(rs(i,0) > 800){
				rs(i,0) = 800;
			}
			else if( std::isnan( rs(i,0) ) ){
				rs(i,0) = 0;
			}

		}

		std::vector<double> rotor_omega;
		for(int i = 0; i < mav.parameters.no_rotors; i++){
			rotor_omega.push_back(rs(i,0));
		}

		msg.angular_velocities = rotor_omega;


		pubActuators.publish(msg);

		loop_rate.sleep();
	}

	return 0;
}

