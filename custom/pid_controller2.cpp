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

	mav.inp_to_rotor_matrix = inp_r;

}

Eigen::MatrixXd InputToRotorSpeed( Eigen::Vector4d U){

	Eigen::MatrixXd rotor_speed2( mav.parameters.no_rotors, 1);
	Eigen::MatrixXd rotor_speed( mav.parameters.no_rotors, 1);

	rotor_speed2 = mav.inp_to_rotor_matrix*U;

	rotor_speed = rotor_speed2.array().sqrt();

	return rotor_speed;

}

///////////////////////////////////////////////////////////////////////////////////////
////////////////////////////HERE STARTS A PID CONTROLLER///////////////////////////////////

//integral for pid
Eigen::Vector3d int_position( 0.0, 0.0, 0.0);
Eigen::Vector3d int_angle( 0.0, 0.0, 0.0);

Eigen::Vector3d Kp_pos( 0, 0, 0);
Eigen::Vector3d Kd_pos( 0, 0, 0);
Eigen::Vector3d Ki_pos( 0, 0, 0);

Eigen::Vector3d Kp_ang( 0, 0, 0);
Eigen::Vector3d Kd_ang( 0, 0, 0);
Eigen::Vector3d Ki_ang( 0, 0, 0);

Eigen::Vector4d PID( State des_state, State curr_state){
	/* Here position and yaw are wrt world from but angles and omega are according to body frame*/


	Eigen::Vector3d err_position = des_state.position - curr_state.position;
	Eigen::Vector3d err_velocity = des_state.velocity - curr_state.velocity;
	Eigen::Vector3d des_acc 	 = des_state.acceleration;

	Eigen::Vector3d r_dd = err_position.array()*Kp_pos.array() + err_velocity.array()*Kd_pos.array() + int_position.array()*Ki_pos.array();
	r_dd = r_dd + des_acc;

	des_state.euler_angle(0) = ( r_dd(0) * s(curr_state.euler_angle(2)) - r_dd(1) * c(curr_state.euler_angle(2)) )/abs_gravity_z;
	des_state.euler_angle(1) = ( r_dd(0) * c(curr_state.euler_angle(2)) + r_dd(1) * s(curr_state.euler_angle(2)) )/abs_gravity_z;

	//des_state.euler_angle(0) = des_state.euler_angle(0) >= 0 ? std::min(des_state.euler_angle(0), 0.4363) : std::max(des_state.euler_angle(0), -0.4363);
	//des_state.euler_angle(1) = des_state.euler_angle(1) >= 0 ? std::min(des_state.euler_angle(1), 0.4363) : std::max(des_state.euler_angle(1), -0.4363);


	Eigen::Vector3d err_angle = des_state.euler_angle - curr_state.euler_angle;
	err_angle(2) = mav_msgs::getShortestYawDistance(des_state.euler_angle(2), curr_state.euler_angle(2));
	
	Eigen::Vector3d err_angular_velocity = des_state.angular_velocity - curr_state.angular_velocity;



	Eigen::Vector3d ang_dd = err_angle.array()*Kp_ang.array() + err_angular_velocity.array()*Kd_ang.array() + int_angle.array()*Ki_ang.array();


	int_position += err_position*0.01;
	int_angle += err_angle*0.01;


	double u1 = mav.parameters.mass*( abs_gravity_z + r_dd(2) );


	Eigen::Matrix3d I;
	I<< mav.parameters.inertia.xx,		0,		0,
		0,		mav.parameters.inertia.yy,		0,
		0,		0,		mav.parameters.inertia.zz;


	Eigen::Vector3d u2 = I*ang_dd;


	//Eigen::Vector4d U( u1, u2(0), u2(1), u2(2) );


	Eigen::Vector4d U_temp(u1, ang_dd(0), ang_dd(1), ang_dd(2));


	return U_temp;

}
///////////////////////////////////////////////////////////////////////////////////////

int main(int argc, char **argv){

	//ros::init(argc, argv, "Controller_NAME");
	ros::init(argc, argv, "pid_controller2");


	ros::NodeHandle n;


	initialise_params(n, mav, abs_gravity_z);
	abs_gravity_z = abs_gravity_z < 0 ? -1*abs_gravity_z : abs_gravity_z;
	InitialiseRotorMatrix();
	prevState.position(2) = 1.0;

	//INITIALIZE PID GAINS
	std::string key[2] = {"position", "attitude"};
	std::string gain[3] = {"kp", "kd", "ki"};
	std::string axis[3] = {"x", "y", "z"};

	for(int i = 0; i < 2; i++){

		for(int j = 0; j < 3; j++){

			for(int k = 0; k < 3; k++){

				double temp;
				n.getParam("/" + mav.parameters.name + "/" +key[i] +"_" + gain[j] + "/" + axis[k], temp );
				if(i == 0){

					if( j == 0 ){
						Kp_pos(k) = temp;
					}
					else if( j == 1 ){
						Kd_pos(k) = temp;
					}
					else if( j == 2 ){
						Ki_pos(k) = temp;
					}
				}
				else if(i == 1){

					if( j == 0 ){
						Kp_ang(k) = temp;
					}
					else if( j == 1 ){
						Kd_ang(k) = temp;
					}
					else if( j == 2 ){
						Ki_ang(k) = temp;
					}
				}
			
			}
		}
	}


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
		input_U = PID(desState, currState);


		rs = InputToRotorSpeed( input_U );


		for(int i = 0; i < mav.parameters.no_rotors; i++){
			if(rs(i,0) > 838){
				rs(i,0) = 838;
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

