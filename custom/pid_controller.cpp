#include <stdio.h>
#include <iostream>
#include <cmath>
#include <vector>
#include <string>
#include <Eigen/Core>
#include <Eigen/Dense>

#include <ros/ros.h>

#include "include_files.h"

#define s(x) std::sin(x)
#define c(x) std::cos(x)

double abs_gravity_z;
Eigen::MatrixXd inv_input_to_rotor_matrix( 6, 4);

Mav mav;
Mav des_state;
Mav previous_des_state;

Velocity 		zeroVel;
AngularVelocity zeroOmg;

void Initialise_(){
	
	zeroVel.z = 0;
	zeroVel.y = 0;
	zeroVel.x = 0;

	zeroOmg.roll = 0;
	zeroOmg.pitch = 0;
	zeroOmg.yaw = 0;

	previous_des_state.pos.x = 0;
	previous_des_state.pos.y = 0;
	previous_des_state.pos.z = 1;

	previous_des_state.eul.roll = 0;
	previous_des_state.eul.pitch = 0;
	previous_des_state.eul.yaw = 0;

}

/////////////////////////////////////////////////////////////////////
/*void PoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
	Position pos;
	Quaternion quar;
	EulerAngles eul;

	pos.x = msg.pose.position.x;
	pos.y = msg.pose.position.y;
	pos.z = msg.pose.position.z;

	quar.x = msg.pose.orientation.x;
	quar.y = msg.pose.orientation.y;
	quar.z = msg.pose.orientation.z;
	quar.w = msg.pose.orientation.w;

	eul = ToEulerAngles(quar);
}*/

void OdometryCallback(const nav_msgs::Odometry::ConstPtr& msg){
	Position _pos;
	Quaternion _quar;
	EulerAngles _eul;

	Eigen::Vector3d posw;

	posw = mav_msgs::vector3FromPointMsg(msg->pose.pose.position);

	_pos.x = posw(0);
	_pos.y = posw(1);
	_pos.z = posw(2);
	
	Eigen::Quaternion<double> quatw = mav_msgs::quaternionFromMsg(msg->pose.pose.orientation);

	_quar.x = quatw.x();
	_quar.y = quatw.y();
	_quar.z = quatw.z();
	_quar.w = quatw.w();

	_eul = ToEulerAngles(_quar);

	Velocity _vel;
	AngularVelocity _omg;

	_vel.x = msg->twist.twist.linear.x;
	_vel.y = msg->twist.twist.linear.y;
	_vel.z = msg->twist.twist.linear.z;

	_omg.roll = msg->twist.twist.angular.x;
	_omg.pitch = msg->twist.twist.angular.y;
	_omg.yaw = msg->twist.twist.angular.z;

	mav.pos = _pos;
	mav.eul = _eul;
	mav.vel = _vel;
	mav.omg = _omg;
}

void TrajectoryCallback(const trajectory_msgs::MultiDOFJointTrajectory::ConstPtr& msg){
	Position _pos;
	Quaternion _quar;
	EulerAngles _eul;
	Velocity _vel;
	AngularVelocity _omg;

	mav_msgs::EigenTrajectoryPoint traj_reference;
	mav_msgs::eigenTrajectoryPointFromMsg(msg->points.front(), &traj_reference);

	_pos.x = traj_reference.position_W(0);
	_pos.y = traj_reference.position_W(1);
	_pos.z = traj_reference.position_W(2);

	_eul.roll = 0;
	_eul.pitch = 0;
	_eul.yaw = traj_reference.getYaw(); 

	des_state.pos = _pos;
	des_state.eul = _eul;
}

/*void ImuCallback(const sensor_msgs::Imu::ConstPtr& msg){
	Acceleration acc;

	acc.x = msg->linear_acceleration.x;
	acc.y = msg->linear_acceleration.y;
	acc.z = msg->linear_acceleration.z;// imu give F-non-Gravitational acceleration

	mav.acc = acc;
}*/

//////////////////////////////////////////////////////////////////////////////////////////
//HERE WE GIVE THRUST AND MOMENT, AND GET BACK ROTOR SPEEDS
void InitialiseMatrices(){

	Rotors* r = mav.parameters.rotors;

	Eigen::MatrixXd inp_r( 6, 4);
	Eigen::MatrixXd r_inp( 4, 6);

	for(int i = 0; i < 6 ; i++){

		r_inp(0,i) = 1;
		r_inp(1,i) = (r[i].arm_length)*(std::sin(r[i].rotor_angle));
		r_inp(2,i) = -1*(r[i].arm_length)*(std::cos(r[i].rotor_angle));
		r_inp(3,i) =  -1*(r[i].direction)*(r[i].moment_constant);
	}

	//inp_r = (( (    (r_inp.transpose())*(r_inp)    ).inverse())*(r_inp.transpose()))/(r[0].force_constant);
	inp_r = (   ( r_inp.transpose() )*(( r_inp*(r_inp.transpose()) ).inverse())    )/(r[0].force_constant);

	inv_input_to_rotor_matrix = inp_r;

}


Eigen::MatrixXd ThrustToSpeed(double u1, double u2, double u3, double u4){

	Eigen::MatrixXd U( 4, 1);
	U <<u1,
		u2,
		u3,
		u4;

	Eigen::MatrixXd rotor_speed2( 6, 1);
	Eigen::MatrixXd rotor_speed( 6, 1);

	rotor_speed2 = inv_input_to_rotor_matrix*U;

	rotor_speed = rotor_speed2.array().sqrt();

	/*for(int i = 0; i < 4; i++){
		std::cout<<U(i,0)<<" ";
	}
	std::cout<<"\n";
*/
	return rotor_speed;


}



///////////////////////////////////////////////////////////////////////////////////////////
//HERE STARTS A PID CONTROLLER////

double Px = 6.45;
double Py = 6.45;
double Pz = 6.45;

double Dx = 4.8;
double Dy = 4.8;
double Dz = 5;

double Ix = 0.075;
double Iy = 0.075;
double Iz = 0.075;

double Proll = 3.65;
double Ppitch = 3.65;
double Pyaw = 0.12;

double Droll = 0.585;
double Dpitch = 0.585;
double Dyaw = 0.18;

double Iroll = 0.1;
double Ipitch = 0.1;
double Iyaw = 0.00;

double intX = 0.0;
double intY = 0.0;
double intZ = 0.0;
double intYaw = 0.0;
double intRoll = 0.0;
double intPitch = 0.0;


Eigen::MatrixXd PID(Position des_pos, double des_yaw, Velocity des_vel = zeroVel){
	//global declaration of firefly's everything
	//velocity desired == 0,0,0 until a trajectory generator is made

	Position curPos = mav.pos;
	Velocity curVel = mav.vel;
	EulerAngles curAng = mav.eul;
	AngularVelocity curOmg = mav.omg;

	double rx_dd, ry_dd, rz_dd;
	double T, Troll, Tpitch, Tyaw;


	//////////////////////////////////////////////////////////////
	//Converting to World frame(velocity is given in body frame)
	double cvy = curVel.y, cvx = curVel.x, cvz = curVel.z;
	Eigen::MatrixXd vel_b( 3, 1);
	Eigen::MatrixXd vel_w( 3, 1);

	vel_b << cvx, cvy, cvz;

	vel_w = oRv(curAng.roll, curAng.pitch, curAng.yaw)*vel_b ;

	curVel.x = vel_w(0,0);
	curVel.y = vel_w(1,0);
	curVel.z = vel_w(2,0);

//	curVel.x = cvx*std::cos(curAng.yaw) - cvy*std::sin(curAng.yaw);
//	curVel.y = cvx*std::sin(curAng.yaw) + cvy*std::cos(curAng.yaw);
	///////////////////////////////////////////////////////////////

	rz_dd = Pz*( des_pos.z - curPos.z ) + Dz*( des_vel.z - curVel.z) + Ix*(intZ);
	ry_dd = Py*( des_pos.y - curPos.y ) + Dy*( des_vel.y - curVel.y) + Iy*(intY);
	rx_dd = Px*( des_pos.x - curPos.x ) + Dx*( des_vel.x - curVel.x) + Iz*(intX);

	double des_roll  = ( rx_dd*std::sin(curAng.yaw) - ry_dd*std::cos(curAng.yaw) )/abs_gravity_z;
	double des_pitch = ( rx_dd*std::cos(curAng.yaw) + ry_dd*std::sin(curAng.yaw) )/abs_gravity_z;

	des_roll = des_roll >= 0 ? std::min(des_roll, 0.4363) : std::max(des_roll, -0.4363);
	des_pitch = des_pitch >= 0 ? std::min(des_pitch, 0.4363) : std::max(des_pitch, -0.4363);



	T 		 = 		mav.parameters.mass*( abs_gravity_z + rz_dd );
	Troll 	 = 		Proll*( des_roll - curAng.roll ) + Droll*( -1*curOmg.roll ) + Iroll*(intRoll);
	Tpitch	 = 		Ppitch*( des_pitch - curAng.pitch ) + Dpitch*( -1*curOmg.pitch ) + Ipitch*(intPitch);
	Tyaw 	 = 		Pyaw*( des_yaw - curAng.yaw ) + Dyaw*( 0.0 - curOmg.yaw ) + Iyaw*(intYaw);


	intZ += ( des_pos.z - curPos.z )*0.01;
	intY += ( des_pos.y - curPos.y )*0.01;
	intX += ( des_pos.x - curPos.x )*0.01;

	intRoll += (des_roll - curAng.roll)*0.01;
	intPitch += (des_pitch - curAng.pitch)*0.01;
	intYaw += (des_yaw - curAng.yaw)*0.01;

	Eigen::MatrixXd rotor_speeds( 6, 1);

	rotor_speeds = ThrustToSpeed(T, Troll, Tpitch, Tyaw);

	/*std::cout<<"\nDesired ROLL : "<<des_roll<<"\n";
	std::cout<<"\nDesired PITCH : "<<des_pitch<<"\n";
	std::cout<<"\nDesired Troll : "<<Troll<<"\n";
	std::cout<<"\nDesired Tpitch : "<<Tpitch<<"\n";
	std::cout<<"\nDesired YAW : "<<des_yaw<<"\n";
	std::cout<<"\nCurrent YAW : "<<curAng.yaw<<"\n";
	*/

	return rotor_speeds;


}

//////////////////////////////////////////////////////////////////////////////////////////

int main(int argc, char **argv){

	ros::init(argc, argv, "pid_controller");

	ros::NodeHandle n;

	////////////////////////////////////

	n.getParam("/firefly/mass", mav.parameters.mass);
	n.getParam("/firefly/inertia/xx", mav.parameters.inertia.xx);
	n.getParam("/firefly/inertia/yy", mav.parameters.inertia.yy);
	n.getParam("/firefly/inertia/zz", mav.parameters.inertia.zz);

	for(int i = 0; i < 6; i++){
		std::string prefix = "/firefly/rotor_configuration/";
		std::string no = std::to_string(i);

		std::string _angle = prefix + no + "/angle";
		std::string _arm_length = prefix + no + "/arm_length";
		std::string _direction = prefix + no + "/direction";
		std::string _fc = prefix + no + "/rotor_force_constant";
		std::string _mc = prefix + no + "/rotor_moment_constant";

		n.getParam(_angle, mav.parameters.rotors[i].rotor_angle);
		n.getParam(_arm_length, mav.parameters.rotors[i].arm_length);
		n.getParam(_direction, mav.parameters.rotors[i].direction);
		n.getParam(_fc, mav.parameters.rotors[i].force_constant);
		n.getParam(_mc, mav.parameters.rotors[i].moment_constant);

	}

	double gravity_z;

	n.getParam("/gazebo_gui/gravity_z", gravity_z);

	abs_gravity_z = gravity_z < 0 ? -1*gravity_z : gravity_z;

	InitialiseMatrices();
	Initialise_();

	/////////////////////////////////////////////////////////////////////////////////////////////////

	/*ros::ServiceClient unpause_physics = n.serviceClient<std_srvs::Empty>("/gazebo/unpause_physics");
	std_srvs::Empty srv;
	unpause_physics.call(srv);*/

	//////////////////////////////////////////////////////////////////////////////////////////////////

	ros::Subscriber subOdom = n.subscribe("/firefly/odometry_sensor1/odometry", 10, OdometryCallback);
	ros::Subscriber subTraj = n.subscribe("/firefly/command/trajectory", 10, TrajectoryCallback);
	//ros::Subscriber subImu = n.subscribe("/firefly/imu",10,ImuCallback);

	ros::Publisher pubActuators = n.advertise<mav_msgs::Actuators>("/firefly/command/motor_speed", 1000);
	
	mav_msgs::Actuators msg;

	ros::Rate loop_rate(100);

	Eigen::MatrixXd rs( 6, 1);

	rs << 0, 0, 0, 0, 0, 0;

	while(ros::ok()){

		ros::spinOnce();

		if(subTraj.getNumPublishers() == 0){
			des_state = previous_des_state;
		}
		else{
			previous_des_state = des_state;
		}

		rs = PID(des_state.pos, des_state.eul.yaw);

		for(int i = 0; i<6; i++){
			if(rs(i,0) > 800){
				//std::cout<<"rotor "<<i<<" is at limit\n";
				rs(i,0) = 800;
			}
			else if( std::isnan( rs(i,0) ) ){
				//std::cout<<"rotor "<<i<<"given < 0\n";
				rs(i,0) = 0;
			}
		}

		msg.angular_velocities = {rs(0,0), rs(1,0), rs(2,0), rs(3,0), rs(4,0), rs(5,0)};

		//printf("\nPOSITION : [ %lf, %lf, %lf]\n", mav.pos.x, mav.pos.y, mav.pos.z);
		//printf("\nROTATION : [ %lf, %lf, %lf]\n", mav.eul.roll, mav.eul.pitch, mav.eul.yaw);
		//printf("\nANGULAR VELOCITIES : [ %lf, %lf, %lf]\n", mav.omg.roll, mav.omg.pitch, mav.omg.yaw);
		//printf("\nActuator Speed : [ %.3lf, %.3lf, %.3lf, %.3lf, %.3lf, %.3lf]\n",rs(0,0), rs(1,0), rs(2,0), rs(3,0), rs(4,0), rs(5,0));
		//printf("ACCELERATION : [ %.3lf, %.3lf, %.3lf]\n\n", firefly.acc.x, firefly.acc.y, firefly.acc.z);

		pubActuators.publish(msg);

		loop_rate.sleep();
	}

	return 0;
}
