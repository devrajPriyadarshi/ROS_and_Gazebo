#include <Eigen/Core>
#include <Eigen/Dense>

#include <ros/ros.h>
#include <mav_msgs/common.h>
#include <mav_msgs/conversions.h>
#include <mav_msgs/eigen_mav_msgs.h>
#include <mav_msgs/default_topics.h>
#include <mav_msgs/Actuators.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

#define s(x) std::sin(x)
#define c(x) std::cos(x)


struct Rotors{
	double rotor_angle;
	double arm_length;
	double force_constant;
	double moment_constant;
	double direction;
	//direction of 1 implies rotor rotates counter - clockwise
};

struct Inertia{
	double xx,yy,zz;
};

struct Parameters{

	std::string name;
	double mass;
	
	Inertia inertia;	

	Rotors rotors[6];

	int no_rotors;
	
};

struct Mav{

	Parameters parameters;

	Eigen::MatrixXd inp_to_rotor_matrix;

};

struct State
{
	Eigen::Vector3d position;
	Eigen::Vector3d velocity;
	Eigen::Vector3d acceleration;
	Eigen::Vector3d jerk;
	Eigen::Vector3d snap;

	Eigen::Vector3d euler_angle;
	Eigen::Vector3d angular_velocity;

	State(){//initialise the values to some defaults
		position = Eigen::Vector3d::Zero();
		velocity = Eigen::Vector3d::Zero();
		acceleration = Eigen::Vector3d::Zero();
		jerk = Eigen::Vector3d::Zero();
		snap = Eigen::Vector3d::Zero();

		euler_angle = Eigen::Vector3d::Zero();
		angular_velocity = Eigen::Vector3d::Zero();
	}
};

Eigen::MatrixXd wRb(double roll, double pitch, double yaw){

	Eigen::MatrixXd R( 3, 3);
	R << c(yaw)*c(pitch) - s(roll)*s(yaw)*sin(pitch) , -c(roll)*s(yaw) , c(yaw)*s(pitch) + c(pitch)*s(roll)*s(yaw),
		 s(yaw)*c(pitch) + s(roll)*c(yaw)*sin(pitch) ,  c(roll)*c(yaw) , s(yaw)*s(pitch) - c(pitch)*s(roll)*c(yaw),
		 			  -c(roll)*s(pitch)				 , 		s(roll)	   , 			c(roll)*c(pitch);

	return R;
}


void initialise_params(ros::NodeHandle& n, Mav& mav, double& gravity_z){

	n.getParam("/mavName", mav.parameters.name);
	n.getParam("/" + mav.parameters.name + "/mass", mav.parameters.mass);
	n.getParam("/" + mav.parameters.name + "/inertia/xx", mav.parameters.inertia.xx);
	n.getParam("/" + mav.parameters.name + "/inertia/yy", mav.parameters.inertia.yy);
	n.getParam("/" + mav.parameters.name + "/inertia/zz", mav.parameters.inertia.zz);

	for(int i = 0; i < 6; i++){
		std::string prefix = "/" + mav.parameters.name + "/rotor_configuration/";
		std::string no = std::to_string(i);

		std::string _angle = prefix + no + "/angle";
		std::string _arm_length = prefix + no + "/arm_length";
		std::string _direction = prefix + no + "/direction";
		std::string _fc = prefix + no + "/rotor_force_constant";
		std::string _mc = prefix + no + "/rotor_moment_constant";

		if(n.getParam(_angle, mav.parameters.rotors[i].rotor_angle)){
			mav.parameters.no_rotors = 6;
		}
		else{
			mav.parameters.no_rotors = 4;
			break;
		}
		n.getParam(_arm_length, mav.parameters.rotors[i].arm_length);
		n.getParam(_direction, mav.parameters.rotors[i].direction);
		n.getParam(_fc, mav.parameters.rotors[i].force_constant);
		n.getParam(_mc, mav.parameters.rotors[i].moment_constant);

	}

	n.getParam("/gazebo_gui/gravity_z", gravity_z);
}
