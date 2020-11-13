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

	std::string s;
	double mass;
	
	Inertia inertia;	

	Rotors rotors[6];
	
};

struct Position {
	double x, y, z;
};

struct Velocity {
	double x, y, z;
};

struct Quaternion {
    double w, x, y, z;
};

struct EulerAngles {
    double roll, pitch, yaw;
};

struct Acceleration {
	double x, y, z;
};

struct AngularVelocity {
	double roll, pitch, yaw;
};

struct Mav{

	Parameters parameters;

	Position pos;
	EulerAngles eul;
	Velocity vel;
	Acceleration acc;
	AngularVelocity omg;

};

Eigen::MatrixXd oRv(double roll, double pitch, double yaw){

	Eigen::MatrixXd R( 3, 3);
	R << c(yaw)*c(pitch) - s(roll)*s(yaw)*sin(pitch) , -c(roll)*s(yaw) , c(yaw)*s(pitch) + c(pitch)*s(roll)*s(yaw),
		 s(yaw)*c(pitch) + s(roll)*c(yaw)*sin(pitch) ,  c(roll)*c(yaw) , s(yaw)*s(pitch) - c(pitch)*s(roll)*c(yaw),
		 			  -c(roll)*s(pitch)				 , 		s(roll)	   , 			c(roll)*c(pitch);

	return R;
}

///////////__FROM____WIKIPEDIA__///////////////////////////////////

EulerAngles ToEulerAngles(Quaternion q) {
    EulerAngles angles;

    // roll (x-axis rotation)
    double sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
    double cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
    angles.roll = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = 2 * (q.w * q.y - q.z * q.x);
    if (std::abs(sinp) >= 1)
        angles.pitch = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        angles.pitch = std::asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    angles.yaw = std::atan2(siny_cosp, cosy_cosp);

    return angles;
}
///////////////////////////////////////////////////////////////////