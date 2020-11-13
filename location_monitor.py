#!/usr/bin/env python 
import math
import numpy as np
import rospy
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
import matplotlib.pyplot as plt
import random

###########################################################################################

"""GLOABAL DATA"""

x_data = []
y_data = []

goal = [] #[x_coordinate, y_coordinate]
obstacles = [[6, 4.8], [3,3]] #[ [x-1, y-1], [x-2, y-2]... [x-n, y-n]]
#these are centers of obstacles

#now for painting them on graph
obs_datax = []
obs_datay = []
obs_data_x = []
obs_data_y = []
obs_data_x_safe = []
obs_data_y_safe = []

detect = {}

#k = [theta,x,y]
kp = [10,1,1] 
kd = [-0.1,-0.1,-0.1]
ki = [0.0,0.00,0.00]

#integral terms for pid, however for turtlesim with no noise/inteference ki = 0 gives
#sufficient result (way better also)
int_velx = 0.0
int_vely = 0.0
int_omega = 0.0

###########################################################################################

#here bot = [x,y,theta], obst = [x,y]
def _dist(bot, obst):
	d = math.sqrt( (bot[0] - obst[0])**2 + (bot[1] - obst[1])**2 )
	return d

def _theta(bot,obst):
	dy = obst[1] - bot[1]
	dx = obst[0] - bot[0]

	if dx != 0:
		ang = math.atan(dy/dx)
	
	if dy >= 0 and dx > 0 :
		d_theta = ang
	elif dy >= 0 and dx < 0 :
		d_theta = 3.14159 + ang
	elif dy <= 0 and dx > 0 :
		d_theta = 2*3.14159 + ang
	elif dy <= 0 and dx < 0 :
		d_theta = 3.14159 + ang
	elif dx == 0.0 and dy != 0.0:
		if dy > 0:
			d_theta = 3.14159/2
		else:
			d_theta = 3*3.14159/2

	return d_theta

###########################################################################################

def _sonar(state):
	global obstacles, detect, obs_data

	detect = {}

	obs_in_way = False
	if not obstacles:
		return (obs_in_way, [], [], {})
	d_obs = []
	t_obs = []
	
	for i in obstacles:
		t_obs.append(_theta(state,i) - state[2])

	if max(t_obs) < -1.54 or min(t_obs) > 1.54:
		return (obs_in_way, [], [], {})
	else :
		for i in obstacles:
			d_obs.append(_dist(state, i))
		if min(d_obs) > 4:
			return (obs_in_way, [], [], {})
		else:
			for i in range(len(obstacles)):
				detect[ t_obs[i] ] = d_obs[i]
				obs_datax.append((state[0] + d_obs[i]*math.cos(state[2]+t_obs[i])))
				obs_datay.append((state[1] + d_obs[i]*math.sin(state[2]+t_obs[i])))
			return (True, t_obs, d_obs, detect)

###########################################################################################

def _publish(v,w):
	pub = rospy.Publisher('/turtle1/cmd_vel',Twist,queue_size = 10)
	msg = Twist()
	msg.linear.x = v
	msg.angular.z = w
	rospy.loginfo( "linear velocity = "+str(v)+", angular velocity = "+ str(w) )
	pub.publish(msg)

###########################################################################################

def PID(state, goal, vel, omega):

	global int_velx, int_vely,int_omega,kp,kd,ki

	#desired value for omega
	dx = goal[0]-state[0]
	dy = goal[1]-state[1]

	d_theta = math.atan2(dy,dx)

	if d_theta < 0:
		d_theta = d_theta + 6.283185307179586 #2pi

	if dy == 0 and dx == 0:
		d_theta = state[2]

	err_theta = d_theta - state[2]

	#for shortest angular distance(v.2), this removes the fuzziness in previous version
	if abs(err_theta) > 3.141592653589793:
		#print "True"
		if err_theta < 0:
			err_theta = err_theta + 6.283185307179586
		elif err_theta > 0:
			err_theta = err_theta - 6.283185307179586

	#pid for controlling theta:
	d_omega = kp[0]*(err_theta) + kd[0]*(omega) + ki[0]*(int_omega)

	#pid for x:
	d_velx = kp[1]*(goal[0] - state[0]) + kd[1]*vel*(math.cos(state[2])) + ki[1]*int_velx

	#pid for y:
	d_vely = kp[2]*(goal[1] - state[1]) + kd[2]*vel*(math.sin(state[2])) + ki[2]*int_vely

	#speed calc:
	d_vel = math.sqrt(d_velx**2 + d_vely**2 )

	
	#for shortest angular path
	#if state[2] - d_theta > 3.14159 or state[2] - d_theta < -3.14159:
	#	d_omega = -d_omega

	
	#int_omega = int_omega + (err_theta)/600.0
	#int_velx = int_velx + (goal[0] - state[0])/600.0
	#int_vely = int_vely + (goal[1] - state[1])/600.0

	
	"""
	ADD IF U WANT TOLERENCE
	if abs(dx) < 0.01 and abs(dy) < 0.01:
		d_vel = 0
		d_omega = 0
	"""
	if d_vel > 4:
		d_vel = 4
	#d_vel = d_vel - random.random(), trying to add disturbance
	return (d_vel, d_omega)

######################################################################################


def obsAvoidance(state, goal, t_obs, d_obs, detected):
	"""USing Limit Cycles"""
	global obstacles

	c = 0

	min_dist = min(d_obs)
	for i in t_obs:
		if detected[i] == min_dist:
			req_t = i
			break
		c = c + 1

	#alpha = _theta(obstacles[c], goal)
	alpha = math.atan2(goal[1]-obstacles[c][1], goal[0]-obstacles[c][0])
	if alpha < 0:
		alpha = alpha + 6.283185307179586

	X_s, Y_s = wrtTargetFrame(state, goal, obstacles[c], alpha)

	if Y_s >= 0:
		sign = 1
	else:
		sign = -1

	X_s_dot = (sign)*Y_s + X_s*(1 - X_s**2 - Y_s**2)
	Y_s_dot = -(sign)*X_s + Y_s*(1 - X_s**2 - Y_s**2)
	
	d_theta = math.atan2(Y_s_dot, X_s_dot)	

	if d_theta < 0:
		d_theta = d_theta + 6.283185307179586 #2pi

	err_theta = d_theta

	#for shortest angular distance(v.2), this removes the fuzziness in previous version
	if abs(err_theta) > 3.141592653589793:
		#print "True"
		if err_theta < 0:
			err_theta = err_theta + 6.283185307179586
		elif err_theta > 0:
			err_theta = err_theta - 6.283185307179586

	ang_vel = 15*err_theta

	X_dot, Y_dot = inWorldFrame(X_s_dot, Y_s_dot, alpha)
	vel = math.sqrt(X_dot**2 + Y_dot**2)

	return( vel, ang_vel )


#####################################################################################


def wrtTargetFrame(state, goal, obs, alpha):

	C = np.array( [[math.cos(alpha), math.sin(alpha)], [-math.sin(alpha), math.cos(alpha)]] )
	X_w = np.array([ state[0] - obs[0], state[1] - obs[1] ])
	X_s = np.dot(C, X_w)
	return (X_s[0], X_s[1])

def inWorldFrame(xs_dot, ys_dot, alpha):

	C = np.array( [[math.cos(alpha), -math.sin(alpha)], [math.sin(alpha), math.cos(alpha)]] )
	V_s = np.array([xs_dot, ys_dot])
	V_w = np.dot(C, V_s)
	return (V_w[0], V_w[1])


#####################################################################################
def callback(_msg):
		
	global x_data, y_data, goal

	x = _msg.x
	y = _msg.y
	theta = _msg.theta
	curr_vel = _msg.linear_velocity
	curr_omega = _msg.angular_velocity
	if theta < 0:
		theta = 6.283185307179586 + theta
	x_data.append(x)
	y_data.append(y)
	_path, theta_obs, dist_obs, _detetcted = _sonar([x,y,theta])
	if _path == False:
		desired_vel, desired_omega = PID( [x,y,theta], goal, curr_vel,  curr_omega)
	else:
		if min(dist_obs) > 1:
			desired_vel, desired_omega = PID( [x,y,theta], goal, curr_vel,  curr_omega)
		else:
			desired_vel, desired_omega = obsAvoidance([x,y,theta], goal, theta_obs, dist_obs, _detetcted)
	_publish(desired_vel,desired_omega)
	
#####################################################################################

def main():
	rospy.init_node('location_monitor')
	rospy.Subscriber('turtle1/pose', Pose, callback)
	pub = rospy.Publisher('turtle1/cmd_vel', Twist, queue_size = 10)
	#print "hello"____printed only once
	rospy.spin()

if __name__ == '__main__':
	try:
		print "Enter the x_coordinate of goal:"
		goal.append(float(raw_input()))
		print "Enter the y_coordinate of goal:"
		goal.append(float(raw_input()))
		main()

		for i in obstacles:
			for j in np.arange(0.0,6.2,0.5):
				obs_data_x.append(i[0] + 0.5*math.cos(j))
				obs_data_y.append(i[1] + 0.5*math.sin(j))
				obs_data_x_safe.append(i[0] + 1*math.cos(j))
				obs_data_y_safe.append(i[1] + 1*math.sin(j))
			plt.plot(obs_data_x,obs_data_y,'r--')
			plt.plot(obs_data_x_safe,obs_data_y_safe,'g--')
		plt.plot(x_data, y_data,'b--')
		plt.plot(obs_datax, obs_datay,'ro')
		plt.axis([0,11,0,11])
		plt.show()
	except rospy.ROSInterruptException:
		pass	