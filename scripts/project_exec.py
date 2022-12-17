#!/usr/bin/env python
import copy
import time
import rospy
import sys
import numpy as np
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from project_header import *
from project_func import *

# 20Hz
SPIN_RATE = 20 

# How many points do we want to calculate regression on
MEAS_WINDOW = 5

# UR3 home location
home = [120*PI/180.0, -90*PI/180.0, 90*PI/180.0, -90*PI/180.0, -90*PI/180.0, 0*PI/180.0]

# UR3 current position, using home position for initialization
current_position = copy.deepcopy(home)

thetas = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

digital_in_0 = 0
analog_in_0 = 0.0

suction_on = True
suction_off = False

current_io_0 = False
current_position_set = False

dart_pos = np.zeros((MEAS_WINDOW, 3))
dart_timestamp = np.zeros((MEAS_WINDOW))
dart_vel = np.zeros(3)

"""
Whenever ur3/gripper_input publishes info this callback function is called.
"""
def input_callback(msg):

	global digital_in_0
	global analog_in_0
	digital_in_0 = msg.DIGIN
	digital_in_0 = digital_in_0 & 1 # Only look at least significant bit, meaning index 0
	analog_in_0 = msg.AIN0

"""
Whenever ur3/position publishes info, this callback function is called.
"""
def position_callback(msg):

	global thetas
	global current_position
	global current_position_set

	thetas[0] = msg.position[0]
	thetas[1] = msg.position[1]
	thetas[2] = msg.position[2]
	thetas[3] = msg.position[3]
	thetas[4] = msg.position[4]
	thetas[5] = msg.position[5]

	current_position[0] = thetas[0]
	current_position[1] = thetas[1]
	current_position[2] = thetas[2]
	current_position[3] = thetas[3]
	current_position[4] = thetas[4]
	current_position[5] = thetas[5]

	current_position_set = True

def pc_callback(pc):

	global dart_pos
	global dart_timestamp

	x_pos = 0.0
	y_pos = 0.0
	z_pos = 0.0
	point_count = 0
	for p in pc2.read_points(pc, field_names={'x','y','z'}, skip_nans=True):
		x_pos += p[0]
		y_pos += p[1]
		z_pos += p[2]
		point_count += 1
	if point_count == 0:
		point_count = 1
	x_pos /= point_count
	y_pos /= point_count
	z_pos /= point_count

	dart_pos = np.roll(dart_pos, 1, axis=0)
	dart_pos[0][0] = x_pos
	dart_pos[0][1] = y_pos
	dart_pos[0][2] = z_pos
	dart_timestamp = np.roll(dart_timestamp, 1)
	dart_timestamp[0] = rospy.get_time()
	# print(dart_timestamp)

	x_vel = 0.0
	y_vel = 0.0
	z_vel = 0.0
	for i in range(MEAS_WINDOW-1):
		dt = dart_timestamp[i] - dart_timestamp[i+1]
		if dt == 0:
			dt = 1
		x_vel += (dart_pos[i][0] - dart_pos[i+1][0])/dt
		y_vel += (dart_pos[i][1] - dart_pos[i+1][1])/dt
		z_vel += (dart_pos[i][2] - dart_pos[i+1][2])/dt
	x_vel /= (MEAS_WINDOW-1)
	y_vel /= (MEAS_WINDOW-1)
	z_vel /= (MEAS_WINDOW-1)
	
	dart_vel[0] = x_vel; dart_vel[1] = y_vel; dart_vel[2] = z_vel
	print(dart_pos)
	# t = calc_intersection(np.array([0, 1, 0]), np.array([0,-.75, 0]), dart_vel, dart_pos[0], np.array([0,0,-1]))
	# intersect = dart_pos[0] + t*dart_vel + t**2 * np.array([0,0,-1])
	# print(intersect)

def two_d_plane_intersection(p1, p2, p3, p4, p5, x):
	p_coeff = np.polyfit([p1[1], p2[1], p3[1], p4[1], p5[1]], [p1[2], p2[2], p3[2], p4[2], p5[2]], 2)
	return p_coeff[0] * x**2 + p_coeff[1] * x + p_coeff[2]


# n: normal vector of plane
# q: point on plane
# d: projective direction
# p: projectile position
# g: gravity vector
def calc_intersection(n, q, d, p, g):
	# we solve for t, a parameterized position on the parabola:
	# p + t*d + (t^2)*g
	discriminant = (np.dot(n, d))**2 - 4*np.dot(n, np.subtract(p, q))*np.dot(n, g)
	# in case of no real roots, return
	if discriminant < 0:
		return np.zeros(3)
	t1 = (np.dot(np.negative(n), d) + math.sqrt(discriminant))/np.dot(2*n, g)
	t2 = (np.dot(np.negative(n), d) - math.sqrt(discriminant))/np.dot(2*n, g)

	t = None
	if t1 > t2:
		t = t1
	else:
		t = t2
	return np.add(p, np.add(t * d, t**2 * g))

"""
Function to control the suction cup on/off
"""
def gripper(pub_cmd, loop_rate, io_0):

	global SPIN_RATE
	global thetas
	global current_io_0
	global current_position

	error = 0
	spin_count = 0
	at_goal = 0

	current_io_0 = io_0

	driver_msg = command()
	driver_msg.destination = current_position
	driver_msg.v = 1.0
	driver_msg.a = 1.0
	driver_msg.io_0 = io_0  
	pub_cmd.publish(driver_msg)

	while(at_goal == 0):

		if( abs(thetas[0]-driver_msg.destination[0]) < 0.0005 and \
			abs(thetas[1]-driver_msg.destination[1]) < 0.0005 and \
			abs(thetas[2]-driver_msg.destination[2]) < 0.0005 and \
			abs(thetas[3]-driver_msg.destination[3]) < 0.0005 and \
			abs(thetas[4]-driver_msg.destination[4]) < 0.0005 and \
			abs(thetas[5]-driver_msg.destination[5]) < 0.0005 ):

			#rospy.loginfo("Goal is reached!")
			at_goal = 1
		
		loop_rate.sleep()

		if(spin_count >  SPIN_RATE*5):

			pub_cmd.publish(driver_msg)
			rospy.loginfo("Just published again driver_msg")
			spin_count = 0

		spin_count = spin_count + 1

	return error

"""
Move robot arm from one position to another
"""
def move_arm(pub_cmd, loop_rate, dest, vel, accel):

	global thetas
	global SPIN_RATE

	error = 0
	spin_count = 0
	at_goal = 0

	driver_msg = command()
	driver_msg.destination = dest
	driver_msg.v = vel
	driver_msg.a = accel
	driver_msg.io_0 = current_io_0
	pub_cmd.publish(driver_msg)

	loop_rate.sleep()

	while(at_goal == 0):

		if( abs(thetas[0]-driver_msg.destination[0]) < 0.0005 and \
			abs(thetas[1]-driver_msg.destination[1]) < 0.0005 and \
			abs(thetas[2]-driver_msg.destination[2]) < 0.0005 and \
			abs(thetas[3]-driver_msg.destination[3]) < 0.0005 and \
			abs(thetas[4]-driver_msg.destination[4]) < 0.0005 and \
			abs(thetas[5]-driver_msg.destination[5]) < 0.0005 ):

			at_goal = 1
			#rospy.loginfo("Goal is reached!")
		
		loop_rate.sleep()

		if(spin_count >  SPIN_RATE*5):

			pub_cmd.publish(driver_msg)
			rospy.loginfo("Just published again driver_msg")
			spin_count = 0

		spin_count = spin_count + 1

	return error

"""
Program run from here
"""
def main():

	global home
	global SPIN_RATE

	# Initialize ROS node
	rospy.init_node('lab2node')

	# Initialize publisher for ur3/command with buffer size of 10
	pub_command = rospy.Publisher('ur3/command', command, queue_size=10)

	# Initialize subscriber to ur3/position & ur3/gripper_input and callback fuction
	# each time data is published
	sub_position = rospy.Subscriber('ur3/position', position, position_callback)
	sub_input = rospy.Subscriber('ur3/gripper_input', gripper_input, input_callback)
	sub_depth_camera = rospy.Subscriber('camera/depth/points', PointCloud2, pc_callback)

	new_dest = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

	# if(len(sys.argv) != 5):
	# 	print("\n")
	# 	print("Invalid number of input!\n")
	# 	print("rosrun lab4pkg_py lab4_exec.py xWgrip yWgrip zWgrip yaw_WgripDegree \n")
	# else:
	# 	print("\nxWgrip: " + sys.argv[1] + ", yWgrip: " + sys.argv[2] + \
	# 		  ", zWgrip: " + sys.argv[3] + ", yaw_WgripDegree: " + sys.argv[4] + "\n")

	# # print(sys.argv)

	# new_dest = lab_invk(float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3]), float(sys.argv[4]))
	

	vel = 50.0
	accel = 50.0

	# Check if ROS is ready for operation
	while(rospy.is_shutdown()):
		print("ROS is shutdown!")

	# Initialize the rate to publish to ur3/command
	loop_rate = rospy.Rate(SPIN_RATE)

	move_arm(pub_command, loop_rate, lab_invk(0.4, 0, 0, 0), 4, 4)

	hold = 0

	while (True):
		if (dart_pos[0][1] != 0 and dart_pos[0][2] != 0
		and dart_pos[1][1] != 0 and dart_pos[1][2] != 0
		and dart_pos[2][1] != 0 and dart_pos[2][2] != 0
		and dart_pos[3][1] != 0 and dart_pos[3][2] != 0
		and dart_pos[4][1] != 0 and dart_pos[4][2] != 0):
			y = two_d_plane_intersection(dart_pos[0], dart_pos[1], dart_pos[2], dart_pos[3], dart_pos[4], -1.0) - 1.0
			print(y)
			hold += 1
		if hold >= 3:
			break
		loop_rate.sleep()

	move_arm(pub_command, loop_rate, lab_invk(y, 0, 0, 0), vel, accel)

	# move_arm(pub_command, loop_rate, lab_invk(0.1, 0.1, 0.1, 0), vel, accel)
	# rospy.sleep(2)
	# move_arm(pub_command, loop_rate, lab_invk(0.3, 0.1, 0.1, 0), vel, accel)
	# rospy.sleep(2)
	# move_arm(pub_command, loop_rate, lab_invk(0.1, 0.3, 0.1, 0), vel, accel)
	# rospy.sleep(2)
	# move_arm(pub_command, loop_rate, lab_invk(0.1, 0.1, 0.1, 0), vel, accel)

	rospy.loginfo("Destination is reached!")



if __name__ == '__main__':
	
	try:
		main()
	# When Ctrl+C is executed, it catches the exception
	except rospy.ROSInterruptException:
		pass