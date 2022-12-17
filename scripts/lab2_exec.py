#!/usr/bin/env python
'''
We get inspirations of Tower of Hanoi algorithm from the website below.
This is also on the lab manual.
Source: https://www.cut-the-knot.org/recurrence/hanoi.shtml
'''

import os
import argparse
import copy
import time
import rospy
import rospkg
import numpy as np
import yaml
import sys
import sensor_msgs.point_cloud2 as pc2
from math import pi
from lab2_header import *

# 20Hz
SPIN_RATE = 20

# How many points do we want to calculate regression on
MEAS_WINDOW = 3

# UR3 home location
home = [155.37*pi/180.0, -70.60*pi/180.0, 86.45*pi/180.0, -103.28*pi/180.0, -90.12*pi/180.0, 0]

# Hanoi tower location 1
Q11 = [2.7259748150991014, -0.4094092025060835, 0.9266684236529832, -2.088055547941796,
      -1.5707963267948966, 1.1551784883042049]
Q12 = [2.7259748150991014, -0.5971319206342605, 0.8311620025902458, -1.8048264087508827,
      -1.5707963267948966, 1.1551784883042049]
Q13 = [2.624079791370164, -0.7314904165477265, 1.6649692360519455, -2.5042751462991157,
      -1.5707963267948966, 1.0532834645752673]
Q21 = [2.624079791370164, -1.003950203147513, 1.591315032644947, -2.158161156292331,
      -1.5707963267948966, 1.0532834645752673]
Q22 = [2.458071665798306, -0.930855911241727, 2.2086155554443176, -2.8485559709974875,
      -1.5707963267948966, 0.8872753390034096]
Q23 = [2.458071665798306, -1.3402611048778377, 2.119906386765944, -2.3504416086830027,
      -1.5707963267948966, 0.8872753390034096]
Q31 = [2.9213154325078783, -0.45125568828853935, 1.0193929551460807, -2.1389335936524376,
      -1.5707963267948966, 1.350519105712982]
Q32 = [2.9213154325078783, -0.6477673994444654, 0.9305417319283986, -1.8535706592788297,
      -1.5707963267948966, 1.350519105712982]
Q33 = [2.8640863435252157, -0.761245999788282, 1.7380602710820459, -2.5476105980886605,
      -1.5707963267948966, 1.2932900167303192]

thetas = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

digital_in_0 = 0
analog_in_0 = 0

suction_on = True
suction_off = False
current_io_0 = False
current_position_set = False

dart_pos = np.zeros((MEAS_WINDOW, 3))
dart_timestamp = np.zeros((MEAS_WINDOW))
dart_vel = np.zeros(3)

# UR3 current position, using home position for initialization
current_position = copy.deepcopy(home)

############## Your Code Start Here ##############
"""
TODO: Initialize Q matrix
"""

Q = [ [Q11, Q12, Q13], \
      [Q21, Q22, Q23], \
      [Q31, Q32, Q33] ]
############### Your Code End Here ###############

############## Your Code Start Here ##############

"""
TODO: define a ROS topic callback funtion for getting the state of suction cup
Whenever ur3/gripper_input publishes info this callback function is called.
"""
def ROS_callback(msg):
    global current_io_0

    current_io_0 = msg.DIGIN

    # return current_io_0

############### Your Code End Here ###############

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
    point_count = 0.0
    for p in pc2.read_points(pc, field_names={'x','y','z'}, skip_nans=True):
        x_pos += p[0]
        y_pos += p[1]
        z_pos += p[2]
        point_count += 1.0
    x_pos /= point_count
    y_pos /= point_count
    z_pos /= point_count

    dart_pos[:-1][0] = x[1:][0]; dart_pos[-1][0] = x_pos
    dart_pos[:-1][1] = x[1:][1]; dart_pos[-1][1] = y_pos
    dart_pos[:-1][2] = x[1:][2]; dart_pos[-1][2] = z_pos
    dart_timestamp[:-1] = dart_timestamp[1:]; dart_timestamp[-1] = rospy.get_time()
    
    x_vel = 0.0
    y_vel = 0.0
    z_vel = 0.0
    for i in range(MEAS_WINDOW-1):
        dt = dart_timestamp[i+1] - dart_timestamp[i]
        x_vel += (dart_pos[i+1][0] - dart_pos[i][0])/dt
        y_vel += (dart_pos[i+1][1] - dart_pos[i][1])/dt
        z_vel += (dart_pos[i+1][2] - dart_pos[i][2])/dt
    x_vel /= (MEAS_WINDOW-1)
    y_vel /= (MEAS_WINDOW-1)
    z_vel /= (MEAS_WINDOW-1)
    
    dart_vel[0] = x_vel; dart_vel[1] = y_vel; dart_vel[2] = z_vel

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

            at_goal = 1

        loop_rate.sleep()

        if(spin_count >  SPIN_RATE*5):

            pub_cmd.publish(driver_msg)
            rospy.loginfo("Just published again driver_msg")
            spin_count = 0

        spin_count = spin_count + 1

    return error

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
            rospy.loginfo("Goal is reached!")

        loop_rate.sleep()

        if(spin_count >  SPIN_RATE*5):

            pub_cmd.publish(driver_msg)
            rospy.loginfo("Just published again driver_msg")
            spin_count = 0

        spin_count = spin_count + 1

    return error

############## Your Code Start Here ##############

def move_block(pub_cmd, loop_rate, start_loc, start_height, \
               end_loc, end_height):
    global Q
    global digital_in_0
    global suction_on
    global suction_off


    ### Hint: Use the Q array to map out your towers by location and "height".
    start_block = Q[start_loc][start_height]
    end_block = Q[end_loc][end_height]

    #Move to home position
    move_arm(pub_cmd,loop_rate, home ,1.0,1.0)
    time.sleep(0.5)

    #Move to pick up the block
    move_arm(pub_cmd,loop_rate, start_block,1.0,1.0)
    time.sleep(0.5)

    #Turn the gripper on
    suction = gripper(pub_cmd, loop_rate, suction_on)
    rospy.sleep(1.0)

    if (current_io_0 == suction):
        gripper(pub_cmd, loop_rate, suction_off)
        print('Robot stopped because of the target cube is missing')
        sys.exit()
    else:
        move_arm(pub_cmd,loop_rate, home ,1.0,1.0)

        #Move to the location
        move_arm(pub_cmd, loop_rate, end_block, 1.0,1.0)
        time.sleep(0.5)

        #Turn suction off
        gripper(pub_cmd, loop_rate, suction_off)
        time.sleep(0.5)

        move_arm(pub_cmd,loop_rate, home ,1.0,1.0)

    error = 0

    return error

############### Your Code End Here ###############

def main():scan

    global home
    global Q
    global SPIN_RATE

    # Initialize ROS node
    rospy.init_node('lab2node')

    # Initialize publisher for ur3/command with buffer size of 10
    pub_command = rospy.Publisher('ur3/command', command, queue_size=10)
    sub_position = rospy.Subscriber('ur3/position', position, position_callback)
    sub_gripper_input = rospy.Subscriber('ur3/gripper_input',gripper_input,ROS_callback)
    sub_depth_camera = rospy.Subscriber('camera/depth/points', PointCloud2, pc_callback)

    # while( start_tower):
    start_tower_string = input("Enter start tower position here <either 1 2 or 3>")
    print("The start tower is at" + str(start_tower_string) + "\n")
    end_tower_string = input("Enter end tower position here <either 1 2 or 3>")
    print("The endtower is at" + str(end_tower_string) + "\n")

    if (int(start_tower_string) == 1):
        start_tower = 0
    elif (int(start_tower_string) == 2):
        start_tower = 1
    elif (int(start_tower_string) == 3):
        start_tower = 2
    else:
        print("Please just enter the character 1 2 or 3  \n\n")

    if (int(end_tower_string) == 1):
        end_tower = 0
    elif (int(end_tower_string) == 2):
        end_tower = 1
    elif (int(end_tower_string) == 3):
        end_tower = 2
    else:
        print("Please just enter the character 1 2 or 3  \n\n")

    last_tower = 3 - start_tower - end_tower




    ############### Your Code End Here ###############

    # Check if ROS is ready for operation
    while(rospy.is_shutdown()):
        print("ROS is shutdown!")

    rospy.loginfo("Sending Goals ...")

    loop_rate = rospy.Rate(SPIN_RATE)

    ############## Your Code Start Here ##############
    # TODO: modify the code so that UR3 can move tower accordingly from user input

        # move start top to end bot
    move_block(pub_command, loop_rate,start_tower,0,end_tower,2)

    #move start second to last  bot
    
    move_block(pub_command,loop_rate,start_tower,1,last_tower,2)

    #move end bot to last mid
    move_block(pub_command,loop_rate,end_tower,2,last_tower,1)

    #move start bot to end bot
    move_block(pub_command,loop_rate,start_tower,2,end_tower,2)

    #move last mid to start bot
    move_block(pub_command,loop_rate,last_tower,1,start_tower,2)

    #move last bot to end mid
    move_block(pub_command,loop_rate,last_tower,2,end_tower,1)

    #move start bot to end top
    move_block(pub_command,loop_rate,start_tower,2,end_tower,0)
    # while(loop_count > 0):

    #     move_arm(pub_command, loop_rate, home, 4.0, 4.0)

    #     rospy.loginfo("Sending goal 1 ...")
    #     move_arm(pub_command, loop_rate, Q[0][0], 4.0, 4.0)

    #     gripper(pub_command, loop_rate, suction_on)
    #     # Delay to make sure suction cup has grasped the block
    #     time.sleep(1.0)

    #     rospy.loginfo("Sending goal 2 ...")
    #     move_arm(pub_command, loop_rate, Q[1][1], 4.0, 4.0)

    #     rospy.loginfo("Sending goal 3 ...")
    #     move_arm(pub_command, loop_rate, Q[2][0], 4.0, 4.0)

    #     loop_count = loop_count - 1

    # gripper(pub_command, loop_rate, suction_off)


    ############### Your Code End Here ###############

if __name__ == '__main__':

    try:
        main()
    # When Ctrl+C is executed, it catches the exception
    except rospy.ROSInterruptException:
        pass


