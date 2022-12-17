#!/usr/bin/env python
import math
import numpy as np
from scipy.linalg import expm
from project_header import *

def Get_MS():
	# =================== Your code starts here ====================#
	# Fill in the correct values for S1~6, as well as the M matrix
	M = np.array([[0, -1, 0, 390],
				  [0, 0, -1, 401],
				  [1, 0, 0, 215.5],
				  [0, 0, 0, 1]])
	
	q = np.array([[-150, 150, 162],
				  [-150, 270, 162],
				  [94, 270, 162],
				  [307, 177, 162],
				  [307, 260, 162],
				  [390, 260, 162]])

	w = np.array([[0, 0, 1],
				  [0, 1, 0],
				  [0, 1, 0],
				  [0, 1, 0],
				  [1, 0, 0],
				  [0, 1, 0]])

	v = np.zeros((6, 3))
	S = np.zeros((6, 6))
	for i in range(6):
		v[i] = np.cross(-w[i], q[i])
		S[i] = np.concatenate((w[i], v[i]), axis=None)

	# ==============================================================#
	return M, S

def matrix_to_exponentiate(v):
	out_v = np.zeros((4,4))
	# Skew-Symmetric portion
	out_v[0][1] = -v[2]
	out_v[0][2] = v[1]
	out_v[1][2] = -v[0]
	out_v[1][0] = v[2]
	out_v[2][0] = -v[1]
	out_v[2][1] = v[0]

	# Velocity Portion
	out_v[0][3] = v[3]
	out_v[1][3] = v[4]
	out_v[2][3] = v[5]
	out_v[3][3] = 0

	return out_v


"""
Function that calculates encoder numbers for each motor
"""
def lab_fk(theta1, theta2, theta3, theta4, theta5, theta6):

	# Initialize the return_value
	return_value = [None, None, None, None, None, None]

	# =========== Implement joint angle to encoder expressions here ===========
	print("Foward kinematics calculated:\n")

	# =================== Your code starts here ====================#

	M, S = Get_MS()

	T = M.copy()

	thetas = np.array((theta1, theta2, theta3, theta4, theta5, theta6))

	for i in range(6):
		T = np.dot(expm(matrix_to_exponentiate(S[5-i]) * thetas[5-i]), T)

	# ==============================================================#

	print(str(T) + "\n")

	return_value[0] = theta1 + PI
	return_value[1] = theta2
	return_value[2] = theta3
	return_value[3] = theta4 - (0.5*PI)
	return_value[4] = theta5
	return_value[5] = theta6

	return return_value

"""
Function that calculates an elbow up Inverse Kinematic solution for the UR3
"""
def lab_invk(xWgrip, yWgrip, zWgrip, yaw_WgripDegree):
	# =================== Your code starts here ====================#
	T_bw = np.array([[1,0,0,.15]
					,[0,1,0,-.15]
					,[0,0,1,-.01]
					,[0,0,0,1]])
	[x_grip, y_grip, z_grip, one] = np.dot(T_bw, [xWgrip, yWgrip, zWgrip, 1])
	x_cen = x_grip - 0.0535 * math.cos(yaw_WgripDegree)
	y_cen = y_grip - 0.0535 * math.sin(yaw_WgripDegree)
	z_cen = z_grip

	# print(x_cen, y_cen, z_cen)

	beta1 = math.asin(.110/math.sqrt(x_cen**2 + y_cen**2))
	alpha1 = math.atan2(y_cen, x_cen)
	theta1 = alpha1 - beta1

	theta6 = math.pi/2 - yaw_WgripDegree + theta1

	x_3end = x_cen + (.083 * -math.cos(theta1) + .110 * math.sin(theta1))
	y_3end = y_cen + (.083 * -math.sin(theta1) + .110 * -math.cos(theta1))
	z_3end = z_cen + .141

	# print(x_3end, y_3end, z_3end)

	# Here we do math in the plane of theta1, and relative to the
	# point at the top of link 1
	x_ = math.sqrt(x_3end**2 + y_3end**2)
	z_ = z_3end - .152
	L_3 = .244
	L_5 = .213

	beta2 = math.acos((L_3**2 + L_5**2 - x_**2 - z_**2)/(2 * L_3 * L_5))
	alpha2 = math.acos((x_**2 + z_**2 + L_3**2 - L_5**2)/(2 * L_3 * math.sqrt(x_**2 + z_**2)))

	theta2 = -(alpha2 + math.atan2(z_, x_))
	theta3 = math.pi - beta2
	theta4 = -(math.pi - alpha2 - beta2 - math.atan2(z_, x_))

	# This joint is always fixed at 90 degrees
	theta5 = -math.pi / 2
	# ==============================================================#
	return lab_fk(theta1, theta2, theta3, theta4, theta5, theta6)
