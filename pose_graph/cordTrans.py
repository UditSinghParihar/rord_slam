# Tab =  b wrt a
# Right hand frame : X: Forward, Y: Left, Z: Up
# Left hand frame : X: Right, Y: Down, Z: Forward


from scipy.spatial.transform import Rotation as R
import numpy as np
from sys import exit, argv
import math


def right2left():
	# Returns: Right wrt left = TL_R = TR2L

	thetaX = math.radians(90)
	thetaY = math.radians(-90)

	Rx = np.array([[1, 0, 0], [0, math.cos(thetaX), -math.sin(thetaX)], [0, math.sin(thetaX), math.cos(thetaX)]])
	Ry = np.array([[math.cos(thetaY), 0, math.sin(thetaY)], [0, 1, 0], [-math.sin(thetaY), 0, math.cos(thetaY)]])

	TR2L = np.identity(4)
	TR2L[0:3, 0:3] =  Ry @ Rx 
	
	return TR2L


def baseWrtCam():
	# Returns: Base wrt Camera

	TCB = np.identity(4)

	qx, qy, qz, qw = 0.557, 0.662, -0.383, -0.323
	tx, ty, tz = -7.195, 0.849, 1.113

	RCB = R.from_quat([qx, qy, qz, qw]).as_dcm()

	TCB[0:3, 0:3] = RCB
	TCB[0, 3] = tx
	TCB[1, 3] = ty
	TCB[2, 3] = tz

	return TCB


def leftTransToRight(T12L):
	# Returns: 2 wrt 1 in Right hand frame

	TLR = right2left()

	T12R = np.linalg.inv(TLR) @ T12L @ TLR

	return T12R


def printEdge(T):
	# T = np.linalg.inv(T)
	Rot = R.from_dcm(T[0:3, 0:3])
	euler = Rot.as_euler('zxy', degrees=True)

	print(T[0, 3], T[0, 1], euler[2])



if __name__ == '__main__':
	# C2 wrt C1 in Left hand frame
	TC1C2L = np.load(argv[1])

	TCB = baseWrtCam()

	# Camera wrt Base in Right hand frame
	TBC = np.linalg.inv(TCB)

	TC1C2R = leftTransToRight(TC1C2L)

	# B2 wrt B1 in Right hand frame
	TB1B2R = TBC @ TC1C2R @ np.linalg.inv(TBC)

	printEdge(TB1B2R)
