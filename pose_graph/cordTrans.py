# Tab =  b wrt a
# Right hand frame : X: Forward, Y: Left, Z: Up
# Left hand frame : X: Right, Y: Down, Z: Forward


from scipy.spatial.transform import Rotation as R
import numpy as np
from sys import exit, argv
import math
import argparse


parser = argparse.ArgumentParser(description='Conversion of RoRD transformations to Loop Closure transformations.', 
	formatter_class=argparse.RawTextHelpFormatter)

parser.add_argument(
	'--static_trans', type=str, default='../configs/camWrtBase.txt',
	help="Static transformation of camera wrt Base. In order: \n"  
	"qx qy qz qw \n"
	"tx ty tz"
)

parser.add_argument(
	'--rord_trans', type=str, default='../demo/transLC.npy', help='Numpy matrix of transformation output of RoRD.')

args = parser.parse_args()


def readStaticTrans(static_trans):
	f = open(static_trans, 'r')
	A = f.readlines()
	f.close()

	for i, line in enumerate(A):
		line = line.rstrip('\n')

		if(i==0):
			(qx, qy, qz, qw) = line.split(' ')
			qx, qy, qz, qw = float(qx), float(qy), float(qz), float(qw)
		elif(i==1):
			(tx, ty, tz) = line.split(' ')
			tx, ty, tz = float(tx), float(ty), float(tz)


	return (qx, qy, qz, qw), (tx, ty, tz)


def right2left():
	# Returns: Right wrt left = TL_R = TR2L

	thetaX = math.radians(90)
	thetaY = math.radians(-90)

	Rx = np.array([[1, 0, 0], [0, math.cos(thetaX), -math.sin(thetaX)], [0, math.sin(thetaX), math.cos(thetaX)]])
	Ry = np.array([[math.cos(thetaY), 0, math.sin(thetaY)], [0, 1, 0], [-math.sin(thetaY), 0, math.cos(thetaY)]])

	TR2L = np.identity(4)
	TR2L[0:3, 0:3] =  Ry @ Rx 
	
	return TR2L


def camWrtBase(static_trans):
	# Returns: Camera wrt Base

	# rosrun tf tf_echo base_link camera_link
	(qx, qy, qz, qw), (tx, ty, tz) = readStaticTrans(static_trans)


	TBC = np.identity(4)

	RBC = R.from_quat([qx, qy, qz, qw]).as_dcm()

	TBC[0:3, 0:3] = RBC
	TBC[0, 3] = tx
	TBC[1, 3] = ty
	TBC[2, 3] = tz

	return TBC


def leftTransToRight(T12L):
	# Returns: 2 wrt 1 in Right hand frame

	TLR = right2left()

	T12R = np.linalg.inv(TLR) @ T12L @ TLR

	return T12R


def printEdge(T):
	# T = np.linalg.inv(T)
	Rot = R.from_dcm(T[0:3, 0:3])
	euler = Rot.as_euler('xyz', degrees=True)
	# print("{:.2f} {:.2f} {:.2f}".format(euler[0], euler[1], euler[2]))
	# print(T)
	# print('---')

	print(T[0, 3], T[0, 1], euler[2])



if __name__ == '__main__':
	# C2 wrt C1 in Left hand frame
	TC1C2L = np.load(args.rord_trans)

	TC1C2R = leftTransToRight(TC1C2L)

	# Camera wrt Base in Right hand frame
	TBC = camWrtBase(args.static_trans)

	# B2 wrt B1 in Right hand frame
	TB1B2R = TBC @ TC1C2R @ np.linalg.inv(TBC)

	printEdge(TB1B2R)
