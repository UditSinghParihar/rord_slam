'''
Transformation convention: 
T2_1 : 2 wrt 1

'''


import matplotlib.pyplot as plt
from sys import argv, exit
import math
import numpy as np
import os
from scipy.spatial.transform import Rotation as R
from mpl_toolkits import mplot3d


def readPose(filename):
	f = open(filename, 'r')
	A = f.readlines()
	f.close()

	X = []
	Y = []
	Z = []
	Qx = []
	Qy = []
	Qz = []
	Qw = []

	for i, line in enumerate(A):
		if(i % 1 == 0):
			(x, y, z, qx, qy, qz, qw) = line.split(' ')

			X.append(float(x))
			Y.append(float(y))
			Z.append(float(z))
			Qx.append(float(qx))
			Qy.append(float(qy))
			Qz.append(float(qz))
			Qw.append(float(qw.rstrip('\n')))

	return X, Y, Z, Qx, Qy, Qz, Qw


def draw(X, Y, Z):
	fig = plt.figure()
	ax = plt.axes(projection='3d')

	# ax.plot3D(X, Y, Z, 'k-')
	ax.scatter3D(X, Y, Z, c=Z, cmap='Greens')

	ax.set_aspect('equal', 'datalim')
	ax.margins(0.1)

	plt.show()


def drawTwo(X, Y, Z, X2, Y2, Z2):
	fig = plt.figure()
	ax = plt.axes(projection='3d')

	# ax.plot3D(X, Y, Z, 'k-')
	ax.scatter3D(X, Y, Z, c=Z, cmap='Greens')

	# ax.plot3D(X2, Y2, Z2, 'k-')
	ax.scatter3D(X2, Y2, Z2, c=Z2, cmap='Reds');

	ax.set_aspect('equal', 'datalim')
	ax.margins(0.1)

	plt.show()


def addNoise(X, Y, Z, Qx, Qy, Qz, Qw):
	# xN = np.zeros(len(X)); yN = np.zeros(len(Y)); zN = np.zeros(len(Z))

	XN = np.zeros(len(X)); YN = np.zeros(len(Y)); ZN = np.zeros(len(Z))
	QxN = np.zeros(len(Qx)); QyN = np.zeros(len(Qy)); QzN = np.zeros(len(Qz)); QwN = np.zeros(len(Qw))

	XN[0] = X[0]; YN[0] = Y[0]; ZN[0] = Z[0]; QxN[0] = Qx[0]; QyN[0] = Qy[0]; QzN[0] = Qz[0]; QwN[0] = Qw[0]

	for i in range(1, len(X)):
		# Get T2_1
		p1 = (X[i-1], Y[i-1], Z[i-1], Qx[i-1], Qy[i-1], Qz[i-1], Qw[i-1])
		p2 = (X[i], Y[i], Z[i], Qx[i], Qy[i], Qz[i], Qw[i])

		R1_w = R.from_quat([p1[3], p1[4], p1[5], p1[6]]).as_dcm()
		R2_w = R.from_quat([p2[3], p2[4], p2[5], p2[6]]).as_dcm()

		T1_w = np.identity(4)
		T2_w = np.identity(4)

		T1_w[0:3, 0:3] = R1_w
		T2_w[0:3, 0:3] = R2_w

		T1_w[0, 3] = p1[0] 
		T1_w[1, 3] = p1[1]
		T1_w[2, 3] = p1[2]

		T2_w[0, 3] = p2[0]
		T2_w[1, 3] = p2[1]
		T2_w[2, 3] = p2[2]

		T2_1 = np.dot(np.linalg.inv(T1_w), T2_w)

		dx, dy, dz = T2_1[0, 3], T2_1[1, 3], T2_1[2, 3]
		dyaw, dpitch, droll = list(R.from_dcm(T2_1[0:3, 0:3]).as_euler('zyx'))
		# dqx, dqy, dqz, dqw = list(R.from_dcm(T2_1[0:3, 0:3]).as_quat())
		
		# Add noise
		if(i<5):
			xNoise = 0; yNoise = 0; zNoise = 0; rollNoise = 0; pitchNoise = 0; yawNoise = 0
		else:
			# np.random.seed(42)
			# xNoise = np.random.normal(0, 0.08); yNoise = np.random.normal(0, 0.08); tNoise = np.random.normal(0, 0.002)
			# xNoise = 0.005; yNoise = 0.005; tNoise = -0.0005
			# xNoise = 0.01; yNoise = 0.01; tNoise = 0.0007
			xNoise = 0; yNoise = 0; zNoise = 0; rollNoise = 0.00015; pitchNoise = 0.00015; yawNoise = 0.00015

		dx += xNoise; dy += yNoise; dz += zNoise
		dyaw += yawNoise; dpitch += pitchNoise; droll += rollNoise

		# Convert to T2_1'
		R2_1N = R.from_euler('zyx', [dyaw, dpitch, droll]).as_dcm()
		
		T2_1N = np.identity(4)
		T2_1N[0:3, 0:3] = R2_1N

		T2_1N[0, 3] = dx
		T2_1N[1, 3] = dy
		T2_1N[2, 3] = dz

		# Get T2_w' = T1_w' . T2_1'
		p1 = (XN[i-1], YN[i-1], ZN[i-1], QxN[i-1], QyN[i-1], QzN[i-1], QwN[i-1])
		R1_wN = R.from_quat([p1[3], p1[4], p1[5], p1[6]]).as_dcm()
		
		T1_wN = np.identity(4)
		T1_wN[0:3, 0:3] = R1_wN

		T1_wN[0, 3] = p1[0] 
		T1_wN[1, 3] = p1[1]
		T1_wN[2, 3] = p1[2]

		T2_wN = np.dot(T1_wN, T2_1N)

		# Get x2', y2', z2', qx2', qy2', qz2', qw2'
		x2N, y2N, z2N = T2_wN[0, 3], T2_wN[1, 3], T2_wN[2, 3]
		qx2N, qy2N, qz2N, qw2N = list(R.from_dcm(T2_wN[0:3, 0:3]).as_quat())

		XN[i] = x2N; YN[i] = y2N; ZN[i] = z2N
		QxN[i] = qx2N; QyN[i] = qy2N; QzN[i] = qz2N; QwN[i] = qw2N

	return (XN, YN, ZN, QxN, QyN, QzN, QwN)


def writeG2O(X, Y, Z, Qx, Qy, Qz, Qw, file):
	g2o = open(file, 'w')
	sp = ' '

	for i, (x, y, z, qx, qy, qz, qw) in enumerate(zip(X, Y, Z, Qx, Qy, Qz, Qw)):
		line = "VERTEX_SE3:QUAT " + str(i) + sp + str(x) + sp + str(y) + sp + str(z) + sp + str(qx) + sp + str(qy) + sp + str(qz) + sp + str(qw) + '\n'
		g2o.write(line)

	# Odometry
	# T1_w : 1 with respect to world
	g2o.write("\n\n\n# Odometry constraints\n\n\n\n")
	info = '20 0 0 0 0 0 20 0 0 0 0 20 0 0 0 20 0 0 20 0 20'

	for i in range(1, len(X)):
		p1 = (X[i-1], Y[i-1], Z[i-1], Qx[i-1], Qy[i-1], Qz[i-1], Qw[i-1])
		p2 = (X[i], Y[i], Z[i], Qx[i], Qy[i], Qz[i], Qw[i])

		R1_w = R.from_quat([p1[3], p1[4], p1[5], p1[6]]).as_dcm()
		R2_w = R.from_quat([p2[3], p2[4], p2[5], p2[6]]).as_dcm()

		T1_w = np.identity(4)
		T2_w = np.identity(4)

		T1_w[0:3, 0:3] = R1_w
		T2_w[0:3, 0:3] = R2_w

		T1_w[0, 3] = p1[0] 
		T1_w[1, 3] = p1[1]
		T1_w[2, 3] = p1[2]

		T2_w[0, 3] = p2[0]
		T2_w[1, 3] = p2[1]
		T2_w[2, 3] = p2[2]

		T2_1 = np.dot(np.linalg.inv(T1_w), T2_w)

		dx, dy, dz = T2_1[0, 3], T2_1[1, 3], T2_1[2, 3]
		dqx, dqy, dqz, dqw = list(R.from_dcm(T2_1[0:3, 0:3]).as_quat())
		
		line = "EDGE_SE3:QUAT " + str(i-1) + sp + str(i) + sp + str(dx) + sp + str(dy) + sp + str(dz) + sp + str(dqx) + sp + str(dqy) + sp + str(dqz) + sp + str(dqw) + sp +  info + '\n'
		g2o.write(line)

	g2o.write("FIX 0\n")
	g2o.close()


def readG2o(fileName):
	f = open(fileName, 'r')
	A = f.readlines()
	f.close()

	X = []
	Y = []
	THETA = []

	for line in A:
		if "VERTEX_SE2" in line:
			(ver, ind, x, y, theta) = line.split(' ')
			X.append(float(x))
			Y.append(float(y))
			THETA.append(float(theta.rstrip('\n')))

	return (X, Y, THETA)


if __name__ == '__main__':
	dirc = os.path.dirname(argv[1])
	# print(os.path.dirname(argv[1]))

	X, Y, Z, Qx, Qy, Qz, Qw = readPose(argv[1])
	# draw(X, Y, Z)
	writeG2O(X, Y, Z, Qx, Qy, Qz, Qw, os.path.join(dirc, "gt.g2o"))

	(XN, YN, ZN, QxN, QyN, QzN, QwN) = addNoise(X, Y, Z, Qx, Qy, Qz, Qw)
	# draw(XN, YN, ZN)
	writeG2O(XN, YN, ZN, QxN, QyN, QzN, QwN, os.path.join(dirc, "noise.g2o"))

	drawTwo(X, Y, Z, XN, YN, ZN)

	# (XOpt, YOpt, ZOpt) = readG2o(os.path.join(dirc, "gt.g2o"))
	# draw(XOpt, YOpt, ZOpt)
