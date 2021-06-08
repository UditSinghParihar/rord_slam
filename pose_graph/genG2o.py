import matplotlib.pyplot as plt
from sys import argv, exit
import math
import numpy as np
import os


def readPose(filename):
	f = open(filename, 'r')
	A = f.readlines()
	f.close()

	X = []
	Y = []
	THETA = []

	for i, line in enumerate(A):
		if(i % 1 == 0):
			(x, y, theta) = line.split(' ')
			# print(x, y, theta.rstrip('\n'))
			X.append(float(x))
			Y.append(float(y))
			THETA.append(math.radians(float(theta.rstrip('\n'))))

	return X, Y, THETA


def draw(X, Y, THETA):
	ax = plt.subplot(111)
	ax.plot(X, Y, 'ro')
	ax.plot(X, Y, 'k-')

	# plt.xlim(-12, 12)
	# plt.ylim(-12, 12)
	ax.set_aspect('equal', 'datalim')
	ax.margins(0.1)

	plt.show()


def drawTheta(X, Y, THETA):
	ax = plt.subplot(111)

	for i in range(len(THETA)):
		x2 = math.cos(THETA[i]) + X[i]
		y2 = math.sin(THETA[i]) + Y[i]
		plt.plot([X[i], x2], [Y[i], y2], 'm->')

	ax.plot(X, Y, 'ro')
	# ax.plot(X, Y, 'k-')

	plt.xlim(-12, 12)
	plt.ylim(-12, 12)
	
	plt.show()


def addNoise(X, Y, THETA):
	xN = np.zeros(len(X)); yN = np.zeros(len(Y)); tN = np.zeros(len(THETA))
	xN[0] = X[0]; yN[0] = Y[0]; tN[0] = THETA[0]

	for i in range(1, len(X)):
		# Get T2_1
		p1 = (X[i-1], Y[i-1], THETA[i-1])
		p2 = (X[i], Y[i], THETA[i])
		T1_w = np.array([[math.cos(p1[2]), -math.sin(p1[2]), p1[0]], [math.sin(p1[2]), math.cos(p1[2]), p1[1]], [0, 0, 1]])
		T2_w = np.array([[math.cos(p2[2]), -math.sin(p2[2]), p2[0]], [math.sin(p2[2]), math.cos(p2[2]), p2[1]], [0, 0, 1]])
		T2_1 = np.dot(np.linalg.inv(T1_w), T2_w)
		del_x = T2_1[0][2]
		del_y = T2_1[1][2]
		del_theta = math.atan2(T2_1[1, 0], T2_1[0, 0])
		
		# Add noise
		if(i<5):
			xNoise = 0; yNoise = 0; tNoise = 0
		else:
			# np.random.seed(42)
			# xNoise = np.random.normal(0, 0.08); yNoise = np.random.normal(0, 0.08); tNoise = np.random.normal(0, 0.002)
			# xNoise = 0.005; yNoise = 0.005; tNoise = -0.0005
			# xNoise = 0.01; yNoise = 0.01; tNoise = 0.0007
			xNoise = 0; yNoise = 0; tNoise = 0.000015

		# if(i > 359 and i < 454):
		# 	xNoise = 0; yNoise = 0; tNoise = 0.0008
		# else:
		# 	xNoise = 0; yNoise = 0; tNoise = 0


		del_xN = del_x + xNoise; del_yN = del_y + yNoise; del_thetaN = del_theta + tNoise

		# Convert to T2_1'
		T2_1N = np.array([[math.cos(del_thetaN), -math.sin(del_thetaN), del_xN], [math.sin(del_thetaN), math.cos(del_thetaN), del_yN], [0, 0, 1]])

		# Get T2_w' = T1_w' . T2_1'
		p1 = (xN[i-1], yN[i-1], tN[i-1])
		T1_wN = np.array([[math.cos(p1[2]), -math.sin(p1[2]), p1[0]], [math.sin(p1[2]), math.cos(p1[2]), p1[1]], [0, 0, 1]])
		T2_wN = np.dot(T1_wN, T2_1N)
		
		# Get x2', y2', theta2'
		x2N = T2_wN[0][2]
		y2N = T2_wN[1][2]
		theta2N = math.atan2(T2_wN[1, 0], T2_wN[0, 0])

		xN[i] = x2N; yN[i] = y2N; tN[i] = theta2N

	# tN = getTheta(xN, yN)

	return (xN, yN, tN)


def writeG2O(X, Y, THETA, file):
	g2o = open(file, 'w')

	for i, (x, y, theta) in enumerate(zip(X, Y, THETA)):
		line = "VERTEX_SE2 " + str(i) + " " + str(x) + " " + str(y) + " " + str(theta)
		g2o.write(line)
		g2o.write("\n")

	# Odometry
	# T1_w : 1 with respect to world
	g2o.write("# Odometry constraints\n")
	info_mat = "500.0 0.0 0.0 500.0 0.0 500.0"
	for i in range(1, len(X)):
		p1 = (X[i-1], Y[i-1], THETA[i-1])
		p2 = (X[i], Y[i], THETA[i])
		T1_w = np.array([[math.cos(p1[2]), -math.sin(p1[2]), p1[0]], [math.sin(p1[2]), math.cos(p1[2]), p1[1]], [0, 0, 1]])
		T2_w = np.array([[math.cos(p2[2]), -math.sin(p2[2]), p2[0]], [math.sin(p2[2]), math.cos(p2[2]), p2[1]], [0, 0, 1]])
		T2_1 = np.dot(np.linalg.inv(T1_w), T2_w)
		del_x = str(T2_1[0][2])
		del_y = str(T2_1[1][2])
		del_theta = str(math.atan2(T2_1[1, 0], T2_1[0, 0]))

		line = "EDGE_SE2 "+str(i-1)+" "+str(i)+" "+del_x+" "+del_y+" "+del_theta+" "+info_mat+"\n"
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

	X, Y, THETA = readPose(argv[1])
	draw(X, Y, THETA)
	# writeG2O(X, Y, THETA, os.path.join(dirc, "gt.g2o"))


	(xN, yN, tN) = addNoise(X, Y, THETA)
	draw(xN, yN, tN)
	writeG2O(xN, yN, tN, os.path.join(dirc, "noise.g2o"))

	
	# (xOpt, yOpt, tOpt) = readG2o(os.path.join(dirc, "gt.g2o"))
	# draw(xOpt, yOpt, tOpt)
