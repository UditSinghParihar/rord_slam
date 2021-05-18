import matplotlib.pyplot as plt
from sys import argv, exit
import math
import numpy as np
import os


def draw(X, Y, THETA):
	ax = plt.subplot(111)
	ax.plot(X, Y, 'ro')
	ax.plot(X, Y, 'k-')

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

	ax.set_aspect('equal', 'datalim')
	ax.margins(0.1)
	
	plt.show()


def drawLC(X, Y, THETA, srcs, trgs):
	ax = plt.subplot(111)
	ax.plot(X, Y, 'ro')
	ax.plot(X, Y, 'k-')

	for src, trg in zip(srcs, trgs):
		ax.plot([X[src], X[trg]], [Y[src], Y[trg]], 'b--', markersize=10)
		ax.plot([X[src], X[trg]], [Y[src], Y[trg]], 'bo', markersize=5)

	ax.set_aspect('equal', 'datalim')
	ax.margins(0.1)

	plt.show()


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


def readLC(filename):
	f = open(filename, 'r')
	A = f.readlines()
	f.close()

	src, trg, trans = [], [], []

	for i, line in enumerate(A):
		if(i%2 == 0):
			st, end = line.split(' ')
			src.append(int(st)); trg.append(int(end.rstrip('\n')))
		else:
			tran = line.split(' ')
			theta = math.radians(float(tran[2].rstrip('\n')))
			trans.append((float(tran[0]), float(tran[1]), theta))

	return src, trg, trans


def writeG2O(X, Y, THETA, src, trg, trans):
	g2o = open(os.path.join(dirc, "noise_lc.g2o"), 'w')

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

	# LC Constraints
	g2o.write('# Loop Closure constraints\n')
	info_mat = "700.0 0.0 0.0 700.0 0.0 700.0\n"
	for i in range(len(src)):
		del_x, del_y, del_theta = str(trans[i][0]), str(trans[i][1]), str(trans[i][2])
		line = "EDGE_SE2 "+str(src[i])+" "+str(trg[i])+" "+del_x+" "+del_y+" "+del_theta+" "+info_mat
		# print(line)
		g2o.write(line)

	g2o.write("FIX 0\n")
	g2o.close()


def optimize():
	cmd = "g2o -robustKernel Cauchy -robustKernelWidth 1 -o {} -i 50 {} > /dev/null 2>&1".format(
		os.path.join(dirc, "opt.g2o"), 
		os.path.join(dirc, "noise_lc.g2o"))
	os.system(cmd)


if __name__ == '__main__':
	dirc = os.path.dirname(argv[1])

	X, Y, THETA = readG2o(argv[1])
	# draw(X, Y, THETA)

	src, trg, trans = readLC(argv[2])
	drawLC(X, Y, THETA, src, trg)
	# exit(1)

	writeG2O(X, Y, THETA, src, trg, trans)

	optimize()
	(xOpt, yOpt, tOpt) = readG2o(os.path.join(dirc, "opt.g2o"))
	# draw(xOpt, yOpt, tOpt)
	drawLC(xOpt, yOpt, tOpt, src, trg)
