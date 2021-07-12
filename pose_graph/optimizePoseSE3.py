import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
from sys import argv, exit
import math
import numpy as np
import os
from scipy.spatial.transform import Rotation as R


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


def drawLC(X, Y, Z, srcs, trgs):
	fig = plt.figure()
	ax = plt.axes(projection='3d')

	ax.scatter3D(X, Y, Z, c=Z, cmap='Greens', marker='o', s=5)

	for src, trg in zip(srcs, trgs):
		ax.scatter3D(X[src], Y[src], Z[src], c='r', marker='o', s=20)
		ax.scatter3D(X[trg], Y[trg], Z[trg], c='r', marker='o', s=20)

		xline = np.linspace(X[src], X[trg], 1000)
		yline = np.linspace(Y[src], Y[trg], 1000)
		zline = np.linspace(Z[src], Z[trg], 1000)

		ax.plot3D(xline, yline, zline, c='r')

	ax.set_aspect('equal', 'datalim')
	ax.margins(0.1)

	plt.show()


def readG2o(fileName):
	f = open(fileName, 'r')
	A = f.readlines()
	f.close()

	X = []
	Y = []
	Z = []
	Qx = []
	Qy = []
	Qz = []
	Qw = []

	for line in A:
		if "VERTEX_SE3:QUAT" in line:			
			if(len(line.split(' ')) == 10):
				(ver, ind, x, y, z, qx, qy, qz, qw, newline) = line.split(' ')
			elif(len(line.split(' ')) == 9):
				(ver, ind, x, y, z, qx, qy, qz, qw) = line.split(' ')

			X.append(float(x))
			Y.append(float(y))
			Z.append(float(z))
			Qx.append(float(qx))
			Qy.append(float(qy))
			Qz.append(float(qz))

			if(len(line.split(' ')) == 10):
				Qw.append(float(qw))
			elif(len(line.split(' ')) == 9):
				Qw.append(float(qw.rstrip('\n')))

	return (X, Y, Z, Qx, Qy, Qz, Qw)


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
			x, y, z, qx, qy, qz, qw = line.split(' ')
			trans.append((float(x), float(y), float(z), float(qx), float(qy), float(qz), float(qw.rstrip('\n'))))

			# theta = math.radians(float(tran[2].rstrip('\n')))
			# trans.append((float(tran[0]), float(tran[1]), theta))

	return src, trg, trans


def writeG2O(X, Y, Z, Qx, Qy, Qz, Qw, src, trg, trans):
	g2o = open(os.path.join(dirc, "noise_lc.g2o"), 'w')

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

	# for i, (x, y, theta) in enumerate(zip(X, Y, THETA)):
	# 	line = "VERTEX_SE2 " + str(i) + " " + str(x) + " " + str(y) + " " + str(theta)
	# 	g2o.write(line)
	# 	g2o.write("\n")	

	# # Odometry
	# # T1_w : 1 with respect to world
	# g2o.write("# Odometry constraints\n")
	# info_mat = "500.0 0.0 0.0 500.0 0.0 500.0"
	# for i in range(1, len(X)):
	# 	p1 = (X[i-1], Y[i-1], THETA[i-1])
	# 	p2 = (X[i], Y[i], THETA[i])
	# 	T1_w = np.array([[math.cos(p1[2]), -math.sin(p1[2]), p1[0]], [math.sin(p1[2]), math.cos(p1[2]), p1[1]], [0, 0, 1]])
	# 	T2_w = np.array([[math.cos(p2[2]), -math.sin(p2[2]), p2[0]], [math.sin(p2[2]), math.cos(p2[2]), p2[1]], [0, 0, 1]])
	# 	T2_1 = np.dot(np.linalg.inv(T1_w), T2_w)
	# 	del_x = str(T2_1[0][2])
	# 	del_y = str(T2_1[1][2])
	# 	del_theta = str(math.atan2(T2_1[1, 0], T2_1[0, 0]))

	# 	line = "EDGE_SE2 "+str(i-1)+" "+str(i)+" "+del_x+" "+del_y+" "+del_theta+" "+info_mat+"\n"
	# 	g2o.write(line)

	# LC Constraints
	g2o.write("\n\n\n# Loop constraints\n\n\n\n")
	info = '40 0 0 0 0 0 40 0 0 0 0 40 0 0 0 40 0 0 40 0 40'

	for i in range(len(src)):
		line = "EDGE_SE3:QUAT " + str(trg[i]) + sp + str(src[i]) + sp + str(trans[i][0]) + sp + str(trans[i][1]) + sp + str(trans[i][2]) + sp + str(trans[i][3]) + sp + str(trans[i][4]) + sp + \
		str(trans[i][5]) + sp + str(trans[i][6]) + sp +  info + '\n'
		g2o.write(line)

	# g2o.write('# Loop Closure constraints\n')
	# info_mat = "700.0 0.0 0.0 700.0 0.0 700.0\n"
	# for i in range(len(src)):
	# 	del_x, del_y, del_theta = str(trans[i][0]), str(trans[i][1]), str(trans[i][2])
	# 	line = "EDGE_SE2 "+str(src[i])+" "+str(trg[i])+" "+del_x+" "+del_y+" "+del_theta+" "+info_mat
	# 	# print(line)
	# 	g2o.write(line)

	g2o.write("FIX 0\n")
	g2o.close()


def optimize():
	cmd = "g2o -robustKernel Cauchy -robustKernelWidth 1 -o {} -i 50 {} > /dev/null 2>&1".format(
		os.path.join(dirc, "opt.g2o"), 
		os.path.join(dirc, "noise_lc.g2o"))
	os.system(cmd)


if __name__ == '__main__':
	dirc = os.path.dirname(argv[1])

	X, Y, Z, Qx, Qy, Qz, Qw = readG2o(argv[1])

	XGt, YGt, ZGt, QxGt, QyGt, QzGt, QwGt = readG2o(argv[3])	
	# draw(X, Y, Z)

	drawTwo(XGt, YGt, ZGt, X, Y, Z)

	src, trg, trans = readLC(argv[2])
	drawLC(X, Y, Z, src, trg)

	writeG2O(X, Y, Z, Qx, Qy, Qz, Qw, src, trg, trans)

	optimize()
	XOpt, YOpt, ZOpt, QxOpt, QyOpt, QzOpt, QwOpt = readG2o(os.path.join(dirc, "opt.g2o"))

	# draw(xOpt, yOpt, tOpt)
	# drawLC(XOpt, YOpt, ZOpt, src, trg)

	drawTwo(XGt, YGt, ZGt, XOpt, YOpt, ZOpt)
	# drawTwo(X, Y, Z, XOpt, YOpt, ZOpt)