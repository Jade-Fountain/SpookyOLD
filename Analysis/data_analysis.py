'''
==============
3D scatterplot
==============

Demonstration of a basic scatterplot in 3D.
'''

from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import numpy as np
import math
from numpy import genfromtxt

# source:http://jekel.me/2015/Least-Squares-Sphere-Fit/
#	fit a sphere to X,Y, and Z data points
#	returns the radius and center points of
#	the best fit sphere
def sphereFit(spX,spY,spZ):
	#   Assemble the A matrix
	spX = np.array(spX)
	spY = np.array(spY)
	spZ = np.array(spZ)
	A = np.zeros((len(spX),4))
	A[:,0] = spX*2
	A[:,1] = spY*2
	A[:,2] = spZ*2
	A[:,3] = 1
	
	#   Assemble the f matrix
	f = np.zeros((len(spX),1))
	f[:,0] = (spX*spX) + (spY*spY) + (spZ*spZ)
	C, residules, rank, singval = np.linalg.lstsq(A,f)

	#   solve for the radius
	t = (C[0]*C[0])+(C[1]*C[1])+(C[2]*C[2])+C[3]
	radius = math.sqrt(t)
	
	return radius, [C[0], C[1], C[2]]


def drawSphere(c, r, ax, colour):
	ax.scatter(c[0], c[1], c[2],c = colour)
	u, v = np.mgrid[0:2*np.pi:20j, 0:np.pi:10j]
	x = np.cos(u)*np.sin(v) * r + c[0]
	y = np.sin(u)*np.sin(v) * r + c[1]
	z = np.cos(v) * r + c[2]
	ax.plot_wireframe(x, y, z, color= colour)


my_data = genfromtxt('Spericaldata.csv')

# print my_data

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

n = 100

# For each set of style and range settings, plot n random points in the box
# defined by x in [23, 32], y in [0, 100], z in [zlow, zhigh].
for i in range(my_data.shape[0]):
	ax.scatter(my_data[i][0], my_data[i][1], my_data[i][2],c = 'r')

# #draw mean
mean = np.mean(my_data, axis = 0)
ax.scatter(mean[0], mean[1], mean[2],c = 'b')

# draw sphere and center
center = ( -0.0387789, 0.00754286,  0.0250226)
radius = 0.2498
drawSphere(center, radius, ax, 'k')

# Draw sphere fitted with least squares
pysphere_r, pysphere_c = sphereFit(my_data[:][0],my_data[:][1],my_data[:][2])
# drawSphere(pysphere_c, pysphere_r, ax, 'g')

# Labels
ax.set_xlabel('X Label')
ax.set_ylabel('Y Label')
ax.set_zlabel('Z Label')

# Make histogram of errors
offset_data = my_data[:,0:3] - center
norms = np.linalg.norm(offset_data, axis=1)
fig = plt.figure()
plt.hist(norms, bins = 100)


plt.show()
