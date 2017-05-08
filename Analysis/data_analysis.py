'''
==============
3D scatterplot
==============

Demonstration of a basic scatterplot in 3D.
'''

from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import numpy as np
from numpy import genfromtxt

def drawSphere(c, r, ax):
	ax.scatter(c[0], c[1], c[2],c = 'k')
	u, v = np.mgrid[0:2*np.pi:20j, 0:np.pi:10j]
	x = np.cos(u)*np.sin(v) * r + c[0]
	y = np.sin(u)*np.sin(v) * r + c[1]
	z = np.cos(v) * r + c[2]
	ax.plot_wireframe(x, y, z, color="k")


my_data = genfromtxt('Spericaldata.csv', delimiter=',')

print my_data

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

n = 100

# For each set of style and range settings, plot n random points in the box
# defined by x in [23, 32], y in [0, 100], z in [zlow, zhigh].
for i in range(my_data.shape[0] / 3):
    ax.scatter(my_data[i][0], my_data[i][1], my_data[i][2],c = 'r')


# draw sphere and center
mean = (-0.0484143, 0.0625665,  0.201494)
center = (0.0917047, 0.147517, 0.156683)
radius = 0.14023
drawSphere(center, radius, ax)
#draw mean
ax.scatter(mean[0], mean[1], mean[2],c = 'b')


# Labels
ax.set_xlabel('X Label')
ax.set_ylabel('Y Label')
ax.set_zlabel('Z Label')

plt.show()
