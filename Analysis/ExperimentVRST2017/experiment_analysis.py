# experiment_analysis.py

from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import numpy as np
import math
from numpy import genfromtxt
from covariance_plot import plot_point_cov

def plotData(folderName, N, point_spec, ellipse_color):
	errors = np.zeros([N,2])
	for i in range(N):
		my_data = genfromtxt(folderName + '/' + str(i+1), delimiter=',')
		errors[i] = my_data.mean(axis=0)

	mean = errors.mean(axis=0)
	print errors
	print "Mean = ", errors.mean(axis=0)
	print "StdDev = ", errors.std(axis=0)
	print "Worst Individual= ", errors.max(axis=0)
	print "Best Individual= ", errors.min(axis=0)
	
	plot_point_cov(errors,color=ellipse_color)
	plt.plot(errors[:,0], errors[:,1], point_spec)
	plt.axis([0,30,0,8])
	plt.plot(mean[0],mean[1], 'rx')

def plotDataUnified(folderName, N, point_spec, ellipse_color):
	errors = np.zeros([0,2])
	for i in range(N):
		my_data = genfromtxt(folderName + '/' + str(i+1), delimiter=',')
		start = 0
		last_value = my_data[0,0]
		for j in range(my_data.shape[0]):
			change = np.linalg.norm(my_data[j,0] - last_value) 
			if(change > 0.2 or j==my_data.shape[0]-1):
				errors = np.append(errors, [my_data[start:j,:].mean(axis=0)], axis=0)
				start = j + 1
				last_value = my_data[j,0]

	mean = errors.mean(axis=0)
	print errors
	print "Mean = ", errors.mean(axis=0)
	print "StdDev = ", errors.std(axis=0)
	print "Worst Individual= ", errors.max(axis=0)
	print "Best Individual= ", errors.min(axis=0)
	
	plot_point_cov(errors,color=ellipse_color)
	plt.plot(errors[:,0], errors[:,1], point_spec)
	plt.axis([0,30,0,8])
	plt.plot(mean[0],mean[1], 'rx')


plotData('CustomMovementVive1',10,point_spec = 'g.', ellipse_color='blue')
plotDataUnified('CustomMovementViveLitMethod1',1, point_spec = 'k.', ellipse_color='grey')
plt.show()

