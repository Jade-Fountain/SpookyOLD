# experiment_analysis.py

from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import numpy as np
import math
from numpy import genfromtxt
from covariance_plot import plot_point_cov

def plotData(folderName, N):
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
	
	plot_point_cov(errors,color='grey')
	plt.plot(errors[:,0], errors[:,1], '.k')
	plt.axis([0,30,0,8])
	plt.plot(mean[0],mean[1], 'rx')
	plt.show()


plotData('CustomMovementVive1',10)

