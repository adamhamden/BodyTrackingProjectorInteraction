#!/usr/bin/env python3

#get plane equation 
#generate random points
#put some noise 
#look at outcome 
import numpy as np
import random

import ransac 


test_plane = np.array([1, 1, 1, 0])
x= np.arange(-5,5,.01)
y= np.arange(-5,5,.01)
z = []
xx, yy = np.meshgrid(x, y)
zz = ((-test_plane[0] * xx - test_plane[1] * yy -test_plane[3])/(test_plane[2]))

point_list = []
for i in range(50,100):
	for j in range(50,100):
		p = (((-test_plane[0] * x[i] - test_plane[1] * y[j] -test_plane[3])/(test_plane[2])))
		point_list.append((x[i], y[j], p))

for i in range(0,1000):
	point_list.append((5*random.random(), random.random(), random.random()))
best_plane = ransac.ransac(point_list, 100)
print(best_plane)

