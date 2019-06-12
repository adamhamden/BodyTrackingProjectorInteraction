#!/usr/bin/env python
from __future__ import print_function

import roslib
import sys
import rospy
import numpy as np
import time 

from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats

from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import random 

sys.path.append('../modules/')
import ransac
import plane_processor as pt

class Queue:

    def __init__(self, max_length=100, type_check_function=None):
        self.max_length = max_length
        self.data = []

        if type_check_function is None:
            self.type_check_function = lambda item: True
        else:
            self.type_check_function = type_check_function

    def add(self, *items):
        for item in items:
            if not self.type_check_function(item):
                raise ValueError("Item didn't pass specified test: " + str(item))
        self.data += items
        num_to_pop = max(0, len(self.data) - self.max_length)
        self.data = self.data[num_to_pop:]

    def get(self):
        return self.data

    def __call__(self, *args, **kwargs):
        return self.get()

queue_x = Queue(1)
queue_y = Queue(1)

fig = plt.figure()
plt.xlim(0,854)
plt.ylim(0, 480)
plt.axis('off')
plt.subplots_adjust(left=0, right=1, top=1, bottom=0)
curve = plt.scatter(0,0)
queue_x.add(0)
queue_y.add(0)

def points_callback(data):
	global curve
	curve.remove()
 	pixel_x = int(data.data[0])
	pixel_y = int(data.data[1])
	queue_x.add(pixel_x)
	queue_y.add(pixel_y)
	s = [2000*4**n for n in range(len(queue_x.get()))]

	curve = plt.scatter(queue_x.get(),queue_y.get(), s=s)
	plt.show()

rospy.init_node('point_grapher', anonymous=True)
point_sub = rospy.Subscriber("/touch_points",numpy_msg(Floats),points_callback)
plt.ion()
plt.show(block=True)
