#!/usr/bin/env python
from __future__ import print_function

import roslib
import sys
import rospy
import cv2

from std_msgs.msg import String
from sensor_msgs.msg import Image
from sensor_msgs import point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField
from body_tracker_msgs.msg import BodyTrackerArray, BodyTracker, Skeleton

from cv_bridge import CvBridge, CvBridgeError
import numpy as np

from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import random 
import math
import time 

sys.path.append('../modules/')
import ransac
import plane_processor as pt

best_plane = np.array([0,0,0,0])
is_best_plane_found = False

SINK_RUN_TIME = 30
SOURCE_RUN_TIME = 30
VAL_TO_CAPTURE = 1
counter = 0

def callback_optimizer(data):

	global SINK_RUN_TIME
	global SOURCE_RUN_TIME
	global VAL_TO_CAPTURE
	global counter

	VAL_TO_CAPTURE = (1./SINK_RUN_TIME) / (1./SOURCE_RUN_TIME)
	VAL_TO_CAPTURE *= 1.05
	VAL_TO_CAPTURE = int(math.ceil(VAL_TO_CAPTURE))
	
	counter += 1

	if counter >= VAL_TO_CAPTURE:
		counter = 0
		t = time.time()
		skeleton_callback(data)
		elapsed = time.time() - t
		SINK_RUN_TIME = 1/elapsed

def graph_best_plane(best_plane, points):

	fig = plt.figure()
	ax = fig.add_subplot(111, projection='3d')
	ax.scatter(points[0], points[1], points[2], c='r', marker='o')
	x= np.arange(-2,4,1)
	y= np.arange(-2,4,1)
	xx, yy = np.meshgrid(x, y)
	zz = ((-best_plane[0] * xx - best_plane[1] * yy -best_plane[3])/(best_plane[2]))
	ax.plot_surface(xx, yy, zz, alpha=0.2)
	plt.show()

def plot_pixel(pixel_x, pixel_y):

	global curve
 	pixel_x = int(pixel_x)
	pixel_y = int(pixel_y)
	curve = plt.scatter(pixel_x,pixel_y)
	plt.show()

def pcl_callback(data):
	
	global best_plane
	global is_best_plane_found
	
	if not is_best_plane_found:
		print("Best plane found")
		is_best_plane_found = True		
		cloud_points = list(pc2.read_points(data, field_names = ("x", "y", "z"), skip_nans=True))

		length_of_sublist = int(.0015*len(cloud_points))
		cloud_points_sublist = random.sample(set(cloud_points), length_of_sublist)
		best_plane = ransac.ransac(cloud_points_sublist, 15)

def skeleton_callback(data):
	global is_best_plane_found
	if is_best_plane_found:

		x = data.joint_position_left_hand_real.x
		y = data.joint_position_left_hand_real.y
		z = data.joint_position_left_hand_real.z
		point = np.array([x,y,z])

	
		if pt.distance_to_plane(best_plane, point) < .5:

			print("contact")
			touch_point_str = str(640*data.joint_position_left_hand_proj.x) +" "+ str(480*data.joint_position_left_hand_proj.y)
			print(touch_point_str)
	 		point_pub.publish(touch_point_str)

rospy.init_node('pcl_proc', anonymous=True)
depth_sub = rospy.Subscriber("/camera/depth_cloud",PointCloud2,pcl_callback)
skeleton_sub = rospy.Subscriber("/body_tracker/skeleton", Skeleton, callback_optimizer)
point_pub = rospy.Publisher("touch_points_proj", String, queue_size=100)
rospy.spin()
