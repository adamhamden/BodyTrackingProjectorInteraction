#!/usr/bin/env python3
import time

import pyautogui


import roslib
import sys
import rospy

from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import time 
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats

from sensor_msgs import point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from body_tracker_msgs.msg import BodyTrackerArray, BodyTracker, Skeleton

from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import random 

import math

from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats

def points_callback(data):
	touch_point = data.data.split()
	x_pos = int(touch_point[0])
	y_pos = int(touch_point[1])
	if 0 <= x_pos <= 1920 and 0 <= y_pos <= 1056:
		print(x_pos, y_pos)
		pyautogui.moveTo(x_pos, y_pos)

rospy.init_node('point_grapher', anonymous=True)
point_sub = rospy.Subscriber("/on_screen_touch_point",String,points_callback)
rospy.spin()
