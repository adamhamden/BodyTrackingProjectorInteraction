#!/usr/bin/env python3
import pyautogui
import roslib
import sys
import rospy

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
