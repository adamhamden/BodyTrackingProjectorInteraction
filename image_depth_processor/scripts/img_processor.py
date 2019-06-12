#!/usr/bin/env python
from __future__ import print_function

import roslib
import sys
import rospy
import cv2

from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

from matplotlib import pyplot as plt
import cv2.xfeatures2d

is_img_captured = False
is_homography_captured = False
homography = 0
img1 = 0
img2 = 0

class image_converter:

  def __init__(self):

    self.screen_pub = rospy.Publisher("on_screen_touch_point",String, queue_size = 100)
    rospy.init_node('image_proc', anonymous=True)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/camera/color/image",Image,self.callback)
    self.touch_point_sub = rospy.Subscriber("/touch_points_proj", String, self.touch_callback)

  def callback(self,data):
    global is_homography_captured
    global is_img_captured
    global img1
    global homography
    global img2

    if not is_img_captured:

      try:
        img1 = self.bridge.imgmsg_to_cv2(data, "rgb8")
      except CvBridgeError as e:
        print(e)

      is_img_captured = True
      print(img1.shape)
      print("got image one")
      MIN_MATCH_COUNT = 10
      plt.imshow(img1), plt.show()

      img2 = cv2.imread('../images/img2.png') 
      img2 = cv2.cvtColor(img2, cv2.COLOR_BGR2RGB)
      print(img2.shape)
      plt.imshow(img2), plt.show()
      print("got image two")

      # Initiate SIFT detector
      sift = cv2.xfeatures2d.SIFT_create()

      # find the keypoints and descriptors with SIFT
      kp1, des1 = sift.detectAndCompute(img1, None)
      kp2, des2 = sift.detectAndCompute(img2, None)
      print("ran sift")
      FLANN_INDEX_KDTREE = 0
      index_params = dict(algorithm=FLANN_INDEX_KDTREE, trees=5)
      search_params = dict(checks=50)

      flann = cv2.FlannBasedMatcher(index_params, search_params)
      matches = flann.knnMatch(des1, des2, k=2)
      print("ran flann")

      # store all the good matches as per Lowe's ratio test.
      good = []
      for m, n in matches:
        if m.distance < 0.7 * n.distance:
          good.append(m)
      print("found good matches")

      if len(good) > MIN_MATCH_COUNT:
        
        src_pts = np.float32([kp1[m.queryIdx].pt for m in good]).reshape(-1, 1, 2)
        dst_pts = np.float32([kp2[m.trainIdx].pt for m in good]).reshape(-1, 1, 2)

        M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)
	is_homography_captured = True
        homography = M
        matchesMask = mask.ravel().tolist()

        h, w = img1.shape[:-1]
        pts = np.float32([[0, 0], [0, h - 1], [w - 1, h - 1], [w - 1, 0]]).reshape(-1, 1, 2)
	dst = cv2.perspectiveTransform(pts, M)
        img2 = cv2.polylines(img2, [np.int32(dst)], True, 255, 3, cv2.LINE_AA)
        draw_params = dict(matchColor=(0, 255, 0),  # draw matches in green color
                           singlePointColor=None,
                           matchesMask=matchesMask,  # draw only inliers
                           flags=2)

        img3 = cv2.drawMatches(img1, kp1, img2, kp2, good, None, **draw_params)
	print(M)
        plt.imshow(img3), plt.show()
      print("images processed")

  def touch_callback(self, data):

    global homography
    global is_homography_captured
    global img1
    global img2
    print (data.data)

    touch_point = data.data.split()
    print(touch_point)

    if is_homography_captured:
	img1 = cv2.circle(img1, (int(float(touch_point[0])),int(float(touch_point[1]))), 10, (0,0,255), -1)
	x_pos = float(touch_point[0])
	y_pos = float(touch_point[1])
	point = [x_pos, y_pos]
	point_transformed = cv2.perspectiveTransform(np.float32(point).reshape(-1,1,2),homography) 
	img2 = cv2.circle(img2, (int(point_transformed[0][0][0]),int(point_transformed[0][0][1])), 10, (0,0,255), -1)

	vis = np.zeros((1056,2560,3), np.uint8)
	vis[:480, :640,:3]=img1
	vis[:1056,640:2560,:3]=img2
        plt.imshow(vis),plt.show()
        plt.imshow(img2), plt.show()

	on_screen_point_str = str(int(point_transformed[0][0][0])) + " " + str(int(point_transformed[0][0][1])) 
	print(on_screen_point_str)	
	self.screen_pub.publish(on_screen_point_str)

ic = image_converter()
try:
  rospy.spin()
except KeyboardInterrupt:
  print("Shutting down")
cv2.destroyAllWindows()
