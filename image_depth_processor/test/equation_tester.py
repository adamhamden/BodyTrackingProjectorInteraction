#!/usr/bin/env python

import unittest
import math
import numpy as np
import time 
import sys

sys.path.append('../modules/')
import plane_processor as file

def arrayEqual(return_val, expected):
	x= return_val
	y= expected
	if x is not None and y is not None:
		x = np.around(np.absolute(return_val/np.linalg.norm(return_val)), decimals=5)
		y = np.around(np.absolute(expected/np.linalg.norm(expected)), decimals=5)
	return np.array_equal(x,y)

points=[
np.array([1,1,-2]),
np.array([2,3,-5]),
np.array([-2,-4,6]),
np.array([1,1,-1]),
np.array([3,4,-6]),
np.array([-2,-5,8]),
np.array([1,-10,-123]),
np.array([1,7,2]),
np.array([1,5,6]),
np.array([2,0,3]),
np.array([3,545,4]),
np.array([5,5,6])
]

planes=[
np.array([1,1,1,0]),
np.array([1,1,1,-1]),
np.array([1,0,0,-1]),
np.array([1,0,-1,1])
]

class TestPlane(unittest.TestCase):
	
	def test_good_points_no_exception(self):

		self.assertTrue(arrayEqual(file.plane_finder(points[0],points[1],points[2]),planes[0]))
		self.assertTrue(arrayEqual(file.plane_finder(points[3],points[4],points[5]),planes[1]))
		self.assertTrue(arrayEqual(file.plane_finder(points[6],points[7],points[8]),planes[2]))
		self.assertTrue(arrayEqual(file.plane_finder(points[9],points[10],points[11]),planes[3]))

	def test_same_points_raise_exception(self):

		self.assertRaises(ValueError, file.plane_finder(points[0], points[0], points[3]))
		self.assertRaises(ValueError, file.plane_finder(points[0], points[3], points[3]))			   
                self.assertRaises(ValueError, file.plane_finder(points[3], points[0], points[3]))		      
                self.assertRaises(ValueError, file.plane_finder(points[0], points[0], points[0]))

	def test_colinear_points_raise_exception(self):

		self.assertRaises(ValueError, file.plane_finder(points[0], 2*points[0], points[3]))
		self.assertRaises(ValueError, file.plane_finder(points[0], 2*points[0], 4*points[0]))
		self.assertRaises(ValueError, file.plane_finder(points[0], points[3], 2*points[0]))
		self.assertRaises(ValueError, file.plane_finder(points[3], points[0], -1*points[3]))
	
	def test_distance_to_plane(self):

		self.assertEqual(file.distance_to_plane(planes[0], points[0]), 0)
		self.assertEqual(file.distance_to_plane(planes[2], points[1]), 1)
		self.assertEqual(file.distance_to_plane(planes[2], points[11]), 4)
		self.assertEqual(file.distance_to_plane(planes[2], points[10]), 2)

	def test_closest_point_to_plane(self):

		for pt in points:
			for pl in planes:
				self.assertAlmostEqual(file.distance_to_plane(pl, pt), file.distance_finder(pt, file.closest_point_to_plane(pl, pt)), places=5)

centers=[
np.array([0,0,0]),
np.array([0.5,-1,0]),
np.array([-1,-1,-1]),
np.array([1,1,1]),
np.array([-1,-2,2]),
np.array([1,2,-2]),
np.array([0.1,0.1,-1]),
]

radius=[
1,
2,
4,
7,
.3,
2.5,
4.44,
8.0
]

points_=[
np.array([10,10,10]),
np.array([11.2,2,3]),
np.array([3,-10,1.33]),
np.array([1.0,86.0,13.4]),
np.array([-10,-10,-10]),
np.array([0.33,22.3,55.322]),
np.array([4.132,-1.2434,2.2]),
np.array([3.14,1.59,2.65]),

]

class TestSphere(unittest.TestCase):
	def test_sphere_closest_point(self):
		for center in centers:
			for point in points_:
				for r in radius:
					self.assertAlmostEqual(file.distance_finder(file.sphere_closest_point(point, center, r), point), abs(file.distance_finder(point, center)-r),places=5)

if __name__ == '__main__':
	unittest.main()
