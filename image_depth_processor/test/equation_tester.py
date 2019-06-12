#!/usr/bin/env python

import unittest
import math
import numpy as np
import time 
import sys

sys.path.append('../modules/')
import plane_processor as file


test_count = 0

def assertEqual(return_val, expected):
	global test_count	
	test_count += 1
	x= return_val
	y= expected
	if x is not None and y is not None:
		x = np.around(np.absolute(return_val/np.linalg.norm(return_val)), decimals=5)
		y = np.around(np.absolute(expected/np.linalg.norm(expected)), decimals=5)
	return np.array_equal(x,y)

def test_valid_plane_finder():

	global test_count	
	print("----------------------------------------------------------------------")

	check = True
	test_count = 0
	t = time.time()

	check *= assertEqual(file.plane_finder(np.array([1,1,-2]),np.array([2,3,-5]),np.array([-2,-4,6])),np.array([1,1,1,0]))
	check *= assertEqual(file.plane_finder(np.array([1,1,-1]),np.array([3,4,-6]),np.array([-2,-5,8])),np.array([1,1,1,-1]))
	check *= assertEqual(file.plane_finder(np.array([1,-10,-123]),np.array([1,7,2]),np.array([1,5,6])),np.array([1,0,0,-1]))
	check *= assertEqual(file.plane_finder(np.array([2,0,3]),np.array([3,545,4]),np.array([5,5,6])),np.array([1,0,-1,1]))

	elapsed = time.time() - t
	print("Ran %d test in %fsec" % (test_count, elapsed))
	if not check:
		print("1 or more tests failed")
	else:
		print("All OK")

def test_invalid_plane_finder():

	global test_count	
	print("----------------------------------------------------------------------")

	check = True
	test_count = 0
	t = time.time()

	check *= assertEqual(file.plane_finder(np.array([1,1,1]),np.array([2,2,2]),np.array([-1,0,1])), None)
	check *= assertEqual(file.plane_finder(np.array([2,2,2]),np.array([4,4,4]),np.array([5,5,5])), None)
	check *= assertEqual(file.plane_finder(np.array([1,1,1]),np.array([3,5,6]),np.array([1,1,1])), None)
	check *= assertEqual(file.plane_finder(np.array([1,1,1]),np.array([1,1,1]),np.array([1,1,1])), None)

	elapsed = time.time() - t
	print("Ran %d test in %fsec" % (test_count, elapsed))
	if not check:
		print("1 or more tests failed")
	else:
		print("All OK")


if __name__ == '__main__':
	test_valid_plane_finder()
	test_invalid_plane_finder()
