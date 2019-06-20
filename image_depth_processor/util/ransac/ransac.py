import numpy as np
import random 
import plane_processor as pt

#reference: http://www.cse.psu.edu/~rtc12/CSE486/lecture15.pdf
#reference: http://www.ipb.uni-bonn.de/pdfs/Yang2010Plane.pdf


def ransac(list_of_points, iterations):
	list_of_pts_captured = []
	best_plane = np.array([0,0,0,0])
	max_points_captured = 0
	
	for i in range (0, iterations):

		points_captured = 0

		three_point_list = np.array(random.sample(set(list_of_points),3))
		point_1 = three_point_list[0]
		point_2 = three_point_list[1]
		point_3 = three_point_list[2]

		plane = pt.plane_finder(point_1, point_2, point_3)
		print(plane)

		for p in range(0, len(list_of_points)):
			if pt.distance_to_plane(plane, list_of_points[p] ) < .05:
				points_captured += 1
		
		list_of_pts_captured.append(points_captured)	
		if points_captured > max_points_captured:
			max_points_captured = points_captured
			best_plane = plane
		
                matches = 0		
		for val in list_of_pts_captured:
			if abs((max_points_captured - val)/max_points_captured) < .1:
				matches = matches + 1

		if matches >= 10:
			break

	return best_plane
