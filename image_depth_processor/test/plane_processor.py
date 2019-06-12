import math
import numpy as np

def distance_finder(point_1, point_2):

    distance = np.linalg.norm(point_1-point_2)
    return distance

def normal_finder(vector_1, vector_2):

    normal_vector = np.cross(vector_1, vector_2)
    return normal_vector

def vector_finder(point_1, point_2):

    vector_1 = np.subtract(point_1,point_2)
    return vector_1

def plane_finder(point_1, point_2, point_3):

    vect_1 = vector_finder(point_1,point_2)
    vect_2 = vector_finder(point_1,point_3)
    vect_3 = vector_finder(point_2, point_3)
    try:
    	if np.array_equal(vect_1, np.array([0,0,0])) or np.array_equal(vect_2, np.array([0,0,0])) or np.array_equal(vect_3, np.array([0,0,0])):
    		raise ValueError('All points must be unique!')
	
    except:
	return 
   
    pt1_unit =  np.around(np.absolute(point_1/np.linalg.norm(point_1)), decimals=5)
    pt2_unit =  np.around(np.absolute(point_2/np.linalg.norm(point_2)), decimals=5)
    pt3_unit =  np.around(np.absolute(point_3/np.linalg.norm(point_3)), decimals=5)

    try:
	if np.array_equal(pt1_unit,pt2_unit) or np.array_equal(pt1_unit,pt3_unit) or np.array_equal(pt3_unit,pt2_unit):
		raise ValueError('Points cannot be on the same line!')

    except:
	return 
    normal_vector = normal_finder(vect_1, vect_2)

    a_coe = normal_vector[0]
    b_coe = normal_vector[1]
    c_coe = normal_vector[2]
    d_coe = 0

    for i in range(0,3):
        d_coe -= point_1[i]*normal_vector[i]
    
    plane = np.array([a_coe, b_coe, c_coe, d_coe])

    return plane

def distance_to_plane(plane, point_1):

	plane_norm = np.linalg.norm(plane[:3])

	mag = plane[3]
	for i in range(0,3):
		mag += plane[i]*point_1[i]

	mag = abs(mag)
	distance = mag / plane_norm

	return distance

def closest_point_to_plane(plane, sensor):

	plane_norm_sq = math.pow(np.linalg.norm(plane[:3]),2)

	mag = plane[3]
	for i in range(0,3):
		mag += plane[i]*sensor[i]

	c = (-1*mag/plane_norm_sq)

	closest_point = []

	for i in range (0,3):
		closest_point.append(sensor[i]+plane[i]*c)

	return np.array(closest_point)

def sphere_closest_point(sensor, center, radius):

	vector = vector_finder(sensor, center)

	unit_vector = vector / np.linalg.norm(vector)

	sized_vector = radius * unit_vector

	return sized_vector


def recomend_sphere(sensor, center, radius):

    distance = distance_finder(sensor, center)
    delta = vector_finder(sensor, center)

    rec_string = ""

    if distance > radius:

        delt = vect_finder(sensor, center)

        if delta[0] > 0:
            rec_string += "move right "
        elif delta[0] < 0: 
            rec_string += "move left "
        else:
            rec_string += "stay still in x "

        if delta[1] > 0:
            rec_string += "move forward "
        elif delta[1] < 0: 
            rec_string += "move back "
        else:
            rec_string += "stay still in y "

        if delta[2] > 0:
            rec_string += "move down "
        elif delta[2] < 0: 
            rec_string += "move up "
        else:
            rec_string += "stay still in z "

    else:
        rec_string += "You are in range!"

    return rec_string

