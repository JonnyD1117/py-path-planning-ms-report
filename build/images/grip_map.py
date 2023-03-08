#!/bin/env python3 
import matplotlib.pyplot as plt 
import numpy as np 
import json 
from math import sin, cos, floor, sqrt, pow, atan2
import math
from scipy.spatial.distance import cdist
from bresenham import bresenham	


def extract_scan_data():
	path = "python_plot/lidar_data.json"
	# path = "python_plot/lidar_data_new.json"

	f = open(path)

	data = json.load(f)

	# Angles and Increments are in radians
	start_angle = data["angle_min"]
	end_angle = data['angle_max']
	angle_increment = data["angle_increment"]

	ranges = data["ranges"]

	min_range = data["range_min"]
	max_range = data['range_max']

	filt_ranges = [] 

	for dist in ranges:

		if dist > max_range or dist < min_range:
			dist = 0

		filt_ranges.append(dist)

	return filt_ranges, (min_range, max_range), (start_angle, end_angle, angle_increment)

def pol2cart(theta, radius):

	x = radius * cos(theta)
	y = radius * sin(theta)

	return (x, y)

def iterate_over_scan():

	meas = []

	ranges, limits, (start_a, end_a, inc_a) = extract_scan_data()
	
	angles = np.arange(start_a, end_a, inc_a)

	measurements = zip(ranges, angles)

	for r, a in measurements:

		meas.append(pol2cart(a, r))


	return meas

def indices2coords(i, j):
	def indices2quarant(i, j):

		half_y = num_grid_y // 2
		half_x = num_grid_x // 2

		if i >= half_x:
			if j <half_y:
				return 3
			else:
				return 4 
		if i < half_x:
			if j <half_y:
				return 2
			else:
				return 1

	quadrant = indices2quarant(i, j)


	if quadrant == 1:

		x = (zero_x_idx ) - i
		y = -(zero_y_idx) + j 

	elif quadrant == 2:
		x = -(zero_x_idx) + i
		y = (zero_y_idx ) - j 

	elif quadrant == 3:
		x = (zero_x_idx ) - i
		y = -(zero_y_idx) + j 

	elif quadrant == 4: 
		x = -(zero_x_idx) + i
		y = (zero_y_idx ) - j 

	return (x, y)

def coords2indices(x, y):


	i = -y + (zero_x_idx )
	j = x  + (zero_y_idx) 

	return (i, j)

def inverse_sensor_model(cur_cell, sensor, pose):

	p_occ = .95
	p_free = .001
	p_0 = .5

	l_occ = math.log(p_occ/(1 - p_occ))
	l_free = math.log(p_free/(1 - p_free))
	l_0 = math.log(p_0/(1 - p_0))

	sen_max = 12    # Meters
	sen_min = 15.0/100.

	x, y = cur_cell
	x_sen = sensor[0]
	y_sen = sensor[1]

	sensor_range = sqrt(pow(x_sen,2) + pow(y_sen ,2))
	x_0, y_0 = pose

	min_tol = .99
	max_tol = 1.01
	
	r = sqrt(pow((x - x_0),2) + pow((y - y_0),2))

	if r >= sensor_range*min_tol and r < sensor_range*max_tol :
		return l_occ
	elif r <= sensor_range and r >=sen_min:
		return l_free
	else:
		return l_0



if __name__ == "__main__":

	cart_meas = []

	p_0 = .5
	l_0 = math.log(p_0/(1 - p_0))

	# Create Occpuancy Grid Map Data Structure
	init_prior = .5

	max_y = 3 # Meters 
	max_x = 3 # Meters

	min_x = -3 
	min_y = -3

	x_res = .01 # Meter Divisions 
	y_res = x_res

	num_grid_x = int((max_x- min_x) * (1 / x_res)) # Total Number of X Grid Cells to create
	num_grid_y = int((max_y - min_y) * (1 / y_res)) # Total Number of Y Grid Cells to create

	print(f"Num Grids = {num_grid_x}")

	zero_x_idx = int(num_grid_x//2.)
	zero_y_idx = int(num_grid_y//2.)

	# Create Numpy array to represent the grid map
	map = np.ones(shape=(num_grid_x+1, num_grid_y+1), dtype=float)*l_0

	# Unpack Scan Data 
	cart_meas = iterate_over_scan()

	# Convert Scan Data Units from Meters to Grid Cells & truncate to discretize the scan data
	x = [int(data[0]*1.0/x_res) for data in cart_meas]
	y = [int(data[1]*1.0/y_res) for data in cart_meas]
	

	conv_data = zip(x,y)
	thing = [] 
	bres_thing = []


	# Iterator Over all of the Scan Data
	for x_sensor, y_sensor in conv_data:
		# For each sensor point, compute the Bresenham set from the origin of the lidar to the measured point
		output = list(bresenham(0,0, x_sensor, y_sensor))

		# x_sen_idx, y_sen_ix = coords2indices(x_sensor,y_sensor)
		x_sen_idx, y_sen_ix = x_sensor, y_sensor

		# For Each Point in the Bresenham set apply the inverse sensor model and 
		for data in output:
			x_bres, y_bres = data
			x_bres_coord, y_bres_coord = coords2indices(x_bres, y_bres)

			idx_data = (x_bres_coord, y_bres_coord)

			# print(f"Coords = {data} Index = {idx_data} ")
			bres_thing.append([x_bres, y_bres])
			try:
				map[x_bres_coord][y_bres_coord] += inverse_sensor_model(data, (x_sen_idx,y_sen_ix), (0,0))
			except:
				pass


	for idx , _ in np.ndenumerate(map):

		x, y = idx

		try:
			map[x][y] = np.exp(map[x][y])/(1 + np.exp(map[x][y]))
		except:
			pass


	plt.imshow(map, cmap='Greys', interpolation='nearest')
	plt.colorbar()
	plt.show()

	bres_x = [i for i, _ in bres_thing]
	bres_y = [j for _, j in bres_thing]
	# # print(output)

	# plt.scatter(bres_x, bres_y)
	# # plt.scatter(x, y)
	# plt.show()


	# Convert Grid Coords to Map Coords

	# Find Scan Extreme points <Lidar origin, Lidar Measurement> (rounded into grid coords)

	# Iterate Over Bresenham Set 

	# Compute Inverse Sensor Model 























	# path = "python_plot/lidar_data.json"

	# f = open(path)

	# data = json.load(f)
	# print(data)

	# # Angles and Increments are in radians
	# start_angle = data["angle_min"]
	# end_angle = data['angle_max']
	# angle_increment = data["angle_increment"]

	# ranges = data["ranges"]

	# min_range = data["range_min"]
	# max_range = data['range_max']

	# filt_ranges = [] 

	# for dist in ranges:

	#     if dist > max_range or dist < min_range:
	#         dist = 0

	#     filt_ranges.append(dist)
		 



	# theta = np.arange(start_angle, end_angle, angle_increment)

	# fig = plt.figure()
	# ax = fig.add_subplot(projection='polar')
	# c = ax.scatter(theta, filt_ranges, cmap='hsv', alpha=0.75)
	# plt.show()