#!/usr/bin/env python

import rospy
import cv2
import numpy as np
import os
import robot_configure
import math
from std_msgs.msg import String
from serial_handler.msg import Sonar
# from serial_handler.msg import Status
# from serial_handler.msg import Encoder
import json
import simulator


put_info_flag = True
put_route_line = True
obstacle_flag = True
draw_sensor_flag = True
panel_flag = True

init_gps_flag = False
size = (600, 600)

# obstacle1 = [(int(size[0]/2.0 - 20), int(size[1]/4.0 - 20)), (int(size[0]/2.0 + 20), int(size[1]/4.0 - 20)), (int(size[0]/2.0 + 20), int(size[1]/4.0 + 20)), (int(size[0]/2.0 - 20), int(size[1]/4.0 + 20))]
obstacle1 = [(280, 130), (320, 130), (320, 170), (280, 170)]
obstacle2 = [(135,200),(180,210),(175,220),(120,220)]
obstacle_coord = [obstacle1, obstacle2]
obstacle_points = []

panel_pos = [500, 300]

lat = []
lon = []
bearing = []

init_lon = 0.0
init_lat = 0.0
init_bearing = 0.0

map_name = 'map'
map_scale = 10.0
gps_scale = 15000000

loc_dataLength = 1000
triangle_size = 6
triangle_angle = math.pi * 0.5
circle_radius = 2
box_width = 108/map_scale
box_height = 120/map_scale
sensor_dist = 288/map_scale
sensor_angle = 110 / 180.0 * math.pi
front_sensor_points1 = []
front_sensor_points2 = []
front_sensor_points3 = []
front_sensor_points4 = []
back_sensor_points1 = []
back_sensor_points2 = []
back_sensor_points3 = []
back_sensor_points4 = []
front_sensor_pose = []
back_sensor_pose = []
front_sensor_values = {1: 8, 2: 8, 3: 8, 4: 8}
back_sensor_values = {1: 8, 2: 8, 3: 8, 4: 8}

cross_pos = {"x": [], "y": []}
cross_latlon = {"lon": [], "lat": []}
cross_size = 5

run = 1
json_str = {"robot_id":"12", "init_point": {"lng": init_lon, "lat": init_lat}, "route": [], "run": run}
clear_route_flag = False

json_str_panel = {"panel_gps": {"lng": 0, "lat": 0}, "name": "GUI_test"}

def read_Init():
	print("Initializing position")
	config_path = os.path.dirname(os.path.abspath(__file__)) + '/robot.cfg'
	size_para   = 3
	ret         = [None] * size_para

	global init_lon, init_lat, init_bearing, init_gps_flag
	global lat, lon, bearing

	ret[0], init_lon                         = robot_configure.read_config_float(config_path, 'init', 'init_lon')
	ret[1], init_lat                         = robot_configure.read_config_float(config_path, 'init', 'init_lat')
	ret[2], init_bearing                     = robot_configure.read_config_float(config_path, 'init', 'init_bearing')

	lat.append(init_lat)
	lon.append(init_lon)
	bearing.append(init_bearing)

	json_str["init_point"]["lng"] = init_lon
	json_str["init_point"]["lat"] = init_lat

	if ret[0] and ret[1] and ret[2]:
		init_gps_flag = True



def GPS_callback(data):
	global lat, lon, bearing, loc_dataLength
	string = data.data
	if len(string) >= 30:
		gps = string.split(" ")
		if len(gps) == 3:
			lon.append(float(gps[0]))
			while len(lat) > loc_dataLength:
				del(lat[0])
			lat.append(float(gps[1]))
			while len(lon) > loc_dataLength:
				del(lon[0])
			bearing.append(float(gps[2]))
			while len(bearing) > loc_dataLength:
				del(bearing[0])
		else:
			rospy.loginfo("bad GPS")
	else:
		rospy.loginfo("bad GPS")

def dot_products(a, b):
	new_vector = []
	for i in range(len(a)):
		value = 0
		for j in range(len(b)):
			value = value + a[i][j] * b[j]
		new_vector.append(value)
	return (new_vector[0], new_vector[1])

def draw_panel(img):
	global panel_pos
	a = [panel_pos[0] - 20/float(map_scale), panel_pos[1] - 40/float(map_scale)]
	b = [panel_pos[0] + 20/float(map_scale), panel_pos[1] + 40/float(map_scale)]
	cv2.rectangle(img, (int(a[0]), int(a[1])), (int(b[0]), int(b[1])), (255,255,255), -1)
	

def draw_triangle(img, x, y, bearing):
	global triangle_size, triangle_angle
	point1_bef = (0,  - triangle_size)
	point2_bef = (triangle_size * math.sin(triangle_angle * 0.5), triangle_size * math.cos(triangle_angle * 0.5))
	point3_bef = (- triangle_size * math.sin(triangle_angle * 0.5), triangle_size * math.cos(triangle_angle * 0.5))
	R = [[math.cos(bearing/180.0*math.pi), -math.sin(bearing/180.0*math.pi), x], [math.sin(bearing/180.0*math.pi), math.cos(bearing/180.0*math.pi), y], [0, 0, 1]]
	point1_bef = [point1_bef[0], point1_bef[1], 1]
	point2_bef = [point2_bef[0], point2_bef[1], 1]
	point3_bef = [point3_bef[0], point3_bef[1], 1]
	point1 = dot_products(R, point1_bef)
	point2 = dot_products(R, point2_bef)
	point3 = dot_products(R, point3_bef)
	point1 = (int(point1[0]), int(point1[1]))
	point2 = (int(point2[0]), int(point2[1]))
	point3 = (int(point3[0]), int(point3[1]))
	cv2.line(img, point1, point2, (255, 0, 0), 3)
	cv2.line(img, point3, point1, (255, 0, 0), 3)
	cv2.line(img, point2, point3, (0, 0, 0), 3)

def draw_sensor(img, x, y, bearing):
	global box_width, box_height, triangle_angle
	global sensor_angle, sensor_dist, front_sensor_pose, back_sensor_pose
	R = [[math.cos(bearing/180.0*math.pi), -math.sin(bearing/180.0*math.pi), x], [math.sin(bearing/180.0*math.pi), math.cos(bearing/180.0*math.pi), y], [0, 0, 1]]

	f1 = [3 * box_width/8.0, -box_height/2.0, 1]
	f2 = [box_width/8.0, -box_height/2.0, 1]
	f3 = [-box_width/8.0, -box_height/2.0, 1]
	f4 = [-3 * box_width/8.0, -box_height/2.0, 1]

	b1 = [-3 * box_width/8.0, box_height/2.0, 1]
	b2 = [-box_width/8.0, box_height/2.0, 1]
	b3 = [box_width/8.0, box_height/2.0, 1]
	b4 = [3 * box_width/8.0, box_height/2.0, 1]

	f = [f1, f2, f3, f4]
	b = [b1, b2, b3, b4]

	front_sensor_pose = []
	back_sensor_pose = []

	for ii in f:
		aa = dot_products(R, ii)
		front_sensor_pose.append((int(aa[0]), int(aa[1])))
	for jj in b:
		cc = dot_products(R, jj)
		back_sensor_pose.append((int(cc[0]),int(cc[1])))

	fleft_boundary_near_bef = [box_width/2.0, -box_height/2.0, 1]
	fright_boundary_near_bef = [-box_width/2.0, -box_height/2.0, 1]
	fsensor_12_near_bef = [fleft_boundary_near_bef[0] - box_width/4.0, fleft_boundary_near_bef[1], 1]
	fsensor_23_near_bef = [0, fleft_boundary_near_bef[1], 1]
	fsensor_34_near_bef = [fright_boundary_near_bef[0] + box_width/4.0, fleft_boundary_near_bef[1], 1]

	fleft_boundary_far_bef = [fleft_boundary_near_bef[0] + sensor_dist * math.cos((math.pi - sensor_angle) /2.0), fleft_boundary_near_bef[1] - sensor_dist * math.sin((math.pi - sensor_angle) /2.0), 1]
	fright_boundary_far_bef = [fright_boundary_near_bef[0] - sensor_dist * math.cos((math.pi - sensor_angle) /2.0), fright_boundary_near_bef[1] - sensor_dist * math.sin((math.pi - sensor_angle) /2.0), 1]
	fsensor_12_far_bef = [fsensor_12_near_bef[0] + sensor_dist * math.cos((math.pi - sensor_angle) /2.0 + sensor_angle/4.0), fsensor_12_near_bef[1] - sensor_dist * math.sin((math.pi - sensor_angle) /2.0 + sensor_angle/4.0), 1]
	fsensor_23_far_bef = [0, fsensor_23_near_bef[1] - sensor_dist, 1]
	fsensor_34_far_bef = [fsensor_34_near_bef[0] - sensor_dist * math.cos((math.pi - sensor_angle) /2.0 + sensor_angle/4.0), fsensor_34_near_bef[1] - sensor_dist * math.sin((math.pi - sensor_angle) /2.0 + sensor_angle/4.0), 1]

	bleft_boundary_near_bef = [-box_width/2.0, box_height/2.0, 1]
	bright_boundary_near_bef = [box_width/2.0, box_height/2.0, 1]
	bsensor_12_near_bef = [bleft_boundary_near_bef[0] + box_width/4.0, bleft_boundary_near_bef[1], 1]
	bsensor_23_near_bef = [0, bleft_boundary_near_bef[1], 1]
	bsensor_34_near_bef = [bright_boundary_near_bef[0] - box_width/4.0, bleft_boundary_near_bef[1], 1]

	bleft_boundary_far_bef = [bleft_boundary_near_bef[0] - sensor_dist * math.cos((math.pi - sensor_angle) /2.0), bleft_boundary_near_bef[1] + sensor_dist * math.sin((math.pi - sensor_angle) /2.0), 1]
	bright_boundary_far_bef = [bright_boundary_near_bef[0] + sensor_dist * math.cos((math.pi - sensor_angle) /2.0), bright_boundary_near_bef[1] + sensor_dist * math.sin((math.pi - sensor_angle) /2.0), 1]
	bsensor_12_far_bef = [bsensor_12_near_bef[0] - sensor_dist * math.cos((math.pi - sensor_angle) /2.0 + sensor_angle/4.0), bsensor_12_near_bef[1] + sensor_dist * math.sin((math.pi - sensor_angle) /2.0 + sensor_angle/4.0), 1]
	bsensor_23_far_bef = [0, bsensor_23_near_bef[1] + sensor_dist, 1]
	bsensor_34_far_bef = [bsensor_34_near_bef[0] + sensor_dist * math.cos((math.pi - sensor_angle) /2.0 + sensor_angle/4.0), bsensor_34_near_bef[1] + sensor_dist * math.sin((math.pi - sensor_angle) /2.0 + sensor_angle/4.0), 1]

	fsensor_points_bef = [fleft_boundary_near_bef, fright_boundary_near_bef, fsensor_12_near_bef, fsensor_23_near_bef, fsensor_34_near_bef, fleft_boundary_far_bef, fright_boundary_far_bef, fsensor_12_far_bef, fsensor_23_far_bef, fsensor_34_far_bef]
	bsensor_points_bef = [bleft_boundary_near_bef, bright_boundary_near_bef, bsensor_12_near_bef, bsensor_23_near_bef, bsensor_34_near_bef, bleft_boundary_far_bef, bright_boundary_far_bef, bsensor_12_far_bef, bsensor_23_far_bef, bsensor_34_far_bef]

	fsensor_points = []
	bsensor_points = []

	for i in fsensor_points_bef:
		a = dot_products(R, i)
		fsensor_points.append(a)
	for j in bsensor_points_bef:
		b = dot_products(R, j)
		bsensor_points.append(b)

	fleft_boundary_near = (int(fsensor_points[0][0]), int(fsensor_points[0][1]))
	fright_boundary_near = (int(fsensor_points[1][0]), int(fsensor_points[1][1]))
	fsensor_12_near = (int(fsensor_points[2][0]), int(fsensor_points[2][1]))
	fsensor_23_near = (int(fsensor_points[3][0]), int(fsensor_points[3][1]))
	fsensor_34_near = (int(fsensor_points[4][0]), int(fsensor_points[4][1]))
	fleft_boundary_far = (int(fsensor_points[5][0]), int(fsensor_points[5][1]))
	fright_boundary_far = (int(fsensor_points[6][0]), int(fsensor_points[6][1]))
	fsensor_12_far = (int(fsensor_points[7][0]), int(fsensor_points[7][1]))
	fsensor_23_far = (int(fsensor_points[8][0]), int(fsensor_points[8][1]))
	fsensor_34_far = (int(fsensor_points[9][0]), int(fsensor_points[9][1]))

	bleft_boundary_near = (int(bsensor_points[0][0]), int(bsensor_points[0][1]))
	bright_boundary_near = (int(bsensor_points[1][0]), int(bsensor_points[1][1]))
	bsensor_12_near = (int(bsensor_points[2][0]), int(bsensor_points[2][1]))
	bsensor_23_near = (int(bsensor_points[3][0]), int(bsensor_points[3][1]))
	bsensor_34_near = (int(bsensor_points[4][0]), int(bsensor_points[4][1]))
	bleft_boundary_far = (int(bsensor_points[5][0]), int(bsensor_points[5][1]))
	bright_boundary_far = (int(bsensor_points[6][0]), int(bsensor_points[6][1]))
	bsensor_12_far = (int(bsensor_points[7][0]), int(bsensor_points[7][1]))
	bsensor_23_far = (int(bsensor_points[8][0]), int(bsensor_points[8][1]))
	bsensor_34_far = (int(bsensor_points[9][0]), int(bsensor_points[9][1]))

	fboundary = [fleft_boundary_near, fleft_boundary_far, fsensor_12_far, fsensor_23_far, fsensor_34_far, fright_boundary_far, fright_boundary_near, fsensor_34_near, fsensor_23_near, fsensor_12_near]
	f12_divide = [fsensor_12_near, fsensor_12_far]
	f23_divide = [fsensor_23_near, fsensor_23_far]
	f34_divide = [fsensor_34_near, fsensor_34_far]

	bboundary = [bleft_boundary_near, bleft_boundary_far, bsensor_12_far, bsensor_23_far, bsensor_34_far, bright_boundary_far, bright_boundary_near, bsensor_34_near, bsensor_23_near, bsensor_12_near]
	b12_divide = [bsensor_12_near, bsensor_12_far]
	b23_divide = [bsensor_23_near, bsensor_23_far]
	b34_divide = [bsensor_34_near, bsensor_34_far]

	if draw_sensor_flag:
		for i in range(-1, len(fboundary)-1):
			cv2.line(img, fboundary[i], fboundary[i+1], (0, 0, 255), 1)

		for j in range(-1, len(bboundary)-1):
			cv2.line(img, bboundary[j], bboundary[j+1], (0, 0, 255), 1)

		cv2.line(img, f12_divide[0], f12_divide[1], (0, 0, 255), 1)
		cv2.line(img, f23_divide[0], f23_divide[1], (0, 0, 255), 1)
		cv2.line(img, f34_divide[0], f34_divide[1], (0, 0, 255), 1)

		cv2.line(img, b12_divide[0], b12_divide[1], (0, 0, 255), 1)
		cv2.line(img, b23_divide[0], b23_divide[1], (0, 0, 255), 1)
		cv2.line(img, b34_divide[0], b34_divide[1], (0, 0, 255), 1)

	fbound1 = [fleft_boundary_near, fleft_boundary_far, fsensor_12_far, fsensor_12_near]
	fbound2 = [fsensor_12_near, fsensor_12_far, fsensor_23_far, fsensor_23_near]
	fbound3 = [fsensor_23_near, fsensor_23_far, fsensor_34_far, fsensor_34_near]
	fbound4 = [fsensor_34_near, fsensor_34_far, fright_boundary_far, fright_boundary_near]
	bbound1 = [bleft_boundary_near, bleft_boundary_far, bsensor_12_far, bsensor_12_near]
	bbound2 = [bsensor_12_near, bsensor_12_far, bsensor_23_far, bsensor_23_near]
	bbound3 = [bsensor_23_near, bsensor_23_far, bsensor_34_far, bsensor_34_near]
	bbound4 = [bsensor_34_near, bsensor_34_far, bright_boundary_far, bright_boundary_near]
	get_sensor_boundaries([fbound1, fbound2, fbound3, fbound4, bbound1, bbound2, bbound3, bbound4])

def get_sensor_boundpoints(sensor_list, global_sensor_list):
	for j in range(-1, len(sensor_list)-1):
		p1 = sensor_list[j]
		p2 = sensor_list[j+1]
		try:
			m = (p2[1] - p1[1])/float(p2[0] - p1[0])
			c = p1[1] - m * p1[0]
			for x in range(min(p1[0], p2[0]), max(p1[0],p2[0])+1):
				y = m * x + c
				if (x, y) not in global_sensor_list:
					global_sensor_list.append((int(x), int(y)))
		except ZeroDivisionError:
			for y in range(min(p1[1],p2[1]),max(p1[1],p2[1])+1):
				if (p1[0], y) not in global_sensor_list:
					global_sensor_list.append((int(p1[0]), int(y)))

def get_sensor_boundaries(sensor_list):
	global front_sensor_points1, front_sensor_points2, front_sensor_points3, front_sensor_points4
	global back_sensor_points1, back_sensor_points2, back_sensor_points3, back_sensor_points4
	front_sensor_points1 = []
	front_sensor_points2 = []
	front_sensor_points3 = []
	front_sensor_points4 = []
	back_sensor_points1 = []
	back_sensor_points2 = []
	back_sensor_points3 = []
	back_sensor_points4 = []

	get_sensor_boundpoints(sensor_list[0], front_sensor_points1)
	get_sensor_boundpoints(sensor_list[1], front_sensor_points2)
	get_sensor_boundpoints(sensor_list[2], front_sensor_points3)
	get_sensor_boundpoints(sensor_list[3], front_sensor_points4)
	get_sensor_boundpoints(sensor_list[4], back_sensor_points1)
	get_sensor_boundpoints(sensor_list[5], back_sensor_points2)
	get_sensor_boundpoints(sensor_list[6], back_sensor_points3)
	get_sensor_boundpoints(sensor_list[7], back_sensor_points4)


def draw_box(img, x, y, bearing):
	global box_width, box_height, triangle_angle
	point1_bef = (-box_width/2.0,  -box_height/2.0)
	point2_bef = (box_width/2.0,  -box_height/2.0)
	point3_bef = (box_width/2.0,  box_height/2.0)
	point4_bef = (-box_width/2.0,  box_height/2.0)
	R = [[math.cos(bearing/180.0*math.pi), -math.sin(bearing/180.0*math.pi), x], [math.sin(bearing/180.0*math.pi), math.cos(bearing/180.0*math.pi), y], [0, 0, 1]]
	point1_bef = [point1_bef[0], point1_bef[1], 1]
	point2_bef = [point2_bef[0], point2_bef[1], 1]
	point3_bef = [point3_bef[0], point3_bef[1], 1]
	point4_bef = [point4_bef[0], point4_bef[1], 1]
	point1 = dot_products(R, point1_bef)
	point2 = dot_products(R, point2_bef)
	point3 = dot_products(R, point3_bef)
	point4 = dot_products(R, point4_bef)
	point1 = (int(point1[0]), int(point1[1]))
	point2 = (int(point2[0]), int(point2[1]))
	point3 = (int(point3[0]), int(point3[1]))
	point4 = (int(point4[0]), int(point4[1]))
	cv2.line(img, point1, point2, (255, 0, 0), 1)
	cv2.line(img, point4, point1, (255, 0, 0), 1)
	cv2.line(img, point2, point3, (255, 0, 0), 1)
	cv2.line(img, point3, point4, (255, 0, 0), 1)

def draw_cross(img):
	global cross_pos, cross_size
	for i in range(len(cross_pos["x"])):
		p1 = (int(cross_pos["x"][i] - cross_size/2.0), int(cross_pos["y"][i] - cross_size/2.0))
		p2 = (int(cross_pos["x"][i] + cross_size/2.0), int(cross_pos["y"][i] - cross_size/2.0))
		p3 = (int(cross_pos["x"][i] + cross_size/2.0), int(cross_pos["y"][i] + cross_size/2.0))
		p4 = (int(cross_pos["x"][i] - cross_size/2.0), int(cross_pos["y"][i] + cross_size/2.0))
		cv2.line(img, p1, p3, (200, 100, 0), 1)
		cv2.line(img, p2, p4, (200, 100, 0), 1)
		cv2.putText(img, "%d"%(i+1), (int(cross_pos["x"][i]), int(cross_pos["y"][i] - 10)), cv2.FONT_HERSHEY_PLAIN, 2, (200, 100, 0), 1)
		j = (i+1)%len(cross_pos["x"])
		if put_info_flag:
			put_pos(img, cross_latlon["lon"][i], cross_latlon["lat"][i], cross_pos["x"][i], cross_pos["y"][i])
		if put_route_line:
			cv2.line(img, (int(size[0]/2.0), int(size[1]/2.0)), (int(cross_pos["x"][0]), int(cross_pos["y"][0])), (100, 100, 100), 1)
			cv2.line(img, (int(cross_pos["x"][i]), int(cross_pos["y"][i])), (int(cross_pos["x"][j]), int(cross_pos["y"][j])), (100, 100, 0), 1)

def draw_obstacle(img):
	global obstacle_coord
	for i in range(len(obstacle_coord)):
		for j in range(-1, len(obstacle_coord[i])-1):
			cv2.line(img, obstacle_coord[i][j], obstacle_coord[i][j+1], (200,0,200), 3)

def get_obstacle_points():
	global obstacle_points
	global obstacle_coord
	for i in range(len(obstacle_coord)):
		for j in range(-1, len(obstacle_coord[i])-1):
			p1 = obstacle_coord[i][j]
			p2 = obstacle_coord[i][j+1]
			try:
				m = (p2[1] - p1[1])/float(p2[0] - p1[0])
				c = p1[1] - m * p1[0]
				for x in range(min(p1[0],p2[0]), max(p1[0],p2[0])+1):
					y = m * x + c
					if (x, y) not in obstacle_points:
						obstacle_points.append((int(x), int(y)))
			except ZeroDivisionError:
				for y in range(min(p1[1],p2[1]), max(p1[1],p2[1])+1):
					if (p1[0], y) not in obstacle_points:
						obstacle_points.append((int(p1[0]), int(y)))
	# print obstacle_points

def check_obstacle_bound_detected(global_sensor_list):
	global obstacle_points

	sensor_x = []
	sensor_y = []
	obstacle_x = []
	obstacle_y = []

	for j in obstacle_points:
		obstacle_x.append(j[0])
		obstacle_y.append(j[1])
	for i in global_sensor_list:
		sensor_x.append(i[0])
		sensor_y.append(i[1])

	min_sensor_x = min(sensor_x)
	max_sensor_x = max(sensor_x)
	min_sensor_y = min(sensor_y)
	max_sensor_y = max(sensor_y)

	upperbound = False
	lowerbound = False

	points_in_boundary = []

	for g in range(len(obstacle_x)):
		if obstacle_x[g] >= min_sensor_x and obstacle_x[g] <= max_sensor_x and obstacle_y[g] >= min_sensor_y and obstacle_y[g] <= max_sensor_y:
			upperbound = False
			lowerbound = False
			sx = sensor_x[:]
			sy = sensor_y[:]
			if obstacle_x[g] in sx:
				index_count = sx.count(obstacle_x[g])
				if index_count >= 2:
					while obstacle_x[g] in sx:
						index = sx.index(obstacle_x[g])
						if sy[index] >= obstacle_y[g]:
							lowerbound = True
						if sy[index] <= obstacle_y[g]:
							upperbound = True
						del sx[index]
						del sy[index]
			# if obstacle_y[g] in front_sensor_y:
			# 	index_count = front_sensor_y.count(obstacle_y[g])
			# 	if index_count >= 2:
			# 		while obstacle_y[g] in front_sensor_y:
			# 			index = front_sensor_y.index(obstacle_y[g])
			# 			if front_sensor_x[index] >= obstacle_x[g]:
			# 				frightbound = True
			# 			if front_sensor_x[index] <= obstacle_x[g]:
			# 				fleftbound = True
			# 			del front_sensor_y[index]
			if upperbound and lowerbound: # and frightbound and fleftbound:
				# rospy.loginfo("front obstacle detected")
				points_in_boundary.append((obstacle_x[g], obstacle_y[g]))
			# else:
			# 	rospy.loginfo("no front obstacle")
	return points_in_boundary


def check_obstacle_detected():

	global front_sensor_points1, front_sensor_points2, front_sensor_points3, front_sensor_points4
	global back_sensor_points1, back_sensor_points2, back_sensor_points3, back_sensor_points4

	f1_points = check_obstacle_bound_detected(front_sensor_points1)
	f2_points = check_obstacle_bound_detected(front_sensor_points2)
	f3_points = check_obstacle_bound_detected(front_sensor_points3)
	f4_points = check_obstacle_bound_detected(front_sensor_points4)

	b1_points = check_obstacle_bound_detected(back_sensor_points1)
	b2_points = check_obstacle_bound_detected(back_sensor_points2)
	b3_points = check_obstacle_bound_detected(back_sensor_points3)
	b4_points = check_obstacle_bound_detected(back_sensor_points4)


	sensor_data(f1_points, 'f', 1)
	sensor_data(f2_points, 'f', 2)
	sensor_data(f3_points, 'f', 3)
	sensor_data(f4_points, 'f', 4)

	sensor_data(b1_points, 'b', 1)
	sensor_data(b2_points, 'b', 2)
	sensor_data(b3_points, 'b', 3)
	sensor_data(b4_points, 'b', 4)


def find_distance(p1, p2):
	h = math.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2)
	return h

def sensor_data(points_list, front_back, index):
	global front_sensor_pose, back_sensor_pose
	global front_sensor_values, back_sensor_values
	global map_scale

	if front_back == 'f':
		min_dist = 100000
		if points_list == []:
			final_value = 8
		else:
			for j in range(len(points_list)):
				dist = find_distance(points_list[j], front_sensor_pose[index-1])
				if dist <= min_dist:
					min_dist = dist
			value = min_dist * map_scale / 1.2
			final_value = int(value/30)
			if final_value > 7:
				final_value = 8
		front_sensor_values[index] = final_value

	elif front_back == 'b':
		min_dist = 100000
		if points_list == []:
			final_value = 8
		else:
			for j in range(len(points_list)):
				dist = find_distance(points_list[j], back_sensor_pose[index-1])
				if dist <= min_dist:
					min_dist = dist
			value = min_dist * map_scale / 1.2
			final_value = int(value/30)
			if final_value > 7:
				final_value = 8
		back_sensor_values[index] = final_value

	# print front_sensor_values


def put_pos(img, lon, lat, x, y, bearing = ""):
	string1_to_put = "Lon: %.10f Lat: %.10f"%(lon, lat)
	string2_to_put = "x: %d               y: %d"%(x,y)
	bearing = str(bearing)
	string3_to_put = "bearing: %s"%bearing
	cv2.putText(img, string1_to_put, (int(x - 125), int(y + 20)), cv2.FONT_HERSHEY_PLAIN, 0.7, (100, 100, 0), 1)
	cv2.putText(img, string2_to_put, (int(x - 75), int(y + 30)), cv2.FONT_HERSHEY_PLAIN, 0.7, (100, 100, 0), 1)
	if bearing == "":
		return
	cv2.putText(img, string3_to_put, (int(x - 30), int(y + 40)), cv2.FONT_HERSHEY_PLAIN, 0.7, (100, 100, 0), 1)

def set_Route(event, x, y, flags, param):
	global json_str, clear_route_flag, init_lon, init_lat
	global cross_pos
	global lat, lon, bearing
	global panel_pos
	a = [panel_pos[0] - 20/float(map_scale), panel_pos[1] - 40/float(map_scale)]
	b = [panel_pos[0] + 20/float(map_scale), panel_pos[1] + 40/float(map_scale)]
	c = [size[0]/2.0 - (50/float(map_scale)), size[1]/2.0 - (50/float(map_scale))]
	d = [size[0]/2.0 + (50/float(map_scale)), size[1]/2.0 + (50/float(map_scale))]
	if event == cv2.EVENT_RBUTTONDOWN:
		json_str["route"] = []
		# clear_route_flag = True
		cross_pos["x"] = []
		cross_pos["y"] = []
		cross_latlon["lon"] = []
		cross_latlon["lat"] = []
		lat = [init_lat]
		lon = [init_lon]
		bearing = [init_bearing]
	elif event == cv2.EVENT_LBUTTONDOWN and (x < a[0] or x > b[0] or y < a[1] or y > b[1]): # and (x < c[0] or x > d[0] or y < c[1] or y > d[1]):
		cross_pos["x"].append(x)
		cross_pos["y"].append(y)
		cross_lon = (x - size[0]/2.0) * float(map_scale) / float(gps_scale) + init_lon
		cross_lat = (size[1]/2.0 - y) * float(map_scale) / float(gps_scale) + init_lat
		cross_latlon["lon"].append(cross_lon)
		cross_latlon["lat"].append(cross_lat)
		json_str["route"].append({"lng": cross_lon, "lat": cross_lat})
		# print json_str
	elif event == cv2.EVENT_LBUTTONDOWN and (x > a[0] and x < b[0] and y > a[1] and y < b[1]):
		print "summon robot (clicked on GUI)"
		panel_lon = (panel_pos[0] - size[0]/2.0) * float(map_scale) / float(gps_scale) + init_lon
		panel_lat = (size[1]/2.0 - panel_pos[1]) * float(map_scale) / float(gps_scale) + init_lat
		json_str_panel["panel_gps"]["lng"] = panel_lon
		json_str_panel["panel_gps"]["lat"] = panel_lat
		json_str_panel_toSend = json.dumps(json_str_panel)
		json_pub_panel.publish(json_str_panel_toSend)
	# elif event == cv2.EVENT_LBUTTONDOWN and (x > c[0] and x < d[0] and y > c[1] and y < d[1]):
	# 	print "back to base"

		


def draw_base(img):
	global init_lon, init_lat, size
	cv2.rectangle(img, (int(size[0]/2.0 - (50/float(map_scale))), int(size[1]/2.0 - (50/float(map_scale)))), (int(size[0]/2.0 + (50/float(map_scale))), int(size[1]/2.0 + (50/float(map_scale)))), (0,255,0), -1)
	if put_info_flag:
		put_pos(img, init_lon, init_lat, size[0]/2.0, size[1]/2.0)

def publish_sonar():
	ss = Sonar()
	ss.front_0 = front_sensor_values[1]
	ss.front_1 = front_sensor_values[2]
	ss.front_2 = front_sensor_values[3]
	ss.front_3 = front_sensor_values[4]
	ss.back_0 = back_sensor_values[1]
	ss.back_1 = back_sensor_values[2]
	ss.back_2 = back_sensor_values[3]
	ss.back_3 = back_sensor_values[4]
	return ss

prev_has_obstacle = 0

# def obstacle_action():




if __name__ == '__main__':

	try:
		rospy.init_node('simulator_GUI', anonymous = True)
		rospy.Subscriber('gps', String, GPS_callback)
		json_pub = rospy.Publisher('job', String, queue_size = 100)
		json_pub_panel = rospy.Publisher('summon_robot', String, queue_size = 100)
		sonar_pub = rospy.Publisher('sonar', Sonar, queue_size = 100)

		read_Init()

		cv2.namedWindow(map_name)
		cv2.setMouseCallback(map_name, set_Route)

		get_obstacle_points()

		while not init_gps_flag:
			rospy.loginfo("first set of gps not obtained")

		while not rospy.is_shutdown():
			map_bg = np.zeros((size[1], size[0], 3), np.uint8)
			frame = np.array(map_bg)
			x = (lon[-1] - init_lon) * gps_scale / float(map_scale) + size[0]/2.0
			y = size[1]/2.0 - ((lat[-1] - init_lat) * gps_scale / float(map_scale))

			draw_base(frame)


			for i in range(len(lat)-1):
				x_prev = (lon[i] - init_lon) * gps_scale / float(map_scale) + size[0]/2.0
				y_prev = size[1]/2.0 - ((lat[i] - init_lat) * gps_scale / float(map_scale))
				# print x_prev, y_prev
				b_prev = 0
				cv2.circle(frame, (int(x_prev), int(y_prev)), circle_radius, (0, 0, 255), -1)


			draw_triangle(frame, x, y, bearing[-1])
			draw_box(frame, x, y, bearing[-1])

			if len(cross_pos["x"]) != 0:
				draw_cross(frame)
			if put_info_flag:
				put_pos(frame, lon[-1], lat[-1], x, y, bearing[-1])

			if obstacle_flag:
				draw_obstacle(frame)
				draw_sensor(frame, x, y, bearing[-1])
				check_obstacle_detected()
				ss = publish_sonar()
				sonar_pub.publish(ss)

			if panel_flag:
				draw_panel(frame)


			cv2.imshow(map_name, frame)

			c = cv2.waitKey(20) & 0xFF
			if c == 13:
				# key_pub.publish('Switch')
				json_str_toSend = json.dumps(json_str)
				json_pub.publish(json_str_toSend)

			elif c == 27:
				break

			# if clear_route_flag:
			# 	json_str_toSend = json.dumps(json_str)
			# 	json_pub.publish(json_str_toSend)
			# 	clear_route_flag = False
			# 	json_str = {"init_point": {"lng": 0.0, "lat": 0.0}, "route": [], "run": run}


		cv2.destroyWindow(map_name)
		rospy.spin()
	except rospy.ROSInterruptException:
		cv2.destroyWindow(map_name)
