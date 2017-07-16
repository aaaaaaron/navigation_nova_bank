#!/usr/bin/env python

import rospy
import cv2
import numpy as np
import os
import robot_configure
import math
from std_msgs.msg import String

init_gps_flag = False
size = (600, 600)

lat = []
lon = []
bearing = []

init_lat = 0.0
init_lon = 0.0
init_bearing = 0.0

loc_dataLength = 50
triangle_size = 6
triangle_angle = math.pi * 0.5
circle_radius = 2

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
	cv2.line(img, point2, point3, (0, 255, 0), 3)


if __name__ == '__main__':
	rospy.init_node('simulator_GUI', anonymous = True)
	rospy.Subscriber('gps', String, GPS_callback)

	read_Init()

	try:
		while not init_gps_flag:
			rospy.loginfo("first set of gps not obtained")
	
		while not rospy.is_shutdown():
			map_bg = np.zeros((size[1], size[0], 3), np.uint8)
			frame = np.array(map_bg)
			x = (lon[-1] - init_lon) * 15000000 + size[0]/2.0
			y = size[1] - ((lat[-1] - init_lat) * 15000000 + size[1]/2.0)
			
			for i in range(len(lat)-1):
				x_prev = (lon[i] - init_lon) * 15000000 + size[0]/2.0
				y_prev = size[1] - ((lat[i] - init_lat) * 15000000 + size[1]/2.0)
				b_prev = 0
				cv2.circle(frame, (int(x_prev), int(y_prev)), circle_radius, (0, 0, 255), -1)
			
			draw_triangle(frame, x, y, bearing[-1])

			cv2.imshow("map", frame)
			if cv2.waitKey(20) & 0xFF == 27:
				break
		
		cv2.destroyWindow("map")
		rospy.spin()
	except rospy.ROSInterruptException:
		cv2.destroyWindow("map")