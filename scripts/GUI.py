#!/usr/bin/env python

import rospy
import cv2
import numpy as np
import os
import robot_configure
import math
from std_msgs.msg import String
import json

put_info_flag = True

init_gps_flag = False
size = (600, 600)

lat = []
lon = []
bearing = []

init_lon = 121.415898
init_lat = 31.21884
init_bearing = 0.0

map_name = 'map'
map_scale = 1.0
gps_scale = 15000000

loc_dataLength = 500
triangle_size = 6
triangle_angle = math.pi * 0.5
circle_radius = 2
box_width = 108/map_scale
box_height = 120/map_scale

cross_pos = {"x": [], "y": []}
cross_latlon = {"lon": [], "lat": []}
cross_size = 5

run = 1
json_str = {"init_point": {"lng": init_lon, "lat": init_lat}, "route": [], "run": run}
clear_route_flag = False

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
			cv2.line(img, (int(size[0]/2.0), int(size[1]/2.0)), (int(cross_pos["x"][0]), int(cross_pos["y"][0])), (100, 100, 100), 1)
			cv2.line(img, (int(cross_pos["x"][i]), int(cross_pos["y"][i])), (int(cross_pos["x"][j]), int(cross_pos["y"][j])), (100, 100, 0), 1)


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
	elif event == cv2.EVENT_LBUTTONDOWN:
		cross_pos["x"].append(x)
		cross_pos["y"].append(y)
		cross_lon = (x - size[0]/2.0) * float(map_scale) / float(gps_scale) + init_lon
		cross_lat = (size[1]/2.0 - y) * float(map_scale) / float(gps_scale) + init_lat
		cross_latlon["lon"].append(cross_lon)
		cross_latlon["lat"].append(cross_lat)
		json_str["route"].append({"lng": cross_lon, "lat": cross_lat})
		# print json_str

def draw_base(img):
	global init_lon, init_lat, size
	cv2.rectangle(img, (int(size[0]/2.0 - (50/float(map_scale))), int(size[1]/2.0 - (50/float(map_scale)))), (int(size[0]/2.0 + (50/float(map_scale))), int(size[1]/2.0 + (50/float(map_scale)))), (0,255,0), -1)
	if put_info_flag:
		put_pos(img, init_lon, init_lat, size[0]/2.0, size[1]/2.0)




if __name__ == '__main__':

	try:
		rospy.init_node('simulator_GUI', anonymous = True)
		rospy.Subscriber('gps', String, GPS_callback)
		json_pub = rospy.Publisher('job', String, queue_size = 100)

		read_Init()

		cv2.namedWindow(map_name)
		cv2.setMouseCallback(map_name, set_Route)

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



			cv2.imshow(map_name, frame)

			c = cv2.waitKey(20) & 0xFF
			if c == 13:

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
