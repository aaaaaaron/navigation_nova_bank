#!/usr/bin/env python
import math
import rospy
import robot_correction

from math import radians, cos, sin, asin, sqrt, atan2, degrees

scale = 900

# just make sure the angle is between [0-360)
# def correct_angle(angle):
# 	if(angle < -360 or angle >= 720):
# 		rospy.logerr('Waring Angle not supposed to appear')
# 		return 

# 	if(angle < 0):
# 		angle = angle + 360 
# 	elif(angle >= 360): 
# 		angle = angle - 360 
# 	return angle

def format_bearing(bearing):
	if(bearing < 0.0):
		while(bearing < 0.0):
			bearing += 360.0
		return bearing 
	
	if(bearing >= 360.0):
		while(bearing >= 360.0):
			bearing -= 360.0
	return bearing 


def normal_bearing(x_source, y_source, x_target, y_target, bearing_now=None):
	if (x_source == x_target) and (y_source == y_target):
		return bearing_now
	delta_x = x_target - x_source
	delta_y = y_target - y_source
	angle_from_x = math.atan2(delta_y, delta_x)
	angle_from_y = math.degrees(angle_from_x) * -1 + 90.0
	angle_from_y = format_bearing(angle_from_y)
	return angle_from_y

def normal_dist(x_source, y_source, x_target, y_target):
	global scale
	delta_x = x_target - x_source
	delta_y = y_target - y_source 
	dist = math.sqrt(delta_x**2 + delta_y**2)
	return dist*scale

def get_normal_coord(x_source, y_source, dist, bearing):
	coord_dist = dist/float(scale)
	alpha = -(bearing - 90.0)
	if alpha < -180.0:
		alpha = alpha + 360.0
	elif alpha > 180.0:
		alpha = alpha - 360.0
	x_new = coord_dist * math.cos(math.radians(alpha)) + x_source
	y_new = coord_dist * math.sin(math.radians(alpha)) + y_source
	return x_new, y_new


# Calculate distance between two gps coordinates 
def haversine(lon1, lat1, lon2, lat2):
	if robot_correction.indoor_coord:
		distance = normal_dist(lon1, lat1, lon2, lat2)
	else:
		#convert to radians
		lon1, lat1, lon2, lat2 = map(radians, [lon1, lat1, lon2, lat2])
		#haversine
		dlon = lon2 - lon1
		dlat = lat2 - lat1
		a = sin(dlat/2.0)**2.0 + cos(lat1) * cos(lat2) * sin(dlon/2.0)**2.0
		c = 2.0 * asin(sqrt(a))
		r = 6371.0088 #radius of earth in kilometers
		
		distance =  c * r
		distance = distance * 1000.0 * 1000.0  					# convert distance to mm
	return distance 

# Calculate angle between two diffent gps positions 
def bearing(lon1, lat1, lon2, lat2):  #from position 1 to 2
	if robot_correction.indoor_coord:
		bearing = normal_bearing(lon1, lat1, lon2, lat2)
	else:
		#convert to radians
		lon1, lat1, lon2, lat2 = map(radians, [lon1, lat1, lon2, lat2])
		
		bearing = atan2(sin(lon2-lon1)*cos(lat2), cos(lat1)*sin(lat2)-sin(lat1)*cos(lat2)*cos(lon2-lon1))
		bearing = degrees(bearing)
		bearing = (bearing + 360.0) % 360.0

	return bearing


#calculate the new latitude based on the current gps coordinates and angle and distance 
def get_gps(lon1, lat1, dist, bearing):
	if dist == 0.0:
		return lon1, lat1
	if robot_correction.indoor_coord:
		lon2, lat2 = get_normal_coord(lon1, lat1, dist, bearing)
	else:
		lon1, lat1, bearing	= map(radians, [lon1, lat1,bearing])
		
		r = 6371.0088 * 1000.0 * 1000.0
		delta = dist/r
		lat2 = asin(sin(lat1) * cos(delta) + cos (lat1) * sin(delta)* cos(bearing))
		lon2 = lon1 + atan2(sin(bearing) * sin (delta) * cos(lat1), cos(delta) - sin (lat1) * sin(lat2))
		#rospy.loginfo("TEST: %f, %f", lon2, lat2)
		lon2 = degrees(lon2)
		lat2 = degrees(lat2)
	return lon2, lat2


