#!/usr/bin/env python
import rospy
import serial
import string
import math
import gpsmath
import robot_drive
import robot_job
import robot_publisher

############################################################
min_correction_distance 	= 0.0
min_correction_angle 		= 0.0
correction_count 			= 0.0
max_correction_run 			= 0.0

from math import radians, cos, sin, asin, sqrt, atan2, degrees

#-------------------------------------------------------#
#	Robot error correction module						#
#-------------------------------------------------------#
# while robot's moving, dynamically update robot gps
# still need to handle more scenarios

#aaron : 1/6/2017, this was added because we cant accurately tune how much the robot deviates while going straight
#so this solution is only suitable for demo, while direction is straight, no deviation is assumed
#while turning, no translation is assumed
def update_robot_gps_new(left_encode, right_encode):
	if(left_encode == 0 and right_encode == 0):
		#no updating of information
				#@yuqing_continueturn
		return
	robot_drive.step_angle = 0.0
	robot_drive.step_distance = 0.0


	if (robot_drive.direction == "forward" or robot_drive.direction == "backward"):
		robot_drive.step_distance  	= float(left_encode + right_encode) / (2.0 * robot_drive.linear_encode_to_mm)
		robot_drive.step_angle 		= 0.0
	elif (robot_drive.direction == "left" or robot_drive.direction == "right"):
		robot_drive.step_distance  	= 0.0
		#delta_yaw 		 			= robot_drive.yaw - robot_drive.past_yaw
		#if (delta_yaw > 180.0):
		#	delta_yaw = delta_yaw - 360.0
		#elif (delta_yaw < -180.0):
		#	delta_yaw = delta_yaw + 360.0
		#robot_drive.step_angle 		= delta_yaw

		arc_length 				 	= float(left_encode - right_encode) / (2.0 * robot_drive.turning_encode_to_mm)
		robot_drive.step_angle 		= (arc_length * 180.0) / (robot_drive.turn_radius * 3.14159265)
	else:
		arc_length 				 	= float(left_encode - right_encode) / (2.0 * robot_drive.turning_encode_to_mm)
		robot_drive.step_distance  	= float(left_encode + right_encode) / (2.0 * robot_drive.linear_encode_to_mm)
		robot_drive.step_angle 		= (arc_length * 180.0) / (robot_drive.turn_radius * 3.14159265)


	if robot_drive.show_log:
		rospy.loginfo("Step distance moved %fmm, Step_angle %f degree", robot_drive.step_distance, robot_drive.step_angle)
	robot_drive.lon_now, robot_drive.lat_now 	= gpsmath.get_gps(robot_drive.lon_now, robot_drive.lat_now, robot_drive.step_distance, robot_drive.bearing_now)
	robot_drive.bearing_now 					= gpsmath.format_bearing(robot_drive.bearing_now + robot_drive.step_angle)
	if robot_drive.show_log:
		rospy.loginfo("Bearing now %f,lon_now %f, lat_now %f", robot_drive.bearing_now, robot_drive.lon_now, robot_drive.lat_now)

def update_robot_gps(left_encode, right_encode):
	robot_drive.step_angle = 0.0
	robot_drive.step_distance = 0.0

	#scenario 1, robot not moving
	if(left_encode == 0 and right_encode == 0):
		#no updating of information
				#@yuqing_continueturn
		return

	# loacal vaiables

	if (robot_drive.direction == "forward" or robot_drive.direction == "backward"):
		left_dist 	= float(left_encode) / robot_drive.linear_encode_to_mm
		right_dist 	= float(right_encode) / robot_drive.linear_encode_to_mm
		left_dist_turn  = float(left_encode) / robot_drive.turning_encode_to_mm 	#aaron 8 july
		right_dist_turn = float(right_encode) / robot_drive.turning_encode_to_mm 	#aaron 8 july
	else :
		left_dist 	= float(left_encode) / robot_drive.turning_encode_to_mm
		right_dist 	= float(right_encode) / robot_drive.turning_encode_to_mm
		left_dist_turn  = float(left_encode) / robot_drive.turning_encode_to_mm 	#aaron 8 july
		right_dist_turn = float(right_encode) / robot_drive.turning_encode_to_mm	#aaron 8 july

	alpha 		= 0.0 # step turn angle in radian
	total_dist 	= abs(left_dist) + abs(right_dist)
	R 			= 0.0 # turn raidus, if two wheels in same direction, then the faster wheel radius is R + robot_drive.turn_radius
	# the slower wheel is R - robot_drive.turn_radius

	# global vaiables
	robot_drive.step_angle 		= 0.0
	robot_drive.step_distance 	= (left_dist + right_dist) / 2.0

	#rospy.loginfo("Bearing now %f,lon_now %f, lat_now %f", robot_drive.bearing_now, robot_drive.lon_now, robot_drive.lat_now)
	# scenario 01, 02 robot moving perfectly straight, bearing won't change, while lan and lon need to be updated
	if(left_dist == right_dist):
		if(right_dist > 0.0 ):
			robot_drive.lon_now, robot_drive.lat_now = gpsmath.get_gps(robot_drive.lon_now, robot_drive.lat_now , right_dist, robot_drive.bearing_now)
		if(right_dist < 0.0):
			robot_drive.lon_now, robot_drive.lat_now = gpsmath.get_gps(robot_drive.lon_now, robot_drive.lat_now , right_dist, -robot_drive.bearing_now)
		#rospy.loginfo("Bearing now %f,lon_now %f, lat_now %f", robot_drive.bearing_now, robot_drive.lon_now, robot_drive.lat_now)
		return
	# scenario 02 robot moving forward with slight
	# robot not so perfectly walking forward, eigher left wheel is faster or right wheel is faster
	elif(left_dist > 0.0 and right_dist > 0.0):
		# a little bit of right turning
		alpha 	= (left_dist_turn - right_dist_turn) / (2.0 * robot_drive.turn_radius)
		R 	= (total_dist * robot_drive.turn_radius) / abs(left_dist_turn - right_dist_turn) 		#aaron 8 july
	# scenario 04 robot moving backward
	elif(left_dist < 0.0 and right_dist < 0.0):
		alpha 	= (left_dist_turn - right_dist_turn) / (2.0 * robot_drive.turn_radius)
		R 	= -total_dist * robot_drive.turn_radius / abs(right_dist_turn - left_dist_turn) 		#aaron 8 july
	# for robot two wheels not moving at the same direction or once of the thing not moving
	# forwaring with rotation
	else:
		alpha 	= total_dist / (2.0 * robot_drive.turn_radius)
		r1 		= abs(left_dist) / alpha
		r2 		= abs(right_dist) / alpha
		R 		= abs(r1 - r2)

		#right turn
		if(left_dist >= 0.0 and right_dist < 0.0):
		#if(left_dist > 0.0 or (left_dist == 0.0 and right_dist < 0.0)):
			alpha = alpha
		else:
			alpha = -alpha

	robot_drive.step_angle 	= degrees(alpha)
	# covnert to degree
	#####bearing 				= robot_drive.bearing_now + robot_drive.step_angle / 2.0
	bearing = robot_drive.bearing_now + robot_drive.step_angle
	# convert between [0 - 360)
	bearing 				= gpsmath.format_bearing(bearing)
	dist 					= R * sin(abs(alpha/2.0)) * 2.0
	if robot_drive.show_log:
		rospy.loginfo("Step in straight line %f mm, Step_angle %f degree, R %f mm, Step_distance calculated %f mm", dist, robot_drive.step_angle, R, robot_drive.step_distance)
	robot_drive.lon_now, robot_drive.lat_now 	= gpsmath.get_gps(robot_drive.lon_now, robot_drive.lat_now, dist, bearing)
	robot_drive.bearing_now 			= bearing
	#robot_publisher.publish_gps()
	#rospy.loginfo("Bearing now %f,lon_now %f, lat_now %f", robot_drive.bearing_now, robot_drive.lon_now, robot_drive.lat_now)

def dist_correction_normal():
	rospy.loginfo("************** Check errors after a normal job **************")
	distance_correction(robot_drive.lon_now, robot_drive.lat_now, robot_drive.bearing_now, robot_drive.lon_target, robot_drive.lat_target, robot_drive.bearing_target, 'C')
	rospy.loginfo("************** Completed checking errors after a normal job **************")

def dist_correction_correction():
	rospy.loginfo("************** Check errors after correction jobs **************")
	distance_correction(robot_drive.lon_now, robot_drive.lat_now, robot_drive.bearing_now, robot_drive.lon_target, robot_drive.lat_target, robot_drive.bearing_target, 'C')
	rospy.loginfo("************** Completed errors after correction jobs **************")

def dist_correction_obstacle():
	rospy.loginfo("**************Check erros after obstacle avoidence**************")
	distance_correction(robot_drive.lon_now, robot_drive.lat_now, robot_drive.bearing_now, robot_drive.lon_target, robot_drive.lat_target, robot_drive.bearing_target, 'O')
	rospy.loginfo("**************Added correction jobs after obstacle avoidence**************")

# correct robot every time by comapring the lat_now, lon_now with target position
def distance_correction(lon_now, lat_now, bearing_now, lon_target, lat_target, bearing_target, correction_type):
	distance 	= gpsmath.haversine(lon_now, lat_now, lon_target, lat_target)
	bearing 	= gpsmath.bearing(lon_now, lat_now, lon_target, lat_target)
	# check the bearing now and bearing target
	rospy.loginfo("Correction Type: %s", correction_type)
	rospy.loginfo("GPS now [%f, %f], GPS target: [%f, %f]", lon_now, lat_now, lon_target, lat_target)
	rospy.loginfo("Bearing now %f, bearing target %f, Bearing move %f, ", bearing_now, bearing_target, bearing)

	diff_angle = (bearing_target - bearing_now + 360.0) % 360.0
	if (diff_angle > 180.0):
		diff_angle = diff_angle - 360.0

	#if((bearing - bearing_now + 360.0)%360.0 >= 90):
	#	distance = -distance

	#if(bearing > 90.0 and bearing < 270.0):
	#	distance = -distance
	#	bearing = (bearing + 180.0) % 360.0

	rospy.loginfo("There's a %f mm distance error, towards %f bearing, a target angle difference of %f, %f", distance, bearing, diff_angle, min_correction_distance)

	need_correct_distance 	= abs(distance) > min_correction_distance
	need_correct_angle 		= abs(diff_angle) > min_correction_angle
	#need_correct_angle 		=  diff_angle > min_correcton_angle and diff_angle < (360.0 - min_correction_angle)

	# if need_correct_distance or need_correct_angle:
	# 	#robot_job.insert_compensation_jobs(lon_now, lat_now, lon_target, lat_target, correction_type, need_correct_distance, need_correct_angle)
	# 	robot_job.insert_compensation_jobs(lon_now, lat_now, bearing_now, lon_target, lat_target, bearing_target, correction_type, need_correct_distance, need_correct_angle)
	# else:
	# 	rospy.loginfo("no need to compensate errors")

# Correct a robot with obstancles by inserting a job to move the robot forward for 1m
def dist_correction_obstacle_need_forward(dist):
	rospy.loginfo("**************obstance correction jobs**************")
	lon_new, lat_new = gpsmath.get_gps(robot_drive.lon_now, robot_drive.lat_now, dist, robot_drive.bearing_now)
	#fist distance correction
	distance_correction(lon_new, lat_new, robot_drive.bearing_now, robot_drive.lon_target, robot_drive.lat_target, robot_drive.bearing_target, 'O')
	#rospy.loginfo("There's a %f mm distance error, %f angle difference", distance, diff_angle)
	rospy.loginfo("Add a job to move forward %d mm", robot_job.dist_forward_after_obstacle)
	robot_job.insert_compensation_jobs(robot_drive.lon_now, robot_drive.lat_now, lat_new, lon_new, 'O', True, False)

# Init gps when robot power on
def init_gps():
	# Setting other dependant parameters
    robot_drive.lon_now         = robot_job.init_lon
    robot_drive.lat_now         = robot_job.init_lat
    robot_drive.lon_target      = robot_job.init_lon
    robot_drive.lat_target      = robot_job.init_lat
    robot_drive.bearing_now     = robot_job.init_bearing
    robot_drive.bearing_target  = robot_job.init_bearing

def distance_bearing_to_target():
	# Get the distance
	distance 	= gpsmath.haversine(robot_drive.lon_now, robot_drive.lat_now, robot_drive.lon_target, robot_drive.lat_target)
	bearing 	= gpsmath.bearing(robot_drive.lon_now, robot_drive.lat_now, robot_drive.lon_target, robot_drive.lat_target)
	angle 		= gpsmath.format_bearing(bearing - robot_drive.bearing_now)
	return distance, angle

