#!/usr/bin/env python
import rospy
import serial
import string
from std_msgs.msg import String
import robot_drive
import math
import robot_correction
import robot_job
import robot_publisher
import gpsmath

#-------------------------------------------------------#
# Robot turning module 									#
#-------------------------------------------------------#
degree_turned 		= 0
degree_to_turn 		= 0

angle_lower_speed 	= 0.0
angle_lowest_speed 	= 0.0

turn_full_speed 	= 0.0
turn_lower_speed 	= 0.0
turn_lowest_speed 	= 0.0

last_angle              = 370.0
first_angle		= 370.0

turn_amend = False

#it takes about 1 second for it to stop

# start a turn job
def start_turn():
	global degree_turned
	global degree_to_turn

	global angle_lower_speed
	global angle_lowest_speed

	global turn_full_speed
	global turn_lower_speed
	global turn_lowest_speed

	global first_angle
	global last_angle

	first_angle = 370.0
	last_angle = 370.0

	# get turning angle to (-180 to 180)
	if(degree_to_turn > 180.0):
		degree_to_turn = degree_to_turn - 360.0
	elif(degree_to_turn < -180.0):
		degree_to_turn = degree_to_turn + 360.0

	if (degree_to_turn < 0.0): #Left turning
 		robot_drive.move_direction = 'Y'
 	else:  #Right turning
 		robot_drive.move_direction = 'X'

	# put more detailed spped definitioan
	#if(abs(degree_to_turn) < angle_lowest_speed):
	#	robot_drive.speed_now 		= robot_drive.speed_lowest
	#	robot_drive.desired_speed 	= robot_drive.speed_lowest
	#elif(abs(degree_to_turn) < angle_lower_speed):
	#	robot_drive.speed_now 		= robot_drive.speed_lower
	#	robot_drive.desired_speed 	= robot_drive.speed_lower
	#else:
	#	robot_drive.speed_now 		= robot_drive.speed_full
	#	robot_drive.desired_speed 	= robot_drive.speed_full

	#added by aaron to handle the slow communication
	robot_drive.speed_now 		= turn_lowest_speed
	robot_drive.speed_desired 	= turn_lowest_speed

	if robot_drive.robot_turning and not robot_drive.robot_on_mission:
		robot_drive.robot_on_mission = True
		degree_turned = 0
		rospy.loginfo("----------------- Started: Degree turned %d, degree to turn %d -----------------", degree_turned, degree_to_turn)
	else:
		rospy.loginfo("Sending command to let robot start turn")
		robot_drive.start()
		robot_drive.roll_start = robot_drive.roll

# tell the robot to complete it's turning job
def stop_turn():
	global degree_turned
	global degree_to_turn
	# if not robot_drive.robot_turning and not robot_drive.robot_moving:
	# if (robot_job.no_normal_jobs() >= 1 and robot_job.job_lists[1].value >= 0) or (not robot_drive.robot_turning and not robot_drive.robot_moving):	#chengyuen21/7
	if not robot_drive.robot_turning and not robot_drive.robot_moving:
		diff = robot_drive.roll - robot_drive.roll_start
		rospy.loginfo("Calculated roll difference %f, degreed turned %f", diff, degree_turned)
		robot_drive.robot_on_mission = False
		degree_turned = 0
		degree_to_turn = 0
		rospy.loginfo('----------------- Robot completed a turn job -----------------')
	else:
		robot_drive.stop_robot()

# change the speed of turing
def continue_turn(step_angle):
	global degree_turned
	global degree_to_turn

	global angle_lower_speed
	global angle_lowest_speed

	global turn_full_speed
	global turn_lower_speed
	global turn_lowest_speed

	if not robot_drive.robot_turning:
		rospy.loginfo("Robot stopped during the mission, start to turn again")
		robot_drive.start()

	#if(abs(degree_to_turn) - abs(degree_turned) < angle_lowest_speed):
	#	robot_drive.desired_speed = 4
	#	rospy.loginfo("Only 2 degrees left, redusing turning speed to 3")
	#elif(abs(degree_to_turn) - abs(degree_turned) < angle_lower_speed):
	#	robot_drive.desired_speed = 5
	#	rospy.loginfo("Only 5 degrees left, redusing turning speed to 4")
	if (abs(degree_to_turn) - abs(degree_turned) < angle_lowest_speed or abs(degree_turned) < angle_lowest_speed) :
		robot_drive.speed_desired = turn_lowest_speed
	elif (abs(degree_to_turn) - abs(degree_turned) < angle_lower_speed or abs(degree_turned) < angle_lower_speed):
		robot_drive.speed_desired = turn_lower_speed
	else:
		robot_drive.speed_desired = turn_full_speed


	#dynamically update robot bearing
	#robot_drive.bearing_now  = correct_angle(robot_drive.bearing_now)
	if(robot_drive.speed_desired != robot_drive.speed_now ):
		robot_drive.change_speed()

	# robot_publisher.publish_command(robot_drive.move_direction, robot_drive.speed_now)

# main functions let robot performs a turning job of certain degree
def turn_degree():
 	global degree_turned
 	global degree_to_turn
 	global turn_amend
	global first_angle
	global last_angle

 	if not robot_drive.robot_on_mission:
 		degree_to_turn = robot_drive.bearing_target - robot_drive.bearing_now

 		if (degree_to_turn > 180.0):
 			degree_to_turn = degree_to_turn - 360.0
 		elif (degree_to_turn < -180):
 			degree_to_turn = degree_to_turn + 360.0

		if abs(degree_to_turn) < robot_correction.min_correction_angle:
			rospy.loginfo("Degree to turn %d < %d",  degree_to_turn, robot_correction.min_correction_angle)
			return True
		else:
			start_turn()
		return False

	# convered from angle to required turn angles
 	# calculate the obsolute anlge

 	# The degree passed is not correct, just log and return
	if(degree_to_turn == 0):
		#No turn is required, clear current job and rerun
		rospy.logwarn('Robot has been assigned a meaning less 0 degree turn task')
		stop_turn()
		return not robot_drive.robot_on_mission

	#Get the turned angle and then calculate
	step_angle = robot_drive.step_angle
	#robot_drive.bearing_now = robot_drive.bearing_now + step_angle
	#rospy.loginfo("To turn %f, Turned %f, bearing_now %f, bearing_target %f", degree_turned, abs(degree_to_turn), robot_drive.bearing_now, robot_drive.bearing_target)

	degree_turned = degree_turned + step_angle
    # 1 step before the robot turn, stop the robot
	degree_threshold = abs(degree_to_turn) - robot_correction.min_correction_angle/2.0
	#bearing_lower_threshold = robot_drive.bearing_target - (robot_correction.min_correction_angle/2.0)
	#bearing_upper_threshold = robot_drive.bearing_target + (robot_correction.min_correction_angle/2.0)
	#degree_threshold = 5
	#simple log for tracing
	#distpub = 'To turn:%f Turned:%f Step angle: %f' % (degree_to_turn, degree_turned, step_angle)
	#rospy.loginfo(distpub)
	if robot_drive.show_log:
		rospy.loginfo("To turn %f, Turned %f, Step %f, Bearing %f, Target %f", degree_to_turn, degree_turned, step_angle, robot_drive.bearing_now, robot_drive.bearing_target)

	# yuqing_Jul30
	# bearing = 0.0
	# cur_angle = 0.0

	# if(robot_job.job_lists[0].classfication == 'U'):
	# 	if(abs(degree_turned) < degree_threshold):
	# 		continue_turn(step_angle)
	# 		return False
	# 	else:
	# 		if (len(robot_job.job_lists) > 1 and robot_job.job_lists[1].description == 'F' and robot_job.job_lists[1].classfication == 'N'):
	# 		 	robot_job.amend_regular_jobs(robot_drive.lon_now, robot_drive.lat_now, robot_job.job_lists[1].lon_target, robot_job.job_lists[1].lat_target)
	# 		 	# robot_correction.distance_correction(robot_drive.lon_now, robot_drive.lat_now, robot_drive.bearing_now, robot_job.job_lists[1].lon_target, robot_job.job_lists[1].lat_target, robot_job.job_lists[1].bearing_target, 'N')
	# 		stop_turn()
	# 		return not robot_drive.robot_on_mission
	# else:
	# 	if (len(robot_job.job_lists) > 1 and robot_job.job_lists[1].description == 'F'):
	# 		bearing = gpsmath.bearing(robot_drive.lon_now, robot_drive.lat_now, robot_job.job_lists[1].lon_target, robot_job.job_lists[1].lat_target)
	# 		rospy.loginfo("Bearing now %f, bearing target %f", robot_drive.bearing_now,  bearing)
	# 		# cur_angle 	= gpsmath.format_bearing( bearing -  robot_drive.bearing_now )
	# 		cur_angle = bearing - robot_drive.bearing_now

	# 		if cur_angle > 180.0:
	# 			cur_angle = cur_angle - 360.0
	# 		elif cur_angle < -180.0:
	# 			cur_angle = cur_angle + 360.0

	# 		rospy.loginfo("cur_angle: %f", abs(cur_angle))

	# 		if (abs(cur_angle) - 3.5) <= 0.0:
	# 			robot_job.amend_regular_jobs(robot_drive.lon_now, robot_drive.lat_now, robot_job.job_lists[1].lon_target, robot_job.job_lists[1].lat_target)
	# 			stop_turn()
 #            			return not robot_drive.robot_on_mission

	# 		if abs(last_angle - 370) > 0.01 and abs(first_angle - 370.0) > 0.01:
	# 			if (last_angle - cur_angle) * (last_angle - first_angle) > 0:
	# 				rospy.logwarn("first_angle: %f, last_angle: %f, cur_angle: %f", first_angle, last_angle,  cur_angle)
	# 				stop_turn()
	# 				robot_job.remove_job(1);
	# 				return not robot_drive.robot_on_mission

	# 		first_angle = last_angle
	# 		last_angle = cur_angle

	# 		continue_turn(step_angle)
	# 		return False

	# 	else:
	if(abs(degree_turned) < degree_threshold):
		continue_turn(step_angle)
		return False
	else:
		if robot_job.current_job().classfication == 'N': # or robot_job.current_job().classfication == 'C':
			try:
				if robot_job.job_lists[1].description == 'F':
					turn_amend = True
			except IndexError:
				pass
		rospy.loginfo("-----------------bearing_now: %f", robot_drive.bearing_now)
		stop_turn()
		return not robot_drive.robot_on_mission



	#estimate the postition 1 second from now,
	#this assumes the robot only stops 1 second after we start sending stop command
#	if (robot_drive.bearing_estimate >= bearing_lower_threshold and robot_drive.bearing_estimate <= bearing_upper_threshold):
#		rospy.loginfo("now %f, low thresh %f, high thresh %f", robot_drive.bearing_now, bearing_lower_threshold, bearing_upper_threshold)
#		stop_turn()
#		#return not robot_drive.robot_on_mission
#	else:
#		continue_turn(step_angle)
#		return False

	#lower threshold and upper threshold
	#if(robot_drive.bearing_now >= bearing_lower_threshold and robot_drive.bearing_now <= bearing_upper_threshold):
	#	rospy.loginfo("now %f, low thresh %f, high thresh %f", robot_drive.bearing_now, bearing_lower_threshold, bearing_upper_threshold)
	#	stop_turn()
	#	return not robot_drive.robot_on_mission
	#else:
	#	continue_turn(step_angle)
	#	return False

	#return False

