#!/usr/bin/env python

import rospy
import serial
import string
import math
import gpsmath
import robot_drive
import robot_job
import robot_correction

robot_on_obstacle 	 	= False # if robot is on obstacle avoidence, it would be set to 1
robot_over_obstacle 	= False # it's effective if the robot_on_obstacle

needForward 			= False
justStop 		 		= False

#status flags
on_obstacle  				= 0 #when obstacle avoidance finish = 1, else 0
has_obstacle 				= 0

reverse_sensor_ok 		 	= 1
distance_sensor_ok 			= 1

front_sensor				= (0, 0, 0, 0)
back_sensor					= (0, 0, 0, 0)

from math import radians, cos, sin, asin, sqrt, atan2, degrees

#-------------------------------------------------------#
#	Robot error correction module						#
#-------------------------------------------------------#
# while robot's moving, dynamically update robot gps
# still need to handle more scenarios

def obstacle_avoidance_do_nothing():
	global robot_on_obstacle
	global robot_over_obstacle
	robot_on_obstacle = False
	robot_over_obstacle = False
	robot_drive.isunlockdone = True

def is_on_obstacle_avoidance(first, second, third, forth):
	if(first == 0 or first == 1 or first == 2 or first == 3):
		return 1
	if(second ==0 or second == 2 or second ==3 or second == 1):
		return 2
	if(third ==0 or third == 2 or third ==3 or third == 1):
		return 3
	if(forth ==0 or forth == 2 or forth ==3 or forth == 1):
		return 4
	return 0

def start_obstacle_avoidance():
	rospy.loginfo('start_obstacle_avoidance')
	global robot_on_obstacle
	global robot_over_obstacle
	robot_on_obstacle 		= True
	robot_over_obstacle 	= False
	#yuqing_unlockconfirm
	robot_drive.isunlockdone = False

# if obstacle avoidence is over
def obstacle_is_over():
	rospy.loginfo('obstacle_is_over')
	global robot_on_obstacle
	global robot_over_obstacle
	robot_on_obstacle 	= False
	robot_over_obstacle = True

#this will keep sending if listener doesn't get "UNLOCK"
def unlock_from_obstacle():
 	rospy.loginfo('sending unlock_from_obstacle')
	robot_drive.unlock_robot()
	#global robot_over_obstacle
	#robot_over_obstacle = False

#added by aaron, 11May, to indicate obstacle avoidance has ended
def obstacle_avoidance_ended():
	global robot_over_obstacle
	robot_over_obstacle = False
	robot_drive.isunlockdone = True

	# Get the last four digit of the reverse car sendor data
def rc_sensor_data(rc_sensor_value):
	first_digit = rc_sensor_value % 16
	rc_sensor_value = rc_sensor_value / 16
	second_digit = rc_sensor_value % 16
	rc_sensor_value = rc_sensor_value / 16
	third_digit = rc_sensor_value % 16
	rc_sensor_value = rc_sensor_value / 16
	forth_digit = rc_sensor_value % 16
	return first_digit, second_digit, third_digit, forth_digit

def clear_correction_trial_tasks(current_job_type):
	rospy.loginfo("Robot correction trial %d failed, perform clearing tasks", robot_correction.correction_count)
	robot_job.clear_correction_jobs()

def quit_obstacle_correction(current_job_type):
	clear_correction_trial_tasks(current_job_type)
	rospy.loginfo("Quitting correction")
	if robot_job.has_jobs_left() > 0:
		job_executing 		= robot_job.current_job()
		current_job_type 	= job_executing.classfication;
		rospy.loginfo("Add correction for next %s job", current_job_type)
		robot_job.complete_current_job()

def clear_after_obstacle_avoidance(current_job_type):
	# Remove the un-finished job
	if(current_job_type == 'N' or current_job_type == 'C'):
		rospy.loginfo("Robot met obstacle during normal job, finishing current job")
		robot_correction.correction_count 	= 0
		robot_job.complete_current_job()
	elif(current_job_type == 'O'):
		if(robot_correction.correction_count  > robot_correction.max_correction_run):
			quit_obstacle_correction(current_job_type)
		else:
			clear_correction_trial_tasks(current_job_type)
	else:
		rospy.logerr("Invalid job_type found")


def resume_from_obstacle_avoidance():
	job_executing = robot_job.current_job()
	current_job_type = job_executing.classfication;

	# robot is resumed to clear state and ready for the correction tasks
	# performing necessary clearing of current tasks
	clear_after_obstacle_avoidance(current_job_type)

	if needForward:
		rospy.loginfo("Need to forward 0.5m after obstacle")
		robot_correction.dist_correction_obstacle_need_forward(robot_job.dist_forward_after_obstacle)
	else:
		robot_correction.dist_correction_obstacle()
		rospy.loginfo("no need to forward 0.5m after obstacle")

	#robot_drive.isunlockdone = False

# Complete the obstacle avoidence after we get a signal from the robot base
def complete_obstacle_avoidance():
	global robot_over_obstacle
	# step 1: Waiting for robot stop moving
	if (robot_drive.robot_turning or robot_drive.robot_moving):
		rospy.loginfo("waiting robot to stop")
		return

	# step 2: Once robot stopped moving, send unlock signal to the robot
	if (not robot_drive.isunlockdone):
		rospy.loginfo("unlock robot for further processing")
		robot_drive.unlock_robot()
		return

	# step 3: Once robot is unlocked, resume control to the program
	# Need to perform necessary correction
	rospy.loginfo("Resume the robot from obstacle avoidence")
	# First get ready the robot for normal walking
	#yuqing_unlockconfirm
	#robot_obstacle.unlock_from_obstacle()
	# Remove the un-finished job
	if robot_drive.robot_on_mission and robot_job.has_jobs_left():
		resume_from_obstacle_avoidance()
	else:
		rospy.loginfo("There's no mission on going")

	robot_over_obstacle = False



	# if(current_job_type == 'N'):
	# 		rospy.loginfo("Rbot met obstacle during normal job, pefrorm correction")
	# 		robot_correction.correction_count 	= 0
	# 		robot_job.complete_current_job()
	# 		# Re-calculate and send the corretion job
	# 		#robot_correction.distance_correction(dist_forward_after_obstacle)
	# 		#@yuqing_forwardafterobstacle
	# 		#forward distance by angle from sensor
	# 		#rospy.loginfo("forward 0.5m after obstacle")
	# 	elif(current_job_type == 'O'):
	# 		if(robot_correction.correction_count  > robot_correction.max_correction_run):
	# 			rospy.loginfo("Robot has tried to move to %f, %f for %d times, failed")
	# 			while (current_job_type == 'O'):
	# 				robot_job.complete_current_job()
	# 				if len(robot_job.job_lists) > 0:
	# 					job_executing 		= robot_job.current_job()
	# 					current_job_type 	= job_executing.classfication;
	# 				else:
	# 					rospy.loginfo("The last job in the queue")
	# 					break;

	# 			rospy.loginfo("Cleared all the correction jobs")
	# 			if len(robot_job.job_lists) > 0:
	# 				job_executing 		= robot_job.current_job()
	# 				current_job_type 	= job_executing.classfication;
	# 				rospy.loginfo("Add correction for next %s job", current_job_type)
	# 				robot_job.complete_current_job()
	# 		else:
	# 			rospy.loginfo("Robot meet a obstacle while peforming correction job")
	# 			robot_correction.correction_count = robot_correction.correction_count + 1
	# 			rospy.loginfo("Robot failed correction job for %d time", robot_correction.correction_count )
	# 			#discard all correction jobs
	# 			while (current_job_type == 'O'):
	# 				robot_job.complete_current_job()
	# 				if len(robot_job.job_lists) > 0:
	# 					job_executing 		= robot_job.current_job()
	# 					current_job_type 	= job_executing.classfication;
	# 				else:
	# 					rospy.loginfo("The last job in the queue")
	# 					break;
	# 	else:
	# 		rospy.logerr("Invalid job_type found")
