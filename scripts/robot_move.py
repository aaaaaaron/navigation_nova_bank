#!/usr/bin/env python
import rospy
import serial
import string
from std_msgs.msg import String
import robot_drive
import robot_correction
import gpsmath
import robot_job

#-------------------------------------------------------#
#	Robot moving module									#
#-------------------------------------------------------#

dist_completed 			= 0.0
dist_to_run 			= 0.0
angle_to_correct 		= 0.0
#@yuqing_correctionper10m
dist_to_correct 		= 0.0
dist_lowest_speed 		= 0.0
dist_lower_speed 		= 0.0
#dist_end_point_check 	= 600.0 #was 600

linear_full_speed 		= 0.0
linear_lower_speed 		= 0.0
linear_lowest_speed		= 0.0

status_pub = rospy.Publisher('status', String, queue_size = 100)
# Starts the robot for moving, put the control variables into proper value
def start_move():
	global dist_completed
	global dist_to_run

	global dist_lowest_speed
	global dist_lower_speed

	global linear_full_speed
	global linear_lower_speed
	global linear_lowest_speed

	rospy.loginfo("Lowest %d, Lower %d, Normal %d", linear_lowest_speed, linear_lower_speed, linear_full_speed)

	# if the task is a short distance task, then start with a lower speed
	robot_drive.speed_now  		= linear_lowest_speed
	robot_drive.speed_desired 	= linear_lowest_speed

    # only if the robot starts to move then change the status
	if robot_drive.robot_moving:
		robot_drive.robot_on_mission = True
		dist_completed = 0.0
		angle_to_correct = 0.0
		rospy.loginfo('----------------- Started a moving job -------------------')
	else:
	  	robot_drive.start()
		rospy.loginfo('Starting a moving job')

# Roboet complet a moving job
def stop_move():
	global dist_completed
	if not robot_drive.robot_moving :
		dist_completed = 0.0
		robot_drive.robot_on_mission = False
		rospy.loginfo('----------------- Robot completed a moving job -----------------')
	else:
		rospy.loginfo('Robot still moving, stopping robot')
		robot_drive.stop_robot()

# Update robot speed as required new speed
def continue_move():
	global dist_to_run
	global dist_completed
	global dist_lowest_speed
	global dist_lower_speed

	global linear_full_speed
	global linear_lower_speed
	global linear_lowest_speed

	# if robot's on mission and somehow it's stopped, need to restart the robot
	if not robot_drive.robot_moving:
        	rospy.loginfo("Robot stopped during the mission, start to move again")
        	robot_drive.start()

    # if robot is approaching the destination, then need to decrease speed
	if(abs(dist_to_run) - abs(dist_completed) < dist_lowest_speed or abs(dist_completed) < dist_lowest_speed):
		robot_drive.speed_desired = linear_lowest_speed
		#rospy.loginfo('Reducing to lowest speed, very close to target position')
	elif(abs(dist_to_run) - abs(dist_completed) < dist_lower_speed or abs(dist_completed) < dist_lower_speed):
		robot_drive.speed_desired = linear_lower_speed
		#rospy.loginfo('Reducing to lower speed, %f to target position', dist_lower_speed)
	else:
		robot_drive.speed_desired = linear_full_speed

	#if (abs(dist_to_run) - abs(dist_completed) < dist_2_speed):
	#	#robot_drive.speed_now 		= robot_drive.speed_2
	#	robot_drive.speed_desired 	= robot_drive.speed_2
	#elif (abs(dist_to_run) - abs(dist_completed) < dist_3_speed):
	#	#robot_drive.speed_now 		= robot_drive.speed_3
	#	robot_drive.speed_desired 	= robot_drive.speed_3
	#elif (abs(dist_to_run) - abs(dist_completed) < dist_4_speed):
	#	#robot_drive.speed_now 		= robot_drive.speed_4
	#	robot_drive.speed_desired 	= robot_drive.speed_4
	#else:
	#	#robot_drive.speed_now 		= robot_drive.speed_6
	#	robot_drive.speed_now 		= robot_drive.speed_6


    # if change of speed request is received
	if(robot_drive.speed_now != robot_drive.speed_desired):
        	robot_drive.change_speed()

# main function to control the robot movement
def move_distance(dist):
	global dist_completed
	global dist_to_run
	global angle_to_correct

	dist_to_run = dist
	# if robot received a meaning less job, just signal, clear the job and return
	if (abs(dist_to_run) < robot_correction.min_correction_distance):
		stop_move()
		rospy.logwarn('Robot received a meaningless moving job')
		return True

	#check the move direction
	if (dist_to_run < 0.0):
		robot_drive.move_direction = 'B'
	else:
		robot_drive.move_direction = 'F'

	# Mission started, let robot start moving
	if not robot_drive.robot_on_mission:
		rospy.loginfo("robot move start move")
		start_move()
		return False

	abs_dist_to_run = abs(dist_to_run)
	# Accumulate the running distance and check
	# Get each step of the distance
	dist_step = robot_drive.step_distance
	# accumulate the distance to the completed distance
	dist_completed = dist_completed + abs(dist_step)   #this is in mm
	# robot is with in the range, then we condidered robot reached the position
	dist_threshold = abs_dist_to_run - robot_correction.min_correction_distance/2 	#0 mm, I can choose -50mm, but since there will be inefficiencies, 0 error threshold might be good enough

	if robot_drive.show_log:
		distpub = 'speed %d, Dist-travelled: %f dist-total:%f dist-step:%f' % (robot_drive.speed_now, dist_completed, abs(dist_to_run) ,dist_step)
		rospy.loginfo(distpub)
	angle_to_correct = angle_to_correct + robot_drive.step_angle
	if robot_drive.show_log:
		rospy.loginfo('Accumulated angle error: %f', angle_to_correct)
	#@yuqing_correctionper10m
	#if travel over 10m, job_completed to 1, start to correct
	if (dist_completed >= dist_to_correct):
		rospy.loginfo("-----------------dist_completed: %f, start to correct", dist_completed)
		stop_move()
		return not robot_drive.robot_on_mission

	dist_remain = dist_threshold - dist_completed;

	#Check current point's distnace and angle to the target end point
	# if dist_remain > abs_dist_to_run/3:
	# 	distance_to_target, angle_to_target = robot_correction.distance_bearing_to_target()
	# 	if angle_to_target < 300 and angle_to_target > 60:
	# 		stop_move()
	# 		return not robot_drive.robot_on_mission
	# 	if distance_to_target < dist_remain / 2:
	# 		stop_move()
	# 		return not robot_drive.robot_on_mission

	if (dist_remain > 0.0) :
		#just continue moving of job not completed and no change of speed command received
		#if speed changed, then just change the move speed
		continue_move()
		return  False
	else :
		stop_move()
        # make sure the robot is stopped before next job
        return not robot_drive.robot_on_mission
       	#clean current job
	return False
