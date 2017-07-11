#!/usr/bin/env python
import rospy
import serial
import string
import time
import robot_publisher
import robot_correction
import math

from std_msgs.msg import String
from math import radians, cos, sin, asin, sqrt, atan2, degrees
#-------------------------------------------------------#
#	Robot drive module									#
#-------------------------------------------------------#

linear_encode_to_mm 		 	= 0.0
turning_encode_to_mm  			= 0.0
turn_radius 					= 0.0

lon_now 						= 0.0
lat_now  						= 0.0
bearing_now 					= 0.0

lon_target 						= 0.0
lat_target 						= 0.0
bearing_target 					= 0.0 		#degrees

speed_6 			= 6
speed_5 			= 5
speed_4 			= 4
speed_3 			= 3
speed_2 			= 2

roll 				= 0.0
pitch 				= 0.0
yaw 				= 0.0
past_yaw 			= 0.0

step_angle 			= 0.0
step_distance		= 0.0

direction 			= "stop"
past_direction 		= "stop"

face_detected 		= 0
face_x 				= 0.0
face_y 				= 0.0
face_w 				= 0.0
face_h 				= 0.0

#-----------------FLAGS------------------#

speed_desired 		= speed_6	# Global robot moving spped, 3 - 5
speed_now 			= speed_6	# Robot moving speed now
robot_on_mission 	= False			# set an indicator that robot's on a job right now
robot_enabled 		= False 		# A switch to enable or disable robot from execuing any jobs
robot_paused 		= False 		# A flag to indicate enable robot to execute job
move_direction 		= 'F'			# Global robot walking/turning direction
robot_initialized	= True 			# Confirm whehter the robot has initialized the true north
robot_moving 		= False			# based on the encoder data to know whether the robot's moving
robot_turning		= False


# Flags indicate the mode desired to minimize the startup keyboard entries
obstacle_mode_desired 	= False
burn_mode_desired 		= False
robot_enabled_desired 	= True

#yuqing_obstaclemodeconfirm
#1: obstacle mode
#0: no-obstalce mode
obstacle_mode = True
#yuqing_unlockconfirm
isunlockdone = False
# Jianbo: Burn mode and normal mode
# True for burn mode when pwer up
# False for normal mode
burn_mode = True

#battery percentage, updated by callback
battery_level  			 	= 100

#status flags
manual_mode 	 			= False
interaction_mode 			= False

motor_1_ok 	 				= True
motor_2_ok 	 	 			= True
encoder_ok 					= True
gyroscope_ok 				= True

show_log					= False
turn_on_light				= False
is_light_on					= False
robot_interacting			= False

roll_start 				= 0.0 

bluetooth_panel 			= ''


############################################################

# start the robot movement or turning
def start():
	global speed_now
	global move_direction
	robot_publisher.publish_command(move_direction, speed_now)

# Get all the encoder not processed
def accum_encoder_data(encoder_data, encoder_received, encoder_processed):
	left_encode = 0
	right_encode = 0
	#in case data received is faster than the processing time
	if(encoder_received > encoder_processed):
		for x in range(encoder_processed, encoder_received):
			left_encode += encoder_data[2 * x]
			right_encode += encoder_data[2 * x +1]

	if(encoder_received < encoder_processed):
		for x in range(encoder_processed, 1000):
			left_encode 	+= encoder_data[2 * x]
			right_encode 	+= encoder_data[2 * x + 1]
			for x in range(0, encoder_received):
				left_encode 	+= encoder_data[2 * x]
				right_encode 	+= encoder_data[2 * x + 1]
	return left_encode, right_encode

# just send a command to stop robot
def stop_robot():
	speed_now = 0
	desired_speed = 0
	move_direction = 'K' #P
	# move_direction = 'S'
	# updated the stop command from 'S' to 'P'
	#while True:
	#	#step 1, send the stop command every 10 milli seoncs
	#	if(robot_moving or robot_turning):
	#		robot_publisher.publish_command(move_direction,speed_now)
	#		time.sleep(0.01)
	#	else:
	#		break
	robot_publisher.publish_command(move_direction, speed_now)

def unlock_robot():
	# send a command to unlock robot after obstacle avoidece
	robot_publisher.pub_command.publish('SB00000BE\n')
	#robot_publisher.pub_command.publish('obstacle unlock')
	rospy.loginfo('Command sent to unlock robot after avoidancee ')

# change speed
def change_speed():
	global move_direction, speed_now, speed_desired
	robot_publisher.publish_command(move_direction, speed_desired)
	distpub = 'Command sent to change speed from %d to %d' % (speed_now, speed_desired)
	speed_now = speed_desired
	rospy.loginfo(distpub)

#@yuqing_toggleobstaclemode
def enter_no_obstacle():
	robot_publisher.pub_command.publish('SW00000WE\n')
	#robot_publisher.pub_command.publish('no obstacle mode')
	rospy.loginfo('Command sent to enter no obstacle mode')

#@yuqing_toggleobstaclemode
def enter_obstacle():
	robot_publisher.pub_command.publish('SO00000OE\n')
	#robot_publisher.pub_command.publish('obstacle mode')
	rospy.loginfo('Coammand sent to enter obstacle mode')
	#yuqing_obstaclemodeconfirm
	#remove unlock

def turn_on_lights():
	robot_publisher.pub_command.publish('SO00000PE\n')
	rospy.loginfo('Command sent to turn lights on ')

def turn_off_lights():
	robot_publisher.pub_command.publish('SC00000PE\n')
	rospy.loginfo('command sent to turn lights off')

#normal and burn mode's commands are written in serial_handler_node
def enter_normal_mode():
	#for i in range (0,3):
	robot_publisher.pub_command.publish('iap_jump_app\r\n\0')
	rospy.loginfo('Command sent to switch to normal mode')

def enter_burn_mode():
	#for i in range (0,5):
	robot_publisher.pub_command.publish('iap\r\n\0')
	rospy.loginfo('Command sent to switch to burn mode')

def change_mode():
	if burn_mode_desired:
		enter_burn_mode()
	else:
		enter_normal_mode()

def change_obstacle_mode():
	if obstacle_mode_desired:
		enter_obstacle()
	else:
		enter_no_obstacle()

def change_light_mode():
	if turn_on_light:
		turn_on_lights()
	else:
		turn_off_lights()
