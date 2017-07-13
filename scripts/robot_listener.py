# holds all kinds of call back etc
#!/usr/bin/env python
import rospy
import string
import time
import gpsmath
import json
import robot_obstacle
import robot_job
import robot_drive
import robot_move
import robot_turn
import robot_correction
import robot_publisher
import robot_listener
import json
import math
import robot_configure
import webbrowser
from datetime import datetime
from std_msgs.msg import String
from geometry_msgs.msg import Vector3
from serial_handler.msg import Status
from serial_handler.msg import Encoder
from serial_handler.msg import Sonar

import os

#used to hold the encoder data received, init with some value
encoder_buffer_size 	= 1000 	# The buffer size for the encoder
encoder_data 		= []	# pairs of encoder data , l, r etc
compass_size 		= 10 	# The compas data buffer size
compass_data 		= [] 	# Hold the compass dat which [0 - 359]
# two index indicate the processing and receiving index of the encoder data
encoder_received	= 0 	# The index of the encoder received
encoder_processed 	= 0		# The index of the encoder signal processed
compass_index 		= 0 	# current compass index
last_process_time 	= 0 	# last processing time
max_delay 			= 1.0	# max delay allowed for not receiving any signal from encooder
last_received_time 	= 0.0 	# the time of receiving the last encoer data


def sonar_callback(data):
	robot_obstacle.front_sensor[0] = data.front_0;
	robot_obstacle.front_sensor[1] = data.front_1;
	robot_obstacle.front_sensor[2] = data.front_2;
	robot_obstacle.front_sensor[3] = data.front_3;
	robot_obstacle.back_sensor[0] = data.back_0;
	robot_obstacle.back_sensor[1] = data.back_1;
	robot_obstacle.back_sensor[2] = data.back_2;
	robot_obstacle.back_sensor[3] = data.back_3;

	if (robot_drive.show_log):
		rospy.log("f: %d, %d, %d, %d, b: %d, %d, %d, %d", data.front_0, data.front_1, data.front_2,data.front_3, data.back_0, data.back_1, data.back_2, data.back_3)
	return;


def IMU_callback(data):
	#store the past value first
	robot_drive.past_yaw  	= robot_drive.yaw
	if(robot_drive.show_log):
		diff_x = -robot_drive.roll + data.x
        	diff_y = -robot_drive.pitch + data.y
        	diff_z = -robot_drive.yaw + data.z

		rospy.loginfo("values (%f, %f, %f)", diff_x, diff_y, diff_z)
	robot_drive.roll  	= data.x
	robot_drive.pitch 	= data.y
	robot_drive.yaw 	= data.z


def serial_encoder_callback(data):
	global encoder_data
	global encoder_received
	robot_drive.burn_mode = False
	#accumulate encoder data
	#Step 1: Get encoder data and convert them to number for later use
	#Get left encoder and right encoder
	left_encode  = data.left_encoder
	right_encode = data.right_encoder

	multiply = left_encode * right_encode

	if left_encode == 0 and right_encode == 0:
		#rospy.loginfo("encoder 0,0")
		robot_drive.robot_moving 	= False
		robot_drive.robot_turning 	= False
	elif multiply > 25:
		#rospy.loginfo("encoder 0.1")
		robot_drive.robot_moving 	= True
		robot_drive.robot_turning 	= False
	elif multiply < -25:
		#rospy.loginfo("encode 1.1")
		robot_drive.robot_turning 	= True
		robot_drive.robot_moving 	= False
	elif multiply < 10:
		robot_drive.robot_turning 	= False
		robot_drive.robot_moving 	= False
		rospy.logwarn("The encoder is changing on a small value")
	elif left_encode != 0 or right_encode != 0:
		#rospy.loginfo("encoder 1.3")
		robot_drive.robot_turning = True
		robot_drive.robot_moving = True
	index = encoder_received * 2
	encoder_data[encoder_received * 2] = float(left_encode)
	encoder_data[encoder_received * 2 + 1] = float(right_encode)

	encoder_received = (encoder_received + 1) % 1000
	#convert encoder number to floading point number, make sure all subsquent calculation is on floating point mode
	#if (robot_drive.robot_on_mission ==1 ):
	#	rospy.loginfo(str(data_string))

def face_detection_callback(data):
	data_string = data.data
	robot_drive.face_detected, robot_drive.face_x, robot_drive.face_y, robot_drive.face_w, robot_drive.face_h = data_string.split(",")

def status_callback(data):
	robot_drive.burn_mode				= False
	robot_obstacle.on_obstacle	 		= data.over_obstacle #when over obstacle avoidance, = 1, else 0
	robot_drive.manual_mode				= data.manual_mode
	robot_drive.obstacle_mode 			= data.obstacle_avoidance_mode
	robot_obstacle.has_obstacle 		= data.has_obstacle
	robot_drive.interaction_mode 		= data.interaction_mode

	robot_drive.motor_1_ok 				= data.motor_1_ok
	robot_drive.motor_2_ok 				= data.motor_2_ok
	robot_drive.encoder_ok 				= data.encoder_ok
	robot_drive.gyroscope_ok 			= data.gyroscope_ok
	robot_obstacle.reverse_sensor_ok 	= data.reverse_sensor_ok
	robot_obstacle.distance_sensor_ok 	= data.distance_sensor_ok
	robot_drive.is_light_on				= data.light_on

	if (data.obstacle_avoidance_mode and data.has_obstacle): 	#when obstacle mode on, and finds obstacle
		robot_obstacle.robot_on_obstacle = True
		robot_obstacle.robot_over_obstacle = False
	elif (data.obstacle_avoidance_mode and data.over_obstacle): 	#when obstacle avoidance is over
		robot_obstacle.robot_on_obstacle = False
		robot_obstacle.robot_over_obstacle = True
	else: 													 #when no obstacle,
		robot_obstacle.robot_on_obstacle = False
		robot_obstacle.robot_over_obstacle = False

	robot_drive.battery_level = data.battery_level
	if(robot_drive.battery_level >= 20):
		robot_job.back_to_base_mode = False

	data_int  	= data.direction
	if (data_int == 0):
		robot_drive.direction = "stop"
	elif (data_int == 1):
		robot_drive.direction = "forward"
	elif (data_int == 2):
		robot_drive.direction = "backward"
	elif (data_int == 3):
		robot_drive.direction = "left"
	elif (data_int == 4):
		robot_drive.direction = "right"

#aaron added globals


# # handle the data from the front reverse car sensor
# def rc_sensor_f_callback(data):
# 	str_right = data.data[-5:]
# 	str_right = str_right.strip('E')
# 	#@yuqing_obstacledriverread
# 	#if robot_obstacle.robot_on_obstacle:
# 		#if(str_right == 'CESO'):
# 			#robot_obstacle.obstancle_is_over()
# 	#else:
# 	#rc_sensor_front = int(str_right, 16)
# 	#first, second, third, forth = robot_obstacle.rc_sensor_data(rc_sensor_front)
# 	#ret = robot_obstacle.is_on_obstacle_avoidence(first, second, third, forth)
# 	#if ret > 0:
# 	#	robot_obstacle.start_obstacle_avidence()
# 	return
#-------------------- communicate with web interface

def chat_callback(data):
	json_str = str(data.data)
	rospy.loginfo(json_str)
	try:
		decoded = json.loads(json_str)
		chat_type = decoded('TYPE')
		chat_action = decoded('ACTION')
		rospy.loginfo("TYPE %d, ACTION %d", chat_type, chat_action)
		if(chat_type == 1 and chat_action == 0):
			robot_drive.robot_paused		  	= False
			robot_drive.robot_interacting		= False
			robot_drive.obstacle_mode_desired 	= True

	except(ValueError, KeyError, TypeError):
		rospy.loginfo('JSON format error')




def communicate_callback(data):

	json_str 	= str(data.data)
	rospy.loginfo(json_str)
	try:
		decoded 	= json.loads(json_str)
		url 		= decoded['url']
		my_id 		= decoded['robot_id']
		web_id 		= decoded['control_id']
		# Open URL in a new tab, if a browser window is already open.
		#webbrowser.register('mozilla', Mozilla('mozilla'))
		webbrowser.open_new(url + '?robotid=' +  + ';web_id=' + web_id)
	except (ValueError, KeyError, TypeError):
		rospy.loginfo('JSON format error:')
		rospy.loginfo(json_str)

def control_callback(data):
	json_str = str(data.data)
	# clear all the existing jobs
	del robot_job.gps_lon[:]
	del robot_job.gps_lat[:]
	try:
		decoded = json.loads(json_str)
		bearing = decoded['bearing']
		robot_job.append_turn_job(robot_drive.lon_now, robot_drive.lat_now, bearing)
		lon_new, lat_new  = robot_job.append_regular_job(robot_drive.lon_now, robot_drive.lat_now, 100000, bearing)
		rospy.loginfo("Finish generating jobs");
	except (ValueError, KeyError, TypeError):
		rospy.loginfo('JSON format error:')
		rospy.loginfo(json_str)


# handle the data from the job creator from our website, based on the gps corrdicates provided,
# Generate a list of jobs
def job_callback(data):
	json_str = str(data.data)
	# Clear the lontiude and latitude list
	del robot_job.gps_lon[:]
	del robot_job.gps_lat[:]
	try:
		rospy.loginfo("Start to parse job")
		decoded = json.loads(json_str)
		# pretty printing of json-formatted strin
		list_route  = decoded['route']
		rospy.loginfo("Decoded route")
		for item in list_route:
			lon = float(item.get(u'lng'))
			lat = float(item.get(u'lat'))
			robot_job.gps_lon.extend([lon])
			robot_job.gps_lat.extend([lat])
		robot_job.clear_job_list()
		rospy.loginfo("Parsing route successful")
		init_point			= decoded['init_point']
		robot_job.init_lon 		= float(init_point.get(u'lng'))
		robot_job.init_lat 		= float(init_point.get(u'lat'))
		update_base(robot_job.init_lon, robot_job.init_lat)
		rospy.loginfo("Parse init point successful")
		no_runs 			= decoded['run']
		rospy.loginfo("Number of runs %d", int(no_runs))
		robot_job.loops 		= int(no_runs)
		#rospy.loginfo("Parsing successful. No of loops %d", no_runs);
		# after parsing the gps corrdinates, now generate robot jobs
		robot_job.generate_jobs_from_gps()
		rospy.loginfo("Finish generating jobs");
	except (ValueError, KeyError, TypeError):
		rospy.loginfo('JSON format error:')
		rospy.loginfo(json_str)

# # Real time get compass data
# def compass_callback(data):
# 	global compass_data
# 	global compass_index
# 	#update compass_data global variable
# 	compass_data[compass_index] = int(data.data)
# 	#rospy.loginfo("compass index: %d, angle: %d", compass_index, compass_data[compass_index])
# 	compass_index = (compass_index + 1) % compass_size


# # handle the data from the back reverse car sensor
# def rc_sensor_b_callback(data):
# 	str_right = data.data[-5:]
# 	str_right = str_right.strip('E')
# 	#@yuqing_obstacledriverread
# 	#if robot_obstacle.robot_on_obstancle:
# 		#if(str_right == 'CESO'):
# 			#robot_obstacle.obstancle_is_over()
# 	#else:
# 	#rc_sensor_front = int(str_right, 16)

# 	#first, second, third, forth = robot_obstacle.rc_sensor_data(rc_sensor_front)
# 	#ret = robot_obstacle.is_on_obstacle_avoidence(first, second, third, forth)
# 	#if ret > 0:
# 	#	robot_obstacle.start_obstacle_avidence()
# 	return

# The main call back, getting encoder data and make decision for the next move
# The main call back, getting encoder data and make decision for the next move



# def encoder_callback(data):
# 	global encoder_data
# 	global encoder_received
# 	robot_drive.burn_mode = False
# 	#accumulate encoder data
# 	#Step 1: Get encoder data and convert them to number for later use
# 	#Get left encoder and right encoder
# 	data_string = data.data
# 	left_encode, right_encode = data_string.split(" ")

# 	left_encode  = int(left_encode)
# 	right_encode = int(right_encode)
# 	if(left_encode == 0 and right_encode == 0):
# 		#rospy.loginfo("encoder 0,0")
# 		robot_drive.robot_moving 	= False
# 		robot_drive.robot_turning 	= False
# 	elif (left_encode * right_encode > 25):
# 		#rospy.loginfo("encoder 0.1")
# 		robot_drive.robot_moving 	= True
# 		robot_drive.robot_turning 	= False
# 	elif (left_encode * right_encode < -25):
# 		#rospy.loginfo("encode 1.1")
# 		robot_drive.robot_turning 	= True
# 		robot_drive.robot_moving 	= False
# 	elif left_encode != 0 or right_encode != 0:
# 		#rospy.loginfo("encoder 1.3")
# 		robot_drive.robot_turning = True
# 		robot_drive.robot_moving = True
# 	index = encoder_received * 2
# 	encoder_data[encoder_received * 2] = float(left_encode)
# 	encoder_data[encoder_received * 2 + 1] = float(right_encode)

# 		#bytesToLog = 'Encoder sequence %d received' % (encoder_received)
# 		#rospy.loginfo(str(bytesToLog))

# 	encoder_received = (encoder_received + 1) % 1000
# 	#convert encoder number to floading point number, make sure all subsquent calculation is on floating point mode
# 	#if (robot_drive.robot_on_mission ==1 ):
# 	#	rospy.loginfo(str(data_string))

# def encoder_callback_new(data):
# 	robot_drive.burn_mode = False
# 	left_encode 			= data.x
# 	right_encode 			= data.y

# 	if(left_encode == 0 or right_encode == 0):
# 		#rospy.loginfo("encoder 0,0")
# 		robot_drive.robot_moving 	= False
# 		robot_drive.robot_turning 	= False
# 	elif (left_encode * right_encode > 0):
# 		robot_drive.robot_moving 	= True
# 		robot_drive.robot_turning 	= False
# 	else:
# 		robot_drive.robot_turning 	= True
# 		robot_drive.robot_moving 	= False

# Subscriber to keyboard topic and peform actions based on the command get
def keyboard_callback(data):
	keyboard_data = ''
	keyboard_data = data.data
	robot_drive.speed_now = 6
	robot_drive.desired_speed = 6
	if (keyboard_data == 'Init'):
		rospy.loginfo("Testing init job")
		robot_job.initialize_job()
	elif (keyboard_data == "Burn"):
		if robot_drive.burn_mode:
			rospy.loginfo('Received keyboard command to exit from burn mode')
			robot_drive.burn_mode_desired = False
		else:
			rospy.loginfo('Received keyboard command to enter burn mode')
			robot_drive.burn_mode_desired = True
	elif (keyboard_data == 'Pause'):
		rospy.loginfo("Pause the task");
		robot_drive.robot_paused = 1;
		robot_job.pause_robot();
	elif (keyboard_data == 'Resume'):
		rospy.loginfo("Resume the task");
		robot_drive.robot_paused = 0;
	elif (keyboard_data == 'Forward'):
		rospy.loginfo("Command received: Start to move forward 4 m")
		robot_job.simple_move(1000.0, robot_drive.bearing_now, 'F')
	elif (keyboard_data == 'Back'):
		rospy.loginfo("Command received: Start to move backward 4 m")
		robot_job.simple_move(-1000.0, robot_drive.bearing_now, 'B')
	elif (keyboard_data == 'Turn_West'):
		rospy.loginfo("Command received: turn to 270 (WEST)")
		#robot_drive.bearing_now = compass_data[compass_index]
		robot_job.simple_turn(270.0)
	elif (keyboard_data == 'Turn_East'):
		rospy.loginfo('Command received: turn to 90 (EAST)')
		#robot_drive.bearing_now = compass_data[compass_index]
		robot_job.simple_turn(90.0)
	elif (keyboard_data == 'Stop'):
		rospy.loginfo("Comamnd received: Clear all jobs")
		robot_job.clear_job_list()
		robot_drive.robot_on_mission =  0
	elif (keyboard_data == "Switch"):
		if(robot_drive.robot_enabled == 1):
			rospy.loginfo("Received keyboard command to disable robot")
			robot_drive.robot_enabled = 0
			#robot_drive.init_gps()
		else:
			#robot_drive.init_gps()
			rospy.loginfo("Reveived keyboard command to enable robot")
			robot_drive.robot_enabled = 1
	elif (keyboard_data == 'Faster'):
		rospy.loginfo('Command received: Try to increase robot speed')
		if(robot_drive.desired_speed < 6):
			rospy.loginfo('Robot speed increased')
			robot_drive.desired_speed = robot_drive.desired_speed + 1
		else:
			rospy.loginfo('Robot speed already maximized')
	elif (keyboard_data == 'Slower'):
		rospy.loginfo('Command received, trying to reduce robot speed')
		if(robot_drive.desired_speed > 3):
			rospy.loginfo('Robot speed reduced')
			robot_drive.desired_speed = robot_drive.desired_speed - 1
		else:
			rospy.loginfo('Robot speed already minimized')
	elif (keyboard_data == "Demo"):
		#robot_drive.bearing_now = 0
		rospy.loginfo("Simple job")
		robot_job.define_test_job()
	elif (keyboard_data == "Test"):
		robot_job.simple_turn(0.0)
	elif (keyboard_data == "No_obstacle"):#@yuqing_toggleobstaclemode
		rospy.loginfo('Received keyboard command to switch off obstacle avoidance mode')
		robot_drive.obstacle_mode_desired = False
	elif (keyboard_data == "Obstacle"):#@yuqing_toggleobstaclemode
		rospy.loginfo('Received keyboard command to switch on obstacle avoidance mode')
		robot_drive.obstacle_mode_desired = True
	elif (keyboard_data == "10m"):#@yuqing_toggleobstaclemode
		rospy.loginfo('forward 10m')
		robot_job.simple_move(10000.0, robot_drive.bearing_now, 'F')
	elif (keyboard_data == "180"):#@yuqing_toggleobstaclemode
		rospy.loginfo('turn 180')
		robot_job.simple_turn(180.0,)
	elif (keyboard_data == "zero"):#@yuqing_toggleobstaclemode
		rospy.loginfo('turn 0')
		robot_job.simple_turn(0.0)
	elif (keyboard_data == "cfg"):
		print_config()
	elif (keyboard_data == "status"):
		print_status()
	elif keyboard_data == 'log':
		robot_drive.show_log = not robot_drive.show_log
	elif keyboard_data == 'light':
		robot_drive.turn_on_light = not robot_drive.turn_on_light
		robot_drive.change_light_mode()
	else:
		rospy.loginfo(keyboard_data)
		rospy.loginfo("Not recognizing command receivied")

#@yuqing_obstacledriverread
#read obstacle finish data thro driver node
def obstacle_status_callback(data):
	string = data.data
	obstacle_mode, obstacle_detected, obstacle_finish_status = string.split(" ")
	#rospy.loginfo('obstacle callback: %s, length: %d ',  string, len(string))
	#rospy.loginfo('robot_on_obstacle: %d', robot_obstacle.robot_on_obstacle)
	if (obstacle_finish_status == 1): 					#this will only happen when obstacle mode is on
		robot_obstacle.obstacle_is_over()
		#robot_obstacle.unlock_from_obstacle()			#unlock robot from obstacle
	#elif(obstacle_finish_status == 0 and robot_obstacle.robot_over_obstacle == True): 		#robot successfully unlocked
	#	robot_obstacle.obstacle_avoidance_ended() 		#change flags
	elif(obstacle_mode == 1 and obstacle_detected == 0 and obstacle_finish_status == 0):	#if on, no obstacle
		robot_obstacle.obstacle_avoidance_do_nothing()
	elif(obstacle_mode == 1 and obstacle_detected == 1 and obstacle_finish_status == 0):	#if on, detected obstacle
		robot_obstacle.start_obstacle_avoidance()
	else: 												#
		robot_obstacle.obstacle_avoidance_do_nothing()

	return

	#if(string == 'FINISH'):
	#	if robot_obstacle.robot_on_obstacle:
	#		#rospy.loginfo('first finish')
	#		robot_obstacle.obstacle_is_over()
	#	else:
	#		rospy.loginfo('Received one more finish')
			#robot_drive.unlock_robot()
	#else:
		#yuqing_obstaclemodeconfirm
		#if (string == "LOWER CONTROL"):
		#	rospy.loginfo("robot enter no obstacle mode")
		#	robot_drive.obstacle_mode = False
		#elif (string == "UPPER CONTROL"):
		#	rospy.loginfo("robot enter obstacle mode")
		#	robot_drive.obstacle_mode = True
		#yuqing_unlockconfirm
		#below commented away 11th may by aaron ###################
		#elif (string == "UNLOCK"):
		#	robot_drive.isunlockdone = True
		#else:
		#	index = string.find('OBSTACLE')
		#	# if current state is no obstacle but recevited obstacel, starts obstacle avidentce
		#	if (not robot_obstacle.robot_on_obstacle):
		#		if(index != -1):
		#			robot_obstacle.start_obstacle_avoidance()
		#			robot_obstacle.needForward = False
		#			robot_obstacle.justStop = False
#
#			if(index != -1):
#				rospy.loginfo('callback: obstacle start %s', string)
#				direction = string[-1]
#				if ((direction == 'L') or (direction =='R')):
#					robot_obstacle.needForward = True
#				else:
#					robot_obstacle.justStop = True
#			else:
#					rospy.loginfo("string: %s", string)
#	return

#@yuqing_publishparam

#aaron 23May

def bluetooth_callback(data):
	string = data.data
	robot_drive.bluetooth_panel = string
	rospy.loginfo("found panel: %s", string)

######################################################################

# init the the encoder buffer with some empty data when system starts
def init_encoder_buffer( size=2000 ):
	global encoder_data
	if(len(encoder_data) == size):
		return
	for i in range(size - len(encoder_data)):
		encoder_data.append(0)

# init the compass buffer with some empty data when sytem states
def init_compass_buffer(size = 10):
	global compass_data
	if(len(compass_data) == size):
		return
	for i in range(size):
		compass_data.append(0)

# For each encoder signal received, log the time, if the next signal not coming for a long time, need to process
def process_encoder_delay():
	global last_received_time
	time_now = datetime.now()
	#rospy.loginfo(time_now)
	#rospy.loginfo("----------------------")
	#rospy.loginfo(last_received_time)
	if(last_received_time != 0):
		delta = time_now - last_received_time
		delay_seconds = delta.seconds + delta.microseconds / 1000000.0
		if(delay_seconds >  max_delay):
			bytesToLog = 'Error: Not receiving data for %f seconds: Stopping robot immediately' % (max_delay)
			rospy.logerr(bytesToLog)
			robot_drive.stop_robot()
		else: 							#aaron comment
			time.sleep(0.05) 			#aaron comment
	else: 								#aaron comment
		time.sleep(0.05) 				#aaron comment

# Very import step, based on the encoder data, we do the conversion and calcuation
def process_encoder_data():
	global encoder_data
	global encoder_received
	global encoder_processed
	# Accumulate all available encoder data
	left_encode, right_encode = robot_drive.accum_encoder_data(encoder_data, encoder_received, encoder_processed)
	# After process, update the proccessed index the same as received index
	encoder_processed = encoder_received
	# dynamically calculate and update the gps data, step_angle, step_distance etc while the robot moving
	robot_correction.update_robot_gps(left_encode, right_encode)
	#robot_correction.update_robot_gps_new(left_encode, right_encode) #aaron

def print_config():
	rospy.loginfo("robot_drive: linear_encode_to_mm - %f", 	robot_drive.linear_encode_to_mm)
	rospy.loginfo("robot_drive: turn_encode_to_mm - %f", 	robot_drive.turning_encode_to_mm)
	rospy.loginfo("robot_drive: Turn radius - %f", 	robot_drive.turn_radius)
	rospy.loginfo("robot_correction: Min correction distance - %f",robot_correction.min_correction_distance)
	rospy.loginfo("robot_correction: min correction angle - %f", 	robot_correction.min_correction_angle)
	rospy.loginfo("robot_correction: max_correction_runs - %f", 	robot_correction.max_correction_runs)
	rospy.loginfo("robot_move: dist_to_correct: %f", 	robot_move.dist_to_correct)
	rospy.loginfo("robot_move: dist_lower_speed: %f", 	robot_move.dist_lower_speed)
	rospy.loginfo("robot_move: dist_lowest_speed: %f", 	robot_move.dist_lowest_speed)
	rospy.loginfo("robot_move: linear full speed: %f", robot_move.linear_full_speed)
	rospy.loginfo("robot_move: linear lower speed: %f", robot_move.linear_lower_speed)
	rospy.loginfo("robot_move: linear lowest speed: %f", robot_move.linear_lowest_speed)
	rospy.loginfo("robot_turn: angle lower speed: %f", robot_turn.angle_lower_speed)
	rospy.loginfo("robot_turn: angle lowest speed: %f", robot_turn.angle_lowest_speed)
	rospy.loginfo("robot_turn: turn full speed: %f", robot_turn.turn_full_speed)
	rospy.loginfo("robot_turn: turn lower speed: %f", robot_turn.turn_lower_speed)
	rospy.loginfo("robot_turn: turn lowest speed: %f", robot_turn.turn_lowest_speed)
	rospy.loginfo("robot_drive: obstacle_mode: %f", 	robot_drive.obstacle_mode)
	rospy.loginfo("robot_drive: robot_enabled: %f", 	robot_drive.robot_enabled)
	rospy.loginfo("robot_drive: robot_paused: %f", 	robot_drive.robot_paused)

def update_base(lon, lat):
	config_path = os.path.dirname(os.path.abspath(__file__)) + '/robot.cfg'
	robot_configure.write_config(config_path, 'init', 'init_lon', lon)
	robot_configure.write_config(config_path, 'init', 'init_lat', lat)

def print_status():
	rospy.loginfo("Burn mode:               %d", robot_drive.burn_mode)
	rospy.loginfo("Obstacle mode:           %d", robot_drive.obstacle_mode)
	rospy.loginfo("On obstacle avoidance:   %d", robot_obstacle.on_obstacle)
	rospy.loginfo("Obstacle detected:       %d", robot_obstacle.has_obstacle)
	rospy.loginfo("Manual control mode:     %d", robot_drive.manual_mode)
	rospy.loginfo("Interaction mode:        %d", robot_drive.interaction_mode)
	rospy.loginfo("Motor 1:                 %d", robot_drive.motor_1_ok)
	rospy.loginfo("Motor 2:                 %d", robot_drive.motor_2_ok)
	rospy.loginfo("Encoders:                %d", robot_drive.encoder_ok)
	rospy.loginfo("Gyroscope:               %d", robot_drive.gyroscope_ok)
	rospy.loginfo("Reverse sensors:         %d", robot_obstacle.reverse_sensor_ok)
	rospy.loginfo("Distance sensors:        %d", robot_obstacle.distance_sensor_ok)
