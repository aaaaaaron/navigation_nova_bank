#!/usr/bin/env python
import rospy
import serial
import string
import math
import gpsmath
import robot_drive
import robot_turn
import robot_move
import robot_correction
import time

#-------------------------------------------------------#
#	Robot jobs module									#
#-------------------------------------------------------#
from std_msgs.msg import String
from math import radians, cos, sin, asin, sqrt, atan2, degrees

###################### EDIT HERE ###########################
#defining or acquiring the GPS coordinates
init_lon			= 0.0
init_lat			= 0.0
init_bearing		= 0.0

gps_lon 			= [] #S,A,B,C,D
gps_lat 			= []
loops 				= 1 			#how many rounds to go
job_lists 			= []

back_to_base_mode 	= False
summon_mode  		= False 

FtoT_flag = True
arc_dist = 0.0

job_before_obstacle = None

mode = "R"

# if not jobs in the sytem
def process_no_job():
	robot_drive.robot_on_mission = False
	if(robot_drive.robot_moving or robot_drive.robot_turning):
		rospy.logwarn('warning: robot is not fully stopped even though a stop command issued')
		robot_drive.stop_robot()
		time.sleep(0.05) 			#aaron comment
		return
	else: 							#aaron comment
		time.sleep(0.1) 			#aaron comment

# Process all kinds of robot job as required
def process_job():
	job_completed = False
	global job_lists, job_before_obstacle
	try:

		if not robot_drive.robot_on_mission:
			rospy.loginfo("\n================== Start a new job ==================")
			rospy.loginfo("Job classifictiaon %s, description %s, value %d", job_lists[0].classfication, job_lists[0].description, job_lists[0].value)
			rospy.loginfo("Target lon lat: %f, %f", job_lists[0].lon_target, job_lists[0].lat_target)
			if job_lists[0].classfication == 'N' or (job_lists[0].classfication == 'C' and job_lists[0].description == 'F' and job_lists[0].value > 2 * robot_correction.min_correction_distance) :
				job_before_obstacle = job_lists[0]
		
		if (job_lists[0].description == 'T') :
			# rospy.loginfo("Bearing now %f, bearing target %f", robot_drive.bearing_now, robot_drive.bearing_target)
			
			robot_drive.bearing_target  = job_lists[0].value

			job_completed = robot_turn.turn_degree()
			
		elif (job_lists[0].description == 'F' or job_lists[0].description == 'B'):
			if (job_lists[0].description == 'B'):
				job_lists[0].value  = -abs(job_lists[0].value)
			
			job_completed =robot_move.move_distance(job_lists[0].value)
			
		else :
			rospy.logwarn('job_des %s:%d', job_lists[0].description, job_lists[0].value)
			rospy.logwarn('warning: illegal job description found, not peform any actions')
		#rospy.loginfo("Bearing target before correction %f", robot_drive.bearing_target)

		return job_completed

	except IndexError:
		rospy.loginfo("Job list empty, returning.")
		process_no_job()

# Complete init compass
def complete_init_compass(compass_value):
	if abs(compass_value) <=2.0 or abs(compass_value) >= 358.0:
		#Clear the remainging initialization jobs
		robot_drive.bearing_now = compass_value
		clear_job_list()
		robot_drive.robot_initialized = 1
		rospy.loginfo("Robot initlization completed")

def pause_robot():
		while True:
			#step 1, send the stop command every 10 milli seoncs
			if(robot_drive.robot_moving or robot_drive.robot_turning):
				rospy.loginfo("Stopping robot")
				robot_drive.stop_robot()
				time.sleep(0.01) 			#aaron comment
			else:
				break
			#robot_drive.robot_enabled = 1

# disable robot if emergency stop button clicked (0)
def disable_robot():
	#rospy.loginfo("disable robot")
	pause_robot()
	#rospy.loginfo("Paused robot")
	clear_job_list()

# class to define i
class Job:
	classfication		= 'N'
	lon_target		= 0.0
	lat_target 		= 0.0
	bearing_target		= 0.0
	description		= ''
	value			= 0
	index			= 0

	# constructor
	def __init__(self, lon, lat, bearing, classify, description, value):
		self.lon_target 	= lon
		self.lat_target 	= lat
		self.bearing_target 	= bearing
		self.classfication	= classify
		self.value 		= value
		self.description 	= description
		rospy.loginfo("Define Job: %s, %s, %f", classify, description, value)

#@yuqing_forwardafterobstacle
dist_forward_after_obstacle = 500

#---------------------------re-factored job lists ------------------------------
# init robot initial poition
def init_robot_gps(lon, lat, bearing):
	global init_lat, init_lon, init_bearing
	init_lon 		= lon
	init_lat 		= lat
	init_bearing 	= bearing

# get intermediate lon lat if the next bearing is unknown (before breaking down to the two arc lon lat)
def get_inter_gps(lon_source, lat_source, bearing_source, lon_target, lat_target, forward_dist):
	is_off = False
	lon_new = lon_source
	lat_new = lat_source
	distance = gpsmath.haversine(lon_source, lat_source, lon_target, lat_target)
	correct_bearing = gpsmath.bearing(lon_source, lat_source, lon_target, lat_target)
	bearing_diff = correct_bearing - bearing_source
	if bearing_diff > 180.0:
		bearing_diff = bearing_diff - 360.0
	elif bearing_diff < -180.0:
		bearing_diff = bearing_diff + 360.0

	if abs(bearing_diff) > robot_correction.min_correction_angle:
		# if abs(bearing_diff) > 90.0 and distance <= 2000:
		# 	bearing_source = gpsmath.format_bearing(bearing_source + 180.0)

		lon_new, lat_new = gpsmath.get_gps(lon_source, lat_source, forward_dist, bearing_source)
		is_off = True
	return (is_off, lon_new, lat_new, distance, abs(bearing_diff))

# get intermediate lon lat after breaking down the turning gps to the gps on the arc
def get_inter_gps_aft(gps_lon_list, gps_lat_list):
	global arc_dist
	try:
		temp_data = []
		for j in range(1, len(gps_lon_list) - 1):
			distance1 	= gpsmath.haversine(gps_lon_list[j-1], gps_lat_list[j-1], gps_lon_list[j], gps_lat_list[j])
			bearing1 	= gpsmath.bearing(gps_lon_list[j-1], gps_lat_list[j-1], gps_lon_list[j], gps_lat_list[j])
			distance2	= gpsmath.haversine(gps_lon_list[j], gps_lat_list[j], gps_lon_list[j+1], gps_lat_list[j+1])
			bearing2 	= gpsmath.bearing(gps_lon_list[j], gps_lat_list[j], gps_lon_list[j+1], gps_lat_list[j+1])

			bearing_diff = bearing2 - bearing1

			if bearing_diff > 180.0:
				bearing_diff = bearing_diff - 360.0
			elif bearing_diff < -180.0:
				bearing_diff = bearing_diff + 360.0

			min_bank_deg = abs(2*(math.atan(robot_drive.bank_radius/robot_drive.min_bank_dist))/math.pi*180.0)
			if (abs(bearing_diff) <= (180.0 - min_bank_deg)):
				
				arc_dist = arc_threshold(bearing2, bearing1)					#chengyuen 11/7
				lon_new1, lat_new1 = gpsmath.get_gps(gps_lon_list[j-1], gps_lat_list[j-1], distance1 - arc_dist, bearing1)
				lon_new2, lat_new2 = gpsmath.get_gps(gps_lon_list[j], gps_lat_list[j], arc_dist, bearing2)

				temp_data.extend([(lon_new1, lat_new1, bearing1, 'F'),(lon_new2, lat_new2, bearing2, 'T')])

			else :
				if bearing_diff < 0:
					bearing_target1 = gpsmath.format_bearing(bearing1 + 90.0)
					bearing_target2 = gpsmath.format_bearing(bearing1 - 80.0)

				else:
					bearing_target1 = gpsmath.format_bearing(bearing1 - 90.0)
					bearing_target2 = gpsmath.format_bearing(bearing1 + 80.0)

				arc_dist = arc_threshold(bearing_target1, bearing1)
				lon_new1, lat_new1 = gpsmath.get_gps(gps_lon_list[j-1], gps_lat_list[j-1], distance1 - arc_dist, bearing1)
				lon_new2, lat_new2 = gpsmath.get_gps(gps_lon_list[j], gps_lat_list[j], arc_dist, bearing_target1)

				arc_dist = arc_threshold(bearing_target2, bearing_target1)
				lon_new_inter1, lat_new_inter1 = gpsmath.get_gps(lon_new2, lat_new2, arc_dist, bearing_target1)
				lon_new3, lat_new3 = gpsmath.get_gps(lon_new_inter1, lat_new_inter1, arc_dist, bearing_target2)

				arc_dist = arc_threshold(bearing2, bearing_target2)
				lon_new_inter2, lat_new_inter2 = gpsmath.get_gps(lon_new3, lat_new3, arc_dist, bearing_target2)
				lon_new4, lat_new4 = gpsmath.get_gps(lon_new_inter2, lat_new_inter2, arc_dist, bearing2)

				temp_data.extend([(lon_new1, lat_new1, bearing1, 'F'),(lon_new2, lat_new2, bearing_target1, 'T'), (lon_new3, lat_new3, bearing_target2, 'T'),(lon_new4, lat_new4, bearing2, 'T')])
		return True, temp_data
	except IndexError:
		rospy.logerr("Need at least 3 coordinates, %d given", len(gps_lon_list))
		return False, None


# generate robot jobs based on robot gps route
def generate_jobs_from_gps():
	#step 1: Move from initial point to the loop start point
	global init_lat, init_lon, init_bearing
	global loops, gps_lon, gps_lat
	global job_lists, arc_dist
	robot_drive.lon_now = init_lon
	robot_drive.lat_now = init_lat
	robot_drive.bearing_now = init_bearing

	# create a complete list (including the number of loops) of all the initial gps before processing
	total_gps_lon = [init_lon]
	total_gps_lat = [init_lat]
	
	rospy.loginfo("Number of loops %d", loops)
	for i in range(loops):
		total_gps_lon.extend(gps_lon)
		total_gps_lat.extend(gps_lat)
	
	if mode == "R":
		if total_gps_lon[-1] == gps_lon[0] and total_gps_lat[-1] == gps_lat[0]:
			if total_gps_lon[-2] == gps_lon[0] and total_gps_lat[-2] == gps_lat[0]:
				del total_gps_lon[-1]
				del total_gps_lat[-1]
			total_gps_lon.extend([init_lon])
			total_gps_lat.extend([init_lat])
		else:
			total_gps_lon.extend([gps_lon[0], init_lon])
			total_gps_lat.extend([gps_lat[0], init_lat])


	data_for_jobs = []

	if gpsmath.haversine(total_gps_lon[0], total_gps_lat[0], total_gps_lon[1], total_gps_lat[1]) < 1000.0:
		rospy.logwarn("First gps point is too close, it's within 1000mm. Expect robot to make a big turn!")

	#to check for the starting bearing
	is_bearing_off = get_inter_gps(total_gps_lon[0], total_gps_lat[0], init_bearing, total_gps_lon[1], total_gps_lat[1], 500)
	if is_bearing_off[0]:
		total_gps_lon.insert(1, is_bearing_off[1])
		total_gps_lat.insert(1, is_bearing_off[2])

	if len(total_gps_lon) > 2:
	#finding the intermediate gps lon lat
		obtain_success, inter_gps_new = get_inter_gps_aft(total_gps_lon, total_gps_lat)

		if obtain_success:
			data_for_jobs.extend(inter_gps_new)

	# adding the last point to the data_for_jobs
			lon_last = data_for_jobs[-1][0]
			lat_last = data_for_jobs[-1][1]
			bearing_last = data_for_jobs[-1][2]
			# bearing_to_base = gpsmath.bearing(lon_last, lat_last, init_lon, init_lat)
			bearing_to_last = gpsmath.bearing(lon_last, lat_last, total_gps_lon[-1], total_gps_lat[-1])
			# bearing_diff_base = gpsmath.format_bearing(bearing_to_base - bearing_last)
			bearing_diff_last = gpsmath.format_bearing(bearing_to_last - bearing_last)

			# data_for_jobs.extend([(init_lon, init_lat, bearing_last, 'F')])
			data_for_jobs.extend([(total_gps_lon[-1], total_gps_lat[-1], bearing_last, 'F')])

	else:
		b = gpsmath.bearing(total_gps_lon[0], total_gps_lat[0], total_gps_lon[1], total_gps_lat[1])
		data_for_jobs.extend([(total_gps_lon[-1], total_gps_lat[-1], b, 'F')])

	append_regular_jobs_new(data_for_jobs, init_lon, init_lat)
	# else:
	# 	rospy.logerr("Jobs not generated")
	

def append_backward_job(lon_source, lat_source, lon_target, lat_target, bearing_now):
	global job_lists
	rospy.loginfo("Added a job to move from (%f, %f) to (%f, %f)", lon_source, lat_source, lon_target, lat_target)
	bearing 	= gpsmath.bearing(lon_source, lat_source, lon_target, lat_target)
	distance 	= gpsmath.haversine(lon_source, lat_source, lon_target, lat_target)
	move_job 	= Job(lon_target, lat_target, bearing_now, 'N', 'B', distance)
	job_lists.extend([move_job])

# generate regualr jobs from point to point()
def append_regular_jobs(lon_source, lat_source, lon_target, lat_target):
	global job_lists
	rospy.loginfo("Added a job to move from (%f, %f) to (%f, %f)", lon_source, lat_source, lon_target, lat_target)
	bearing 	= gpsmath.bearing(lon_source, lat_source, lon_target, lat_target)
	distance 	= gpsmath.haversine(lon_source, lat_source, lon_target, lat_target)
	turn_job 	= Job(lon_source, lat_source, bearing, 'N', 'T', bearing)
	move_job 	= Job(lon_target, lat_target, bearing, 'N', 'F', distance)
	rospy.loginfo("Added a turn job: Turn to %f", bearing)
	rospy.loginfo("Added a move job: Move %f mm", distance)
	job_lists.extend([turn_job])
	job_lists.extend([move_job])

def append_regular_jobs_new(jobs_data, lon_s, lat_s):
	global job_lists
	for i in range(len(jobs_data)):
		if i == 0:
			lon_source = lon_s
			lat_source = lat_s
		else:
			lon_source = jobs_data[i-1][0]
			lat_source = jobs_data[i-1][1]
		rospy.loginfo("Added a job to move from (%f, %f) to (%f, %f)", lon_source, lat_source, jobs_data[i][0], jobs_data[i][1])
		if jobs_data[i][3] == 'F':
			distance 	= gpsmath.haversine(lon_source, lat_source, jobs_data[i][0], jobs_data[i][1])
			bearing 	= gpsmath.bearing(lon_source, lat_source, jobs_data[i][0], jobs_data[i][1])
			bearing_diff = gpsmath.format_bearing(bearing - jobs_data[i][2])
			if bearing_diff >= 90.0 and bearing_diff <= 270.0:
				sign = -1
			else:
				sign = 1
			rospy.loginfo("Added a Forward Job, %f mm", distance * sign)
			job = Job(jobs_data[i][0], jobs_data[i][1], jobs_data[i][2], 'N', 'F', distance * sign)
		elif jobs_data[i][3] == 'T':
			rospy.loginfo("Added a Turn Job, %f deg", jobs_data[i][2])
			job = Job(jobs_data[i][0], jobs_data[i][1], jobs_data[i][2], 'N', 'T', jobs_data[i][2])
		job_lists.extend([job])


def amend_obstacle_jobs(lon_source, lat_source, lon_target, lat_target):
	global job_lists
	rospy.loginfo("Amended a job to move from (%f, %f) to (%f, %f)", lon_source, lat_source, lon_target, lat_target)
	bearing 	= gpsmath.bearing(lon_source, lat_source, lon_target, lat_target)
	distance 	= gpsmath.haversine(lon_source, lat_source, lon_target, lat_target)
	turn_job 	= Job(lon_source, lat_source, bearing, 'O', 'T', bearing)

	move_job 	= Job(lon_target, lat_target, bearing, 'O', 'F', distance)
	rospy.loginfo("Amended a turn job: Turn to %f", bearing)
	rospy.loginfo("Amended a move job: Move %f mm", distance)

	# if current_job_motion == 'F' or current_job_motion == 'B':
	# 	# job_lists[1] = move_job
	# 	job_lists.insert(0, turn_job)
	# 	# job_lists.insert(2, Job(lon_target, lat_target, bearing, 'N', job_lists[0].description, 0.0))
	# 	job_lists.insert(1, move_job)
	# elif current_job_motion == 'T':
	# 	job_lists.insert(0, turn_job)
	# 	#job_lists[2] = move_job

	job_lists.insert(0, turn_job)
	job_lists.insert(1, move_job)

def insert_compensation_jobs(jobs_data, correction_type):
	global job_lists
	temp_job = []
	for i in range(len(jobs_data)):
		if i == 0:
			lon_source = robot_drive.lon_now
			lat_source = robot_drive.lat_now
		else:
			lon_source = jobs_data[i-1][0]
			lat_source = jobs_data[i-1][1]
		rospy.loginfo("Amended a job to move from (%f, %f) to (%f, %f)", lon_source, lat_source, jobs_data[i][0], jobs_data[i][1])
		if jobs_data[i][3] == 'F':
			distance 	= gpsmath.haversine(lon_source, lat_source, jobs_data[i][0], jobs_data[i][1])
			bearing 	= gpsmath.bearing(lon_source, lat_source, jobs_data[i][0], jobs_data[i][1])
			bearing_diff = gpsmath.format_bearing(bearing - jobs_data[i][2])
			if bearing_diff >= 90.0 and bearing_diff <= 270.0:
				sign = -1
			else:
				sign = 1
			rospy.loginfo("Amended a Forward Job, %f mm", distance * sign)
			job = Job(jobs_data[i][0], jobs_data[i][1], jobs_data[i][2], correction_type, 'F', distance * sign)
		elif jobs_data[i][3] == 'T':
			rospy.loginfo("Amended a Turn Job, %f deg", jobs_data[i][2])
			job = Job(jobs_data[i][0], jobs_data[i][1], jobs_data[i][2], correction_type, 'T', jobs_data[i][2])
		temp_job.extend([job])
	temp_job.reverse()
	for j in temp_job:
		job_lists.insert(0, j)

def amend_regular_jobs(job_executed, correction_type, forward_dist_bef_turn):
	global job_lists

	# if job_executed.classfication == 'O':
	# 	rospy.logerr("%f, %f", job_executed.lon_target, job_executed.lat_target)

	if job_executed.description == 'F':
		rospy.loginfo("Amended a job from F to move from (%f, %f) to (%f, %f)", robot_drive.lon_now, robot_drive.lat_now, job_executed.lon_target, job_executed.lat_target)
		bearing 	= gpsmath.bearing(robot_drive.lon_now, robot_drive.lat_now, job_executed.lon_target, job_executed.lat_target)
		distance 	= gpsmath.haversine(robot_drive.lon_now, robot_drive.lat_now, job_executed.lon_target, job_executed.lat_target)
		bearing_from_robot = gpsmath.format_bearing(bearing - robot_drive.bearing_now)

		# rospy.logerr(distance)
		# rospy.logerr(bearing_from_robot)
		if distance > 2*robot_drive.bank_radius: # and (bearing_from_robot > 270.0 or bearing_from_robot < 90.0):
			inter_pos	= get_inter_gps(robot_drive.lon_now, robot_drive.lat_now, robot_drive.bearing_now, job_executed.lon_target, job_executed.lat_target, forward_dist_bef_turn)
			# rospy.logerr("F interpos: %d", inter_pos[0])
			if inter_pos[0]:
				lon_list = [robot_drive.lon_now, inter_pos[1], job_executed.lon_target]
				lat_list = [robot_drive.lat_now, inter_pos[2], job_executed.lat_target]
				obtain_success, new_job = get_inter_gps_aft(lon_list, lat_list)
				distance_to_move = gpsmath.haversine(new_job[-1][0], new_job[-1][1], job_executed.lon_target, job_executed.lat_target)
				move_job = Job(job_executed.lon_target, job_executed.lat_target, new_job[-1][2], correction_type, 'F', distance_to_move)
				job_lists.insert(0, move_job)
				if obtain_success:
					insert_compensation_jobs(new_job, correction_type)
			else:
				move_job = Job(job_executed.lon_target, job_executed.lat_target, bearing, correction_type, 'F', distance)
				job_lists.insert(0, move_job)

	elif has_jobs_left() and job_executed.description == 'T':
		rospy.loginfo("Amended a job from T to move from (%f, %f) to (%f, %f)", robot_drive.lon_now, robot_drive.lat_now, current_job().lon_target, current_job().lat_target)
		bearing 	= gpsmath.bearing(robot_drive.lon_now, robot_drive.lat_now, current_job().lon_target, current_job().lat_target)
		distance 	= gpsmath.haversine(robot_drive.lon_now, robot_drive.lat_now, current_job().lon_target, current_job().lat_target)
		dist_from_ori_target 	= gpsmath.haversine(robot_drive.lon_now, robot_drive.lat_now, job_executed.lon_target, job_executed.lat_target)
		bearing_from_robot = gpsmath.format_bearing(bearing - robot_drive.bearing_now)

		if (bearing_from_robot >= 292.5 or bearing_from_robot <= 67.5):

			inter_pos	= get_inter_gps(robot_drive.lon_now, robot_drive.lat_now, robot_drive.bearing_now, current_job().lon_target, current_job().lat_target, forward_dist_bef_turn)
			# rospy.logerr("T interpos: %d", inter_pos[0])
			if inter_pos[0]:
				lon_list = [robot_drive.lon_now, inter_pos[1], current_job().lon_target]
				lat_list = [robot_drive.lat_now, inter_pos[2], current_job().lat_target]
				obtain_success, new_job = get_inter_gps_aft(lon_list, lat_list)
				distance_to_move = gpsmath.haversine(new_job[-1][0], new_job[-1][1], current_job().lon_target, current_job().lat_target)
				# rospy.logerr(distance_to_move)
				move_job = Job(current_job().lon_target, current_job().lat_target, new_job[-1][2], correction_type, 'F', distance_to_move)
				job_lists[0] = move_job
				if obtain_success:
					insert_compensation_jobs(new_job, correction_type)
			else:
				# rospy.logerr(dist_from_ori_target)
				if dist_from_ori_target > robot_correction.min_correction_distance:
					move_job = Job(current_job().lon_target, current_job().lat_target, robot_drive.bearing_now, correction_type, 'F', distance)
					job_lists[0] = move_job

def turn_aft_move():
	global job_lists
	lon_next = job_lists[1].lon_target
	lat_next = job_lists[1].lat_target
	del job_lists[1]
	lon_now = robot_drive.lon_target
	lat_now = robot_drive.lat_target
	lon_prev = robot_drive.lon_now
	lat_prev = robot_drive.lat_now
	lon_list = [lon_prev, lon_now, lon_next]
	lat_list = [lat_prev, lat_now, lat_next]
	obtain_success, new_job = get_inter_gps_aft(lon_list, lat_list)
	if obtain_success:
		insert_compensation_jobs(new_job, 'C')

def has_jobs_left():
	global job_lists
	#rospy.loginfo("No of jobs left %d", len(job_lists))
	return len(job_lists) > 0

def current_job():
	global job_lists

	if has_jobs_left():
		return job_lists[0]

	return 'I';

def left_gps_distance():
	job_now = current_job()
	dist_temp = gpsmath.haversine(job_now.lon_target, job_now.lat_target, robot_drive.lon_now,  robot_drive.lat_now)
	return dist_temp

def clear_job_list():
	global job_lists
	del job_lists[:]

def remove_job(idx):
	global job_lists
	length = len(job_lists)
	if idx <= length-1:
		del job_lists[idx]

# If return value is false, then need coorecction
# If return value is true, then no need correction for the current job
def complete_current_job():
	global job_lists
	if job_lists[0].classfication == 'N':
		robot_drive.lon_target 		= job_lists[0].lon_target;
		robot_drive.lat_target 		= job_lists[0].lat_target;
		robot_drive.bearing_target 	= job_lists[0].bearing_target;
	rospy.loginfo("Removed current job with classification: %s, description: %s, value: %f", job_lists[0].classfication, job_lists[0].description, job_lists[0].value)
	del job_lists[0]
	rospy.loginfo("Number of jobs left to execute %d", len(job_lists))
	rospy.loginfo("================== The job finished and discarded ==================\n\n")

# list of test jobs: Not from gps, but just like move, turn etc
# ---------------------------------------------------------------------------------
def simple_move(distance, bearing, direction):
	rospy.loginfo("Added a turn job T, %d", bearing)
	simple_turn(bearing)
	# add a move job to move 10 meters
	lon_new, lat_new  = append_regular_job(robot_drive.lon_now, robot_drive.lat_now, distance, bearing)

def simple_turn(bearing):
	append_turn_job(robot_drive.lon_now, robot_drive.lat_now, bearing)

# ----------------------------------------------------------------------------------
# A list of operations for regular jobs

def append_turn_job(lon_target, lat_target, bearing_target):
	turn_job 	= Job(lon_target, lat_target, bearing_target, 'N', 'T', bearing_target)
	job_lists.extend([turn_job])

def append_regular_job(lon_now, lat_now, distance, bearing):
	# Get new GPS
	lon_new, lat_new  = gpsmath.get_gps(lon_now, lat_now, distance, bearing)
	if(bearing == robot_drive.bearing_now and distance < 0):
		append_backward_job(lon_now, lat_now, lon_new, lat_new, bearing)
	else:
		append_regular_jobs(lon_now, lat_now, lon_new, lat_new)
	return lon_new, lat_new

def define_test_job():
	lon_new, lat_new = append_regular_job(robot_drive.lon_now, robot_drive.lat_now, 3000.0, robot_drive.bearing_now)
	append_turn_job(lon_new, lat_new, robot_drive.bearing_now - 90.0)
	append_turn_job(lon_new, lat_new, robot_drive.bearing_now)
	# add a turn job to turn to 0 degree
	# append_turn_job(robot_drive.lon_now, robot_drive.lat_now, 0.0)
	# # add a move job to move 10 meters
	# lon_new, lat_new  = append_regular_job(robot_drive.lon_now, robot_drive.lat_now, 10000.0, 0.0)
	# # now turn to 90
	# append_turn_job(lon_new, lat_new , 90.0)
	# # move another 10 meters
	# lon_new, lat_new  = append_regular_job(lon_new, lat_new, 10000.0, 90.0)
	# # now turn to 180
	# append_turn_job(lon_new, lat_new , 180.0)
	# # move another 10 meters
	# lon_new, lat_new  = append_regular_job(lon_new, lat_new, 10000.0, 180.0)
	# # now turn to 270
	# append_turn_job(lon_new, lat_new , 270.0)
	# # move another 10 meters
	# lon_new, lat_new  = append_regular_job(lon_new, lat_new, 10000.0, 270.0)
	# # now turn to 270
	# append_turn_job(robot_drive.lon_now, robot_drive.lat_now, 0.0)

# The job used to initialize the compass
def define_initialize_job():
	append_turn_job(robot_drive.lon_now, robot_drive.lat_now, 0.0)
	append_turn_job(robot_drive.lon_now, robot_drive.lat_now, 90.0)
	append_turn_job(robot_drive.lon_now, robot_drive.lat_now, 180.0)
	append_turn_job(robot_drive.lon_now, robot_drive.lat_now, 270.0)
	append_turn_job(robot_drive.lon_now, robot_drive.lat_now, 0.0)
	append_turn_job(robot_drive.lon_now, robot_drive.lat_now, 90.0)

def no_correction_jobs():
	global job_lists
	count = 0
	no_job = len(job_lists)
	for i in range(no_job):
		if(job_lists[i].classfication == 'C'):
			count = count + 1
		else:
			break
	return count

def no_normal_jobs():
	global job_lists
	count = 0
	no_job = len(job_lists)
	for i in range(no_job):
		if(job_lists[i].classfication == 'N'):
			count = count + 1
		else:
			break
	return count


def clear_correction_jobs():
	global job_lists
	while has_jobs_left():
		if(job_lists[0].classfication == 'N' or job_lists[0].classfication == 'U'):
			break;
		else:
			complete_current_job()


def insert_move_job(lon_source, lat_source, bearing_source, lon_target, lat_target, bearing_target, correction_type):
	global job_lists
	bearing 	= gpsmath.bearing(lon_source, lat_source, lon_target, lat_target)
	distance 	= gpsmath.haversine(lon_source, lat_source, lon_target, lat_target)
	if distance < 0:																							#chengyuen14/7
		move_job 				= Job(lon_target, lat_target, bearing, correction_type, 'B', abs(distance))
		job_lists.insert(0, move_job)
	else:
		move_job 				= Job(lon_target, lat_target, bearing, correction_type, 'F', distance)
		job_lists.insert(0, move_job)
	rospy.loginfo("Number of jobs %d", len(job_lists))

# a list of operations for the correction jobs
# def insert_compensation_jobs(lon_source, lat_source, bearing_source, lon_target, lat_target, bearing_tarcorrection_typeget, correction_type, need_correct_distance, need_correct_angle):
# 	global job_lists
# 	bearing 	= gpsmath.bearing(lon_source, lat_source, lon_target, lat_target)
# 	distance 	= gpsmath.haversine(lon_source, lat_source, lon_target, lat_target)

# 	# reverse_value1 = arc_threshold(bearing, bearing_source)		#chengyuen14/7
# 	# reverse_value2 = arc_threshold(bearing_target, bearing)		#chengyuen14/7

# 	# first_reverse_job		= Job(lon_source, lat_source, bearing_source, correction_type, 'B', 2*reverse_value1)	#chengyuen14/7
# 	turn_job 				= Job(lon_source, lat_source, bearing_target, correction_type, 'T', bearing_target)
# 	turn_before_move_job 	= Job(lon_source, lat_source, bearing, correction_type, 'T', bearing)
# 	# distance = distance - reverse_value1 - reverse_value2														#chengyuen14/7
# 	if distance < 0:																							#chengyuen14/7
# 		move_job 				= Job(lon_target, lat_target, bearing, correction_type, 'B', abs(distance))
# 	else:
# 		move_job 				= Job(lon_target, lat_target, bearing, correction_type, 'F', distance)
# 	# second_reverse_job		= Job(lon_target, lat_target, bearing_target, correction_type, 'B', reverse_value2)	#chengyuen14/7
# 	#reverse_job 			= Job(lon_target, lat_target, bearing, correction_type, 'B', distance)

# 	#if (need_correct_distance and not need_correct_angle):
# 	#	if((bearing - bearing_source + 360.0)%360.0 >= 90):
# 	#		rospy.loginfo("Added a backward distance correction")
# 	#		job_lists.insert(0, reverse_job)
# 	#	else:
# 	#		rospy.loginfo("Added a forward distance correction")
# 	#		job_lists.insert(0, move_job)

# 	# if need_correct_distance and need_correct_angle:
# 	# 	rospy.loginfo("Added a distance correction and angle correction")
# 	# 	job_lists.insert(0, first_reverse_job)										#chengyuen14/7
# 	# 	job_lists.insert(1, turn_before_move_job)									#chengyuen14/7
# 	# 	job_lists.insert(2, move_job)												#chengyuen14/7
# 	# 	job_lists.insert(3, turn_job)												#chengyuen14/7
# 	# 	job_lists.insert(4, second_reverse_job)										#chengyuen14/7
# 	# elif need_correct_distance and not need_correct_angle:
# 	# 	rospy.loginfo("Added an distance correction")
# 	# 	job_lists.insert(0, first_reverse_job)										#chengyuen14/7
# 	# 	job_lists.insert(1, turn_before_move_job)									#chengyuen14/7
# 	# 	job_lists.insert(2, move_job)												#chengyuen14/7
# 	# 	job_lists.insert(3, turn_job)												#chengyuen14/7
# 	# 	job_lists.insert(4, second_reverse_job)										#chengyuen14/7
# 	# elif (need_correct_angle and not need_correct_distance):
# 	# 	rospy.loginfo("Added a angle correction")
# 	# 	job_lists.insert(0, first_reverse_job)										#chengyuen14/7
# 	# 	job_lists.insert(1, turn_job)												#chengyuen14/7
# 	# 	job_lists.insert(2, second_reverse_job)										#chengyuen14/7

# 	if need_correct_distance and need_correct_angle:
# 		rospy.loginfo("Added a distance correction and angle correction")
# 		job_lists.insert(0, turn_before_move_job)
# 		job_lists.insert(1, move_job)
# 		job_lists.insert(2, turn_job)
# 	elif need_correct_distance and not need_correct_angle:
# 		rospy.loginfo("Added an distance correction")
# 		job_lists.insert(0, turn_before_move_job)
# 		job_lists.insert(1, move_job)
# 		job_lists.insert(2, turn_job)
# 	elif (need_correct_angle and not need_correct_distance):
# 		rospy.loginfo("Added a angle correction")
# 		job_lists.insert(0, turn_job)


def distance_route(gps_lon_lst, gps_lat_lst):
	dist_total = 0.0
	for k in range(len(gps_lon_lst) - 1):
		k_nex = k + 1
		distance_cal 	= gpsmath.haversine(gps_lon_lst[k], gps_lat_lst[k], gps_lon_lst[k_nex], gps_lat_lst[k_nex])
		dist_total = dist_total + distance_cal

	return dist_total

def generate_rb_jobs(gps_lon_lst, gps_lat_lst):
	rospy.loginfo("Number of jobs %d", len(gps_lon_lst));
	# for k in range(len(gps_lon_lst) - 1):
	# 	k_nex = k + 1
	# 	append_regular_jobs(gps_lon_lst[k], gps_lat_lst[k], gps_lon_lst[k_nex], gps_lat_lst[k_nex])

	data_for_jobs = []

	is_bearing_off = get_inter_gps(gps_lon_lst[0], gps_lat_lst[0], robot_drive.bearing_now, gps_lon_lst[1], gps_lat_lst[1], 500)
	if is_bearing_off[0]:
		gps_lon_lst.insert(1, is_bearing_off[1])
		gps_lat_lst.insert(1, is_bearing_off[2])

	if len(gps_lon_lst) > 2:
	#finding the intermediate gps lon lat
		obtain_success, inter_gps_new = get_inter_gps_aft(gps_lon_lst, gps_lat_lst)

		if obtain_success:
			data_for_jobs.extend(inter_gps_new)

	# adding the last point to the data_for_jobs
			lon_last = data_for_jobs[-1][0]
			lat_last = data_for_jobs[-1][1]
			bearing_last = data_for_jobs[-1][2]
			# bearing_to_base = gpsmath.bearing(lon_last, lat_last, init_lon, init_lat)
			bearing_to_last = gpsmath.bearing(lon_last, lat_last, gps_lon_lst[-1], gps_lat_lst[-1])
			# bearing_diff_base = gpsmath.format_bearing(bearing_to_base - bearing_last)
			bearing_diff_last = gpsmath.format_bearing(bearing_to_last - bearing_last)

			# data_for_jobs.extend([(init_lon, init_lat, bearing_last, 'F')])
			data_for_jobs.extend([(gps_lon_lst[-1], gps_lat_lst[-1], bearing_last, 'F')])

	else:
		b = gpsmath.bearing(gps_lon_lst[0], gps_lat_lst[0], gps_lon_lst[1], gps_lat_lst[1])
		data_for_jobs.extend([(gps_lon_lst[-1], gps_lat_lst[-1], b, 'F')])

	append_regular_jobs_new(data_for_jobs, robot_drive.lon_now, robot_drive.lat_now)

def find_closest_loop_index():
	# Find the loop points which is cloest to the current position
	distance = 100000000.0
	gps_num = len(gps_lon)
	index  = 0
	for k in range (gps_num):
		distance_cal 	= gpsmath.haversine(robot_drive.lon_now, robot_drive.lat_now, gps_lon[k],gps_lat[k])
		if(distance_cal < distance):
			distance = distance_cal
			index = k
	return index

def find_closest_loop_index_panel():
	distance = 100000000.0
	gps_num = len(gps_lon)
	index  = 0
	for k in range (gps_num):
		distance_cal 	= gpsmath.haversine(robot_drive.panel_lon, robot_drive.panel_lat, gps_lon[k],gps_lat[k])
		if(distance_cal < distance):
			distance = distance_cal
			index = k
	return index
	index = 0
	return index

def go_to_panel_jobs():
	gps_num = len(gps_lon)
	panel_stop_dist = 1000

	gps_lon_tmp_1 = []
	gps_lon_tmp_2 = []
	gps_lat_tmp_1 = []
	gps_lat_tmp_2 = []

	d = gpsmath.haversine(robot_drive.lon_now, robot_drive.lat_now, robot_drive.panel_lon, robot_drive.panel_lat)
	# rospy.logwarn("%f", d)
	if d <= 2*panel_stop_dist:
		return

	gps_lon_tmp_1.extend([robot_drive.lon_now])
	gps_lat_tmp_1.extend([robot_drive.lat_now])
	gps_lon_tmp_2.extend([robot_drive.lon_now])
	gps_lat_tmp_2.extend([robot_drive.lat_now])

	if(gps_num != 0):	
		index = find_closest_loop_index()
		index2 = find_closest_loop_index_panel()
	
		rospy.loginfo('index 1: %d, index2: %d', index, index2)

		end_index1 = index 
		while end_index1 != index2: 
			gps_lon_tmp_1.extend([gps_lon[end_index1]])
			gps_lat_tmp_1.extend([gps_lat[end_index1]])
			end_index1 = (end_index1 + 1) % gps_num
		gps_lon_tmp_1.extend([gps_lon[index2]])
		gps_lat_tmp_1.extend([gps_lat[index2]])

		end_index2 = index
		while end_index2 != index2: 
			gps_lon_tmp_2.extend([gps_lon[end_index2]])
			gps_lat_tmp_2.extend([gps_lat[end_index2]])
			end_index2 = (end_index2 - 1 + gps_num) % gps_num
		gps_lon_tmp_2.extend([gps_lon[index2]])
		gps_lat_tmp_2.extend([gps_lat[index2]])


	dist1 = distance_route(gps_lon_tmp_1, gps_lat_tmp_1)
	dist2 = distance_route(gps_lon_tmp_2, gps_lat_tmp_2)
	# rospy.logwarn("%f, %f", dist1, dist2)
	panel_to_closet_pt_bearing = gpsmath.bearing(robot_drive.panel_lon, robot_drive.panel_lat, gps_lon[index], gps_lat[index])
	panel_stop_lon, panel_stop_lat = gpsmath.get_gps(robot_drive.panel_lon, robot_drive.panel_lat, panel_stop_dist, panel_to_closet_pt_bearing)

	if(dist1 <= dist2):
		gps_lon_tmp_1.extend([panel_stop_lon])
		gps_lat_tmp_1.extend([panel_stop_lat])
		generate_rb_jobs(gps_lon_tmp_1, gps_lat_tmp_1)
	else:
		gps_lon_tmp_2.extend([panel_stop_lon])
		gps_lat_tmp_2.extend([panel_stop_lat])
		generate_rb_jobs(gps_lon_tmp_2, gps_lat_tmp_2)
	rospy.loginfo("Number of jobs %d", len(job_lists))

def back_to_base_jobs():
	# Find the loop points which is cloest to the current position

	gps_num = len(gps_lon)

	if(gps_num ==0):
		return

	index = find_closest_loop_index()

	gps_lon_tmp_1 = []
	gps_lon_tmp_2 = []
	gps_lat_tmp_1 = []
	gps_lat_tmp_2 = []

	gps_lon_tmp_1.extend([robot_drive.lon_now])
	gps_lat_tmp_1.extend([robot_drive.lat_now])
	gps_lon_tmp_2.extend([robot_drive.lon_now])
	gps_lat_tmp_2.extend([robot_drive.lat_now])

	# Prepare path1
	for k in range (index, gps_num + 1):
		idx =  k % gps_num
		gps_lon_tmp_1.extend([gps_lon[idx]])
		gps_lat_tmp_1.extend([gps_lat[idx]])

	# parepare path2
	for k in range (index, -1, -1):
		gps_lon_tmp_2.extend([gps_lon[k]])
		gps_lat_tmp_2.extend([gps_lat[k]])

	dist1 = distance_route(gps_lon_tmp_1, gps_lat_tmp_1)
	dist2 = distance_route(gps_lon_tmp_2, gps_lat_tmp_2)

	if(dist1 < dist2):
		gps_lon_tmp_1.extend([init_lon])
		gps_lat_tmp_1.extend([init_lat])
		generate_rb_jobs(gps_lon_tmp_1, gps_lat_tmp_1)
	else:
		gps_lon_tmp_2.extend([init_lon])
		gps_lat_tmp_2.extend([init_lat])
		generate_rb_jobs(gps_lon_tmp_2, gps_lat_tmp_2)

def prepare_to_panel(): 
	global summon_mode 
	rospy.loginfo('go to panel jobs')
	go_to_panel_jobs() 


def prepare_back_to_base():
	global back_to_base_mode
	back_to_base_jobs()
	back_to_base_mode = True;

#------------------------- end of re-factoring ----------------------------------


def arc_threshold(next_angle, current_angle):
    angle = next_angle - current_angle
    if angle > 180.0:
        angle = abs(angle - 360.0)
    elif angle < -180.0:
        angle = abs(angle + 360.0)
    value = robot_drive.bank_radius * math.tan((angle/2.0) / 180.0 * math.pi)
    value = abs(value) #+ robot_correction.min_correction_distance
    return value
