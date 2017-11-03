#!/usr/bin/env python
import rospy
import serial
import string
from std_msgs.msg import String
import robot_drive
import robot_correction
import gpsmath
import math
import robot_job
import robot_publisher
import robot_move
# import write_log

#-------------------------------------------------------#
#	Robot moving module									#
#-------------------------------------------------------#

dist_completed 			= 0.0
dist_to_run 			= 0.0
angle_to_correct 		= 0.0
#@yuqing_correctionper10m
dist_to_correct 		= 0.0
# dist_completed_to_correct = 0.0
dist_lowest_speed 		= 0.0
dist_lower_speed 		= 0.0
#dist_end_point_check 	= 600.0 #was 600

linear_full_speed 		= 0.0
linear_lower_speed 		= 0.0
linear_lowest_speed		= 0.0

move_amend = False
move_amend2 = False

next_lon = 0.0
next_lat = 0.0
next_pt_dist = 2000.0

dist_completed_imu		= 0.0
dist_to_correct_imu		= 0.0
imu_to_correct			= 0.0

status_pub = rospy.Publisher('status', String, queue_size = 100)
# Starts the robot for moving, put the control variables into proper value
def start_move():
	global dist_completed
	global dist_to_run
	global dist_completed_imu

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
		dist_completed_imu = 0.0
		angle_to_correct = 0.0
		rospy.loginfo('----------------- Started a moving job -------------------')
	else:
	  	robot_drive.start()
		rospy.loginfo('Starting a moving job')

# Roboet complet a moving job
def stop_move():
	global dist_completed, amend_check, dist_completed_imu
	# direction_sign = False
	# if len(robot_job.job_lists) > 1:
	# 	direction_sign = robot_job.job_lists[0].value * robot_job.job_lists[1].value >= 0
	# if not direction_sign and robot_drive.robot_moving:
	# 	rospy.loginfo("Robot changing direction")
	# 	robot_drive.stop_robot()

	# elif (robot_job.no_normal_jobs() >= 1) or not robot_drive.robot_moving :
	if not robot_drive.robot_moving:
		# string = "distance completed ; %f ; robot direction ; %f\n\n"%(dist_completed, robot_drive.bearing_now)
		# write_log.write_to_file(string)
		dist_completed = 0.0
		dist_completed_imu = 0.0
		robot_drive.robot_on_mission = False
		rospy.loginfo('----------------- Robot completed a moving job -----------------')
		# amend_check = False
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

	# robot_publisher.publish_command(robot_drive.move_direction, robot_drive.speed_now)

# main function to control the robot movement
def move_distance(dist):
	global dist_completed, dist_completed_imu
	global dist_to_run
	global angle_to_correct, dist_to_correct_imu, imu_to_correct
	global move_amend, move_amend2
	global next_lon, next_lat, next_pt_dist
	# global dist_completed_to_correct

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
	dist_completed_imu = dist_completed_imu + abs(dist_step)
	# dist_completed_to_correct = dist_completed_to_correct + abs(dist_step)
	# robot is with in the range, then we condidered robot reached the position
	dist_threshold = abs_dist_to_run - robot_correction.min_correction_distance/2 	#0 mm, I can choose -50mm, but since there will be inefficiencies, 0 error threshold might be good enough

	if robot_drive.show_log:
		distpub = 'speed %d, Dist-travelled: %f dist-total:%f dist-step:%f' % (robot_drive.speed_now, dist_completed, abs(dist_to_run) ,dist_step)
		rospy.loginfo(distpub)
	angle_to_correct = angle_to_correct + robot_drive.step_angle
	if robot_drive.show_log:
		rospy.loginfo('Accumulated angle error: %f', angle_to_correct)
	#@yuqing_correctionper10m
	#if travel over 5m, job_completed to 1, start to correct
	# rospy.logerr("Dist completed: %.10f"%robot_move.dist_completed)
	
	# ''' after a certain distance add/minus a certain amount of angle from the imu data '''
	# if dist_completed_imu >= dist_to_correct_imu:
	# 	rospy.logwarn("dist: %f", dist_completed_imu)
	# 	rospy.logwarn("original bearing: %f", robot_drive.bearing_now)
	# 	robot_drive.bearing_now = robot_drive.bearing_now + imu_to_correct
	# 	robot_drive.bearing_now = gpsmath.format_bearing(robot_drive.bearing_now)
	# 	rospy.logwarn("corrected bearing: %f", robot_drive.bearing_now)
	# 	dist_completed_imu = 0.0
	
		
	''' insert correction to go back to path instead of just heading '''
	if robot_correction.correction_mode == 1:
		if robot_job.supposed_lon == robot_job.job_lists[0].lon_target:
			rospy.logwarn("source and target location are the same")
		else:
			dist_from_source = gpsmath.haversine(robot_job.supposed_lon, robot_job.supposed_lat, robot_drive.lon_now, robot_drive.lat_now)
			angle_from_source = gpsmath.bearing(robot_job.supposed_lon, robot_job.supposed_lat, robot_drive.lon_now, robot_drive.lat_now)
			angle_target = gpsmath.bearing(robot_job.supposed_lon, robot_job.supposed_lat, robot_job.job_lists[0].lon_target, robot_job.job_lists[0].lat_target)
			if (angle_from_source == None) or (angle_target == None):
				alpha = 0.0
			else:
				alpha = angle_target - angle_from_source
			if(alpha > 180.0):
				alpha = alpha - 360.0
			elif(alpha < -180.0):
				alpha = alpha + 360.0
			alpha = abs(alpha)

			projected_dist = abs(dist_from_source * math.cos(math.radians(alpha)))
			# rospy.logwarn("projected dist: %f", projected_dist)
			closest_point_lon, closest_point_lat = gpsmath.get_gps(robot_job.supposed_lon, robot_job.supposed_lat, projected_dist, angle_target)
			# rospy.logerr("closest: %f, %f", closest_point_lon, closest_point_lat)
			closest_dist = gpsmath.haversine(robot_drive.lon_now, robot_drive.lat_now, closest_point_lon, closest_point_lat)
			# rospy.logwarn("closest dist: %f", closest_dist)
			if closest_dist >= 1000.0:
				rospy.loginfo("-----------------dist_off_course: %f, start to correct", closest_dist)
				next_lon, next_lat = gpsmath.get_gps(closest_point_lon, closest_point_lat, next_pt_dist, angle_target)
				# rospy.logerr("next: %f, %f", next_lon, next_lat)
				end_dist_from_closest_pt = gpsmath.haversine(closest_point_lon, closest_point_lat, robot_job.job_lists[0].lon_target, robot_job.job_lists[0].lat_target)
				if end_dist_from_closest_pt <= next_pt_dist or (abs(end_dist_from_closest_pt - next_pt_dist) <= 2*robot_drive.bank_radius):
					pass
				else:
					move_amend2 = True
					stop_move()
					return not robot_drive.robot_on_mission

		if (dist_completed >= dist_to_correct):
			bearing 	= gpsmath.bearing(robot_drive.lon_now, robot_drive.lat_now, robot_job.job_lists[0].lon_target, robot_job.job_lists[0].lat_target)
			diff_angle = (bearing - robot_drive.bearing_now + 360.0) % 360.0
			if (diff_angle > 180.0):
				diff_angle = diff_angle - 360.0
			# rospy.logerr("Diff Angle: %.10f"%diff_angle)
			# rospy.logerr(diff_angle)
			if  abs(diff_angle) >= robot_correction.min_correction_angle:# and abs(diff_angle) <= robot_correction.min_correction_angle + 5.0:
				rospy.loginfo("-----------------dist_completed: %f, angle_off_course: %f, start to correct", dist_completed, diff_angle)
				# if not amend_check:
				# 	robot_job.amend_regular_jobs(robot_drive.lon_now, robot_drive.lat_now, robot_job.job_lists[0].lon_target, robot_job.job_lists[0].lat_target)
				move_amend = True
			
				stop_move()
				return not robot_drive.robot_on_mission


	elif robot_correction.correction_mode == 0:
		if (dist_completed >= dist_to_correct):	
			bearing 	= gpsmath.bearing(robot_drive.lon_now, robot_drive.lat_now, robot_job.job_lists[0].lon_target, robot_job.job_lists[0].lat_target)
			diff_angle = (bearing - robot_drive.bearing_now + 360.0) % 360.0
			if (diff_angle > 180.0):
				diff_angle = diff_angle - 360.0
			# rospy.logerr("Diff Angle: %.10f"%diff_angle)
			# rospy.logerr(diff_angle)
			if  abs(diff_angle) >= robot_correction.min_correction_angle:# and abs(diff_angle) <= robot_correction.min_correction_angle + 5.0:
				rospy.loginfo("-----------------dist_completed: %f, angle_off_course: %f, start to correct", dist_completed, diff_angle)
				# if not amend_check:
				# 	robot_job.amend_regular_jobs(robot_drive.lon_now, robot_drive.lat_now, robot_job.job_lists[0].lon_target, robot_job.job_lists[0].lat_target)
				move_amend = True
			
				stop_move()
				return not robot_drive.robot_on_mission

#chengyuen-todo 18/7
# correction_yq begin
	# bearing 	= gpsmath.bearing(robot_drive.lon_now, robot_drive.lat_now, robot_job.job_lists[0].lon_target, robot_job.job_lists[0].lat_target)
	# diff_angle = (bearing - robot_drive.bearing_now + 360.0) % 360.0
	# if (diff_angle > 180.0):
	# 	diff_angle = diff_angle - 360.0

	# rospy.logerr(diff_angle)
	# if ( abs(diff_angle) >= robot_correction.min_correction_angle and abs(diff_angle) <= robot_correction.min_correction_angle + 5.0):

	# 	dist_correct_pub = "--------------- Correction after %f deg off course"%diff_angle

	# 	rospy.loginfo(dist_correct_pub)

		# dist_completed_to_correct = 0.0
		# stop_move()
		# if not amend_check:
		# 	# rospy.logerr("Amendment")
		# 	robot_job.amend_regular_jobs(robot_drive.lon_now, robot_drive.lat_now, robot_job.job_lists[0].lon_target, robot_job.job_lists[0].lat_target)
		# 	amend_check = True
		# stop_move()
		# return not robot_drive.robot_on_mission
# correction_yq end

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
		rospy.loginfo("-----------------dist_completed: %f", dist_completed)
		stop_move()
		move_amend = True
		
        # make sure the robot is stopped before next job
        return not robot_drive.robot_on_mission
       	#clean current job
	return False
