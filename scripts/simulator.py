#!/usr/bin/env python
# The class is to simulate the robot b

import rospy
import serial
import string
import sys
import random
from geometry_msgs.msg import Vector3
from serial_handler.msg import Encoder
from serial_handler.msg import Status
from serial_handler.msg import Sonar

from datetime import datetime

from std_msgs.msg import String

encoder_pub 		= rospy.Publisher('encoder', Encoder, queue_size = 1000)
status_pub			= rospy.Publisher('hardware_status', Status, queue_size = 1000)
ss = Status()
#velocity_pub  		= rospy.Publisher('velocity', Vector3, queue_size = 1)
velocity_vector  	= Vector3()
velocity_vector.x  	= 0.0
velocity_vector.y 	= 0.0
velocity_vector.z 	= 0.0

left_encode   	= 0
right_encode 	= 0
burn_mode		= True
direction 		= 0
sonar_data		= []
prev_has_obstacle	= 0
ss.obstacle_avoidance_mode = 0
ss.battery_level = 90

def executor_simulator(data):
	global burn_mode
	global left_encode
	global right_encode
	global direction
	global ss
	command_str = str(data.data)
	command_str = command_str.rstrip('\0')
	command_str = command_str.rstrip('\n')
	command_str = command_str.rstrip('\r')
	rospy.loginfo("recieved command %s", command_str)
	rospy.loginfo("--------------------------")
	deviate_l = random.randint(-30, 30)
	deviate_r = random.randint(-30, 30)
	if (command_str == 'iap_jump_app'):
		rospy.loginfo('Turn off burn mode')
		burn_mode = False
	elif (command_str == 'iap'):
		rospy.loginfo("Turn to burn mode")
		burn_mode = True
	elif (command_str == 'SF000006E'):
		left_encode = 2000 + deviate_l + 50
		right_encode  = 2000 + deviate_r
		direction = 1
	elif (command_str == 'SF000005E'):
		left_encode = 1600 + deviate_l + 50
		right_encode  = 1600 + deviate_r
		direction = 1
	elif(command_str == 'SF000004E'):
		left_encode = 1200 + deviate_l + 50
		right_encode = 1200 + deviate_r
		direction = 1
	elif(command_str == 'SF000003E'):
		left_encode = 850 + deviate_l + 50
		right_encode = 850 + deviate_r
		direction = 1
	elif(command_str == 'SF000002E'):
		left_encode = 600 + deviate_l + 50
		right_encode = 600 + deviate_r
		direction = 1
	elif(command_str == 'SB000006E'):
		left_encode = -2000 + deviate_l
		right_encode  = -2000 + deviate_r
		direction = 2
	elif(command_str == 'SB000005E'):
		left_encode = -1600 + deviate_l
		right_encode  = -1600 + deviate_r
		direction = 2
	elif(command_str == 'SB000004E'):
		left_encode = -1200 + deviate_l
		right_encode = -1200 + deviate_r
		direction = 2
	elif(command_str == 'SB000003E'):
		left_encode = -850 + deviate_l
		right_encode = -850 + deviate_r
		direction = 2
	elif(command_str == 'SB000002E'):
		left_encode = -600 + deviate_l
		right_encode = -600 + deviate_r
		direction = 2
	elif(command_str == 'SL000006E'):
		left_encode = 220 + deviate_l
		right_encode = 1400 + deviate_r
		direction = 3
	elif(command_str == 'SL000005E'):
		left_encode = 220 + deviate_l
		right_encode = 1400 + deviate_r
		direction = 3
	elif(command_str == 'SL000004E'):
		left_encode = 220 + deviate_l
		right_encode = 1400 + deviate_r
		direction = 3
	elif(command_str == 'SL000003E'):
		left_encode = 220 + deviate_l
		right_encode = 1400 + deviate_r
		direction = 3
	elif(command_str == 'SL000002E'):
		left_encode = 220 + deviate_l
		right_encode = 1400 + deviate_r
		direction = 3
	elif(command_str == 'SY000006E'):
		left_encode = 0
		right_encode = 2000 + deviate_r
		direction = 3
	elif(command_str == 'SY000005E'):
		left_encode = 0
		right_encode = 1600 + deviate_r
		direction = 3
	elif(command_str == 'SY000004E'):
		left_encode = 0
		right_encode = 1200 + deviate_r
		direction = 3
	elif(command_str == 'SY000003E'):
		left_encode = 0
		right_encode = 850 + deviate_r
		direction = 3
	elif(command_str == 'SY000002E'):
		left_encode = 0
		right_encode = 600 + deviate_r
		direction = 3
	elif(command_str == 'SD000006E'):
		left_encode = -2000 + deviate_l
		right_encode = 2000 + deviate_r
		direction = 3
	elif(command_str == 'SD000005E'):
		left_encode = -1600 + deviate_l
		right_encode = 1600 + deviate_r
		direction = 3
	elif(command_str == 'SD000004E'):
		left_encode = -1200 + deviate_l
		right_encode = 1200 + deviate_r
		direction = 3
	elif(command_str == 'SD000003E'):
		left_encode = -850 + deviate_l
		right_encode = 850 + deviate_r
		direction = 3
	elif(command_str == 'SD000002E'):
		left_encode = -600 + deviate_l
		right_encode = 600 + deviate_r
		direction = 3
	elif(command_str == 'SR000006E'):
		left_encode = 1400 + deviate_l
		right_encode = 220 + deviate_r
		direction = 4
	elif(command_str == 'SR000005E'):
		left_encode = 1400 + deviate_l
		right_encode = 220 + deviate_r
		direction = 4
	elif(command_str == 'SR000004E'):
		left_encode = 1400 + deviate_l
		right_encode = 220 + deviate_r
		direction = 4
	elif(command_str == 'SR000003E'):
		left_encode = 1400 + deviate_l
		right_encode = 220 + deviate_r
		direction = 4
	elif(command_str == 'SR000002E'):
		left_encode = 1400 + deviate_l
		right_encode = 220 + deviate_r
		direction = 4
	elif(command_str == 'SX000006E'):
		left_encode = 2000 + deviate_l
		right_encode = 0
		direction = 4
	elif(command_str == 'SX000005E'):
		left_encode = 1600 + deviate_l
		right_encode = 0
		direction = 4
	elif(command_str == 'SX000004E'):
		left_encode = 1200 + deviate_l
		right_encode = 0
		direction = 4
	elif(command_str == 'SX000003E'):
		left_encode = 850 + deviate_l
		right_encode = 0
		direction = 4
	elif(command_str == 'SX000002E'):
		left_encode = 600 + deviate_l
		right_encode = 0
		direction = 4
	elif(command_str == 'SC000006E'):
		left_encode = 2000 + deviate_l
		right_encode = -2000 + deviate_r
		direction = 4
	elif(command_str == 'SC000005E'):
		left_encode = 1600 + deviate_l
		right_encode = -1600 + deviate_r
		direction = 4
	elif(command_str == 'SC000004E'):
		left_encode = 1200 + deviate_l
		right_encode = -1200 + deviate_r
		direction = 4
	elif(command_str == 'SC000003E'):
		left_encode = 850 + deviate_l
		right_encode = -850 + deviate_r
		direction = 4
	elif(command_str == 'SC000002E'):
		left_encode = 600 + deviate_l
		right_encode = -600 + deviate_r
		direction = 4
	elif(command_str == 'SO00000OE'):
		ss.obstacle_avoidance_mode = 1
	elif(command_str == 'SW00000WE'):
		ss.obstacle_avoidance_mode = 0
	else:
		left_encode = 0
		right_encode = 0
		direction = 0


	rospy.loginfo("I heard %s: %d:%d direction: %d",command_str, left_encode, right_encode, direction)

def sonar_listener(data):
	global sonar_data
	sonar_data = [data.front_0, data.front_1, data.front_2, data.front_3, data.back_0, data.back_1, data.back_2, data.back_3]


def encoder_simulator():
	if burn_mode:
		rospy.loginfo("Robot hardware in burn mode, doing notheing")
		return
	#rospy.loginfo("Robot hardware in normal mode")
	global encoder_pub
	global left_encode
	global right_encode
	global velocity_vector
	global direction
	global status_pub
	global sonar_data, prev_has_obstacle
	global ss


	ss.has_obstacle = 0
	ss.over_obstacle = 0
	i = 0
	while i <= len(sonar_data)-1:
		for j in sonar_data[:4]:
			if j <= 1:
				ss.has_obstacle = 1
				break
		for k in sonar_data[4:]:
			if direction == 2 and k == 0:
				ss.has_obstacle = 1
				break
		i = i + 1
	if prev_has_obstacle == 1 and ss.has_obstacle == 0:
		ss.over_obstacle = 1
	prev_has_obstacle = ss.has_obstacle

	d_l = random.randint(-30, 30)
	d_r = random.randint(-30, 30)

	if ss.obstacle_avoidance_mode == 1:
		try:
			if sonar_data[0] == 0 or sonar_data[1] == 0 or sonar_data[2] == 0 or sonar_data[3] == 0:
				if sonar_data[4] == 0 or sonar_data[5] == 0 or sonar_data[6] == 0 or sonar_data[7] == 0:
					left_encode = 0
					right_encode = 0
					direction = 0
					rospy.loginfo("on obstacle")
				else:
					left_encode = -1200 + d_l
					right_encode = -1200 + d_r
					direction = 2
					rospy.loginfo("on obstacle")
			elif sonar_data[0] == 1 or sonar_data[1] == 1:
				if sonar_data[2] == 1 or sonar_data[3] == 1:
					left_encode = 1400 + d_l
					right_encode = 220 + d_r
					direction = 4
					rospy.loginfo("on obstacle")
				else:
					left_encode = 220 + d_l
					right_encode = 1400 + d_r
					direction = 3
					rospy.loginfo("on obstacle")
			elif sonar_data[2] == 1 or sonar_data[3] == 1:
				if sonar_data[0] == 1 or sonar_data[1] == 1:
					left_encode = 220 + d_l
					right_encode = 1400 + d_r
					direction = 3
					rospy.loginfo("on obstacle")
				else:
					left_encode = 1400 + d_l
					right_encode = 220 + d_r
					direction = 4
					rospy.loginfo("on obstacle")
			if direction == 2 and (sonar_data[4] == 0 or sonar_data[5] == 0 or sonar_data[6] == 0 or sonar_data[7] == 0):
				left_encode = 1200 + d_l
				right_encode = 1200 + d_r
				direction = 1
				rospy.loginfo("on obstacle")
			if ss.over_obstacle == 1:
				left_encode = 0
				right_encode = 0
				direction = 0
				rospy.loginfo("over obstacle")
		except IndexError:
			pass

	bytesToPublish = '%d %d' % (left_encode, right_encode)
	if(left_encode != 0  or right_encode != 0):
		rospy.loginfo(bytesToPublish)
	ee = Encoder()
	ee.left_encoder = left_encode
	ee.right_encoder = right_encode
	encoder_pub.publish(ee)
	

	ss.direction = direction
	status_pub.publish(ss)

	dt  		 		= 0.1
	vx 			 		= (float(left_encode)+float(right_encode))/(2.0 * dt)
	vy 			 		= 0.0
	vth 		 		= (float(left_encode)-float(right_encode)) / (614.0 * dt)
	velocity_vector.x 	= vx
	velocity_vector.y 	= vy
	velocity_vector.z 	= vth
	#velocity_pub.publish(velocity_vector)

def simulator():
	rospy.init_node('simulator', anonymous=True)
	rospy.Subscriber("command", String, executor_simulator)
	rospy.Subscriber('sonar', Sonar, sonar_listener)
	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		encoder_simulator()
		rate.sleep()

# spin() simply keeps python from exiting until this node is stopped
	rospy.spin()

if __name__ == '__main__':
	try:
		simulator()
	except rospy.ROSInterruptException:
		#ser.close()  #this doesn't seem to work well
		pass


