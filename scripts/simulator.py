#!/usr/bin/env python
# The class is to simulate the robot b

import rospy
import serial
import string
import sys
from geometry_msgs.msg import Vector3
from serial_handler.msg import Encoder

from datetime import datetime

from std_msgs.msg import String

encoder_pub 		= rospy.Publisher('encoder', Encoder, queue_size = 1000)
#velocity_pub  		= rospy.Publisher('velocity', Vector3, queue_size = 1)
velocity_vector  	= Vector3()
velocity_vector.x  	= 0.0
velocity_vector.y 	= 0.0
velocity_vector.z 	= 0.0

left_encode   	= 0
right_encode 	= 0
burn_mode		= True

def executor_simulator(data):
	global burn_mode
	global left_encode
	global right_encode
	command_str = str(data.data)
	command_str = command_str.rstrip('\0')
	command_str = command_str.rstrip('\n')
	command_str = command_str.rstrip('\r')
	rospy.loginfo("recieved command %s", command_str)
	rospy.loginfo("--------------------------")
	if (command_str == 'iap_jump_app'):
		rospy.loginfo('Turn off burn mode')
		burn_mode = False
	elif (command_str == 'iap'):
		rospy.loginfo("Turn to burn mode")
		burn_mode = True
	elif (command_str == 'SF000006E'):
		left_encode = 4000
		right_encode  = 3920
	elif (command_str == 'SF000005E'):
		left_encode = 1444
		right_encode  = 1444
	elif(command_str == 'SF000004E'):
		left_encode = 1108
		right_encode = 1108
	elif(command_str == 'SF000003E'):
		left_encode = 867
		right_encode = 867
	elif(command_str == 'SF000002E'):
		left_encode = 600
		right_encode = 588
	elif(command_str == 'SB000006E'):
		left_encode = -2000
		right_encode  = -2000
	elif(command_str == 'SB000005E'):
		left_encode = -1444
		right_encode  = -1444
	elif(command_str == 'SB000004E'):
		left_encode = -1108
		right_encode = -1108
	elif(command_str == 'SB000003E'):
		left_encode = -867
		right_encode = -867
	elif(command_str == 'SB000002E'):
		left_encode = -666
		right_encode = -666
	elif(command_str == 'SL000006E'):
		left_encode = -1500
		right_encode  = 1500
	elif(command_str == 'SL000005E'):
		left_encode = -1108
		right_encode  = 1108
	elif(command_str == 'SL000004E'):
		left_encode = -888
		right_encode = 888
	elif(command_str == 'SL000003E'):
		left_encode = -666
		right_encode = 666
	elif(command_str == 'SL000002E'):
		left_encode = -444
		right_encode = 444
	elif(command_str == 'SR000006E'):
		left_encode = 1500
		right_encode  = -1500
	elif(command_str == 'SR000005E'):
		left_encode = 1108
		right_encode = -1108
	elif(command_str == 'SR000004E'):
		left_encode = 888
		right_encode = -888
	elif(command_str == 'SR000003E'):
		left_encode = 666
		right_encode = -666
	elif(command_str == 'SR000002E'):
		left_encode = 444
		right_encode = -444
	else:
		left_encode = 0
		right_encode = 0


	rospy.loginfo("I heard %s: %d:%d",command_str, left_encode, right_encode)

def encoder_simulator():
	if burn_mode:
		rospy.loginfo("Robot hardware in burn mode, doing notheing")
		return
	#rospy.loginfo("Robot hardware in normal mode")
	global encoder_pub
	global left_encode
	global right_encode
	global velocity_vector
	bytesToPublish = '%d %d' % (left_encode, right_encode)
	if(left_encode != 0  or right_encode != 0):
		rospy.loginfo(bytesToPublish)
	ee = Encoder()
	ee.left_encoder = left_encode
	ee.right_encoder = right_encode
	encoder_pub.publish(ee)

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


