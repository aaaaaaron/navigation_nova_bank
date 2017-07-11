#!/usr/bin/env python
import rospy
import serial
import string
import time
import robot_publisher
from std_msgs.msg import String

def send_command():
	rospy.init_node('back',anonymous=True)
	#handle the format of the string
	#commandToPub = 'SB000006E' #might need to add \n behind the E
	#publishing the command as a string
	robot_publisher.publish_command('B', 5)
	time.sleep(1)
	robot_publisher.publish_command('P', 0)

if __name__ == '__main__':
	try:
		# AAron's initial one for final testing
		#job_generator(initial_bearing, loops)
		send_command()
	except rospy.ROSInterruptException:
		pass
