#!/usr/bin/env python
import rospy
import string
from std_msgs.msg import String

def send_lock_command():
	pub = rospy.Publisher('command', String, queue_size=10)
	rospy.init_node('lock_cpp', anonymous=True)
	rate = rospy.Rate(1)

	stringToSend = 'lock'

	for i in range (0,5):
		if not rospy.is_shutdown():
                	pub.publish(stringToSend)
		rate.sleep()
	
if __name__ == '__main__':
	try:
		send_lock_command()
	except rospy.ROSInterruptException:
		pass
