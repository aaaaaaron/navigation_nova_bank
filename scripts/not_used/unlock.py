#!/usr/bin/env python
import rospy
import serial
import string
from std_msgs.msg import String

ser = serial.Serial()
ser.port = "/dev/serial/by-id/usb-Arduino__www.arduino.cc__Arduino_Uno_75630313536351217041-if00"
#depends on the device port name
ser.baudrate = 115200
ser.open()


def send_unlock_command():
	stringToSend = 'iap_jump_app\r\n'
	if ser.isOpen():
		ser.write(stringToSend)
        else:
		rospy.loginfo("Commanding Serial port not connected")


def send_command():
	#handle the format of the string
	stringToSend = 'SF000006E\0' #might need to add \n behind the E
	#sending the string
	if ser.isOpen():
		ser.write(stringToSend)
	else:
		rospy.loginfo("Commanding Serial port not connected")


if __name__ == '__main__':
	try:
		# AAron's initial one for final testing
		#job_generator(initial_bearing, loops)
		send_unlock_command()
		#send_command()
	except rospy.ROSInterruptException:
		pass
