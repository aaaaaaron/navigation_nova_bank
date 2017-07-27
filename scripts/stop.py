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


def send_command():
	#handle the format of the string
	stringToSend = 'SP000000E\n' #might need to add \n behind the E
	#sending the string
	if ser.isOpen():
		ser.write(stringToSend)
	


if __name__ == '__main__':
	try:
		# AAron's initial one for final testing
		#job_generator(initial_bearing, loops)
		send_command()
	except rospy.ROSInterruptException:
		pass
