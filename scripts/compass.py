#!/usr/bin/env python

import rospy
import serial
import string
import time
from std_msgs.msg import String

ser = serial.Serial()

status_pub = rospy.Publisher('status', String, queue_size = 100)

def open_serial():
    	global ser
    	if ser.isOpen():
        	return 1 
    	ser.port = "/dev/serial/by-id/usb-Arduino__www.arduino.cc__Arduino_Uno_75435363138351A09171-if00"
    	ser.baudrate = 9600
    	try:
    		ser.open()
    	except serial.serialutil.SerialException as ex:
			rospy.logerr(ex)
			return 0 
    	if ser.isOpen():
        	return 1
    	return 0 

def compass():
	global ser
	pub = rospy.Publisher('compass', String, queue_size = 100)
	rospy.init_node('compass', anonymous=True)
	rate = rospy.Rate(100)
	#rospy.loginfo(str("test"))
	while not rospy.is_shutdown():
		#rospy.loginfo(str("Reading data from serial port"))
		if(open_serial() == 0):
			rospy.loginfo("Waiting serail port to open...")
			status_pub.publish("compass 0")
			time.sleep(0.1)
			continue
		bytesToRead = ser.readline()
		#rospy.loginfo(bytesToRead)
		bytesToRead = bytesToRead.strip('\n')
		if len(bytesToRead)  > 0: 
			#removes all alphabets turns into integer to remove 00
			all = string.maketrans('','')
			nodigs = all.translate(all, string.digits)
			bytesToRead = bytesToRead.translate(all, nodigs)
			bytesToPublish = int(bytesToReadnot)
			#publishing data in string for standardization
			#rospy.loginfo(str(bytesToRead))
			pub.publish(str(bytesToPublish))
			status_pub.publish("compass 1")
		else:
			status_pub.publish("compass -1")
		rate.sleep()

if __name__ == '__main__':
	try:
		compass()
	except rospy.ROSInterruptException:
		#ser.close()  #this doesn't seem to work well
		pass
		
		
