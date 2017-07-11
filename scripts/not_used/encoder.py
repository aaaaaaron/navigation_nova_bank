#!/usr/bin/env python
import rospy
import serial
import string
import time

from datetime import datetime

from std_msgs.msg import String

ser = serial.Serial()


#add a timer to measure the serial port read line waiting time
start = 0.0
end = 0.0

status_pub = rospy.Publisher('status', String, queue_size = 100)


def open_serial():
	global ser
	if ser.isOpen():
		return 1 

	#real robot port
	ser.port = "/dev/serial/by-id/usb-Arduino__www.arduino.cc__Arduino_Uno_75630313536351217041-if00"
	ser.baudrate = 115200
	#testing port
	#ser.port = "/dev/serial/by-id/usb-Arduino__www.arduino.cc__Arduino_Uno_75439333335351412220-if00"
	#ser.baudrate = 9600	
	#ser.timeout = 1.0
	try:
    		ser.open()
	except serial.serialutil.SerialException as ex:
		rospy.logerr(ex)
		return 0 
	if ser.isOpen():
		return 1
	return 0 

def encoder():
	global ser
	pub = rospy.Publisher('encoder', String, queue_size = 10)
	rospy.init_node('encoder', anonymous=True)
	rate = rospy.Rate(10)
	
	rospy.loginfo("Started encoder")
	
	while not rospy.is_shutdown():
		if open_serial() == 0:
			rospy.loginfo("Not able to open serail port")
			status_pub.publish('encoder 0')
			time.sleep(0.1)
			continue
		#log the time everytime a message from serial port 
		#start = datetime.now()
		#rospy.loginfo(str(start))
		bytesToRead = ser.readline()
		#rospy.log info(str(bytesToRead))
		bytesToRead = bytesToRead.strip('\n')
		rospy.loginfo(bytesToRead)
		if len(bytesToRead)  == 17: 
			#separates the data into readable things
                      	#rospy.loginfo(str(bytesToRead))
			l_encoder, l_direction, r_encoder, r_direction = bytesToRead.split(" ")
			nr_encoder = int(r_encoder)
			nl_encoder = int (l_encoder)
			if r_direction == "0" : # it means the reverse direction 
				nr_encoder = int(nr_encoder)
			else:
				nr_encoder = -int(nr_encoder)
			if l_direction == "0": 
				nl_encoder = int(nl_encoder)
			else :
				nl_encoder = -int(nl_encoder)

			#turning them into strings
                                #real data 
            # Publish the data as left right format, direction of data are reflected on the sign
			bytesToPublish = '%d %d' % (nl_encoder, nr_encoder)

			if(nl_encoder != 0  or nr_encoder != 0):
			 	rospy.loginfo(bytesToPublish)
			pub.publish(str(bytesToPublish))
			status_pub.publish('encoder 1')
		else:
			rospy.logwarn("Found data which is not in a required format")
			rospy.logwarn(bytesToRead)
			status_pub.publish('encoder -1')

		#start = datetime.now()
		#rospy.loginfo(str(start))

if __name__ == '__main__':
	try:
		encoder()
	except rospy.ROSInterruptException:
		#ser.close()  #this doesn't seem to work well
		pass
		
		
