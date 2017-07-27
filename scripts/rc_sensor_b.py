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
    ser.port = "/dev/serial/by-id/usb-Arduino__www.arduino.cc__Arduino_Uno_8543836303935150E1B0-if00"
    ser.baudrate = 9600
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
    pub = rospy.Publisher('rc_sensor_b', String, queue_size = 10)
    rospy.init_node('rc_sensor_b', anonymous=True)
    rate = rospy.Rate(10)
    
    rospy.loginfo("Started reverse car sensor (back)")
    
    while not rospy.is_shutdown():
        if open_serial() == 0:
            rospy.loginfo("Not able to open serail port for the reverse sensor")
            status_pub.publish('rc_sencor_b 0')
            time.sleep(0.1)
            continue
        #log the time everytime a message from serial port 
        #start = datetime.now()
        #rospy.loginfo(str(start))
        bytesToRead = ser.readline()
        #rospy.log info(str(bytesToRead))
        bytesToRead = bytesToRead.strip('\r\n')

        # need to add necessary validation of data here 
        if len(bytesToRead)  == 7: 
            #separates the data into readable thing
            #rospy.loginfo(bytesToPublish)
            pub.publish(str(bytesToRead))
            status_pub.publish('rc_sensor_b 1')
        else:
            rospy.logwarn("Found data which is not in a required format")
            rospy.logwarn(bytesToRead)
            status_pub.publish('rc_sensor_b -1')

        #start = datetime.now()
        #rospy.loginfo(str(start))

if __name__ == '__main__':
    try:
        encoder()
    except rospy.ROSInterruptException:
        #ser.close()  #this doesn't seem to work well
        pass
        
        
