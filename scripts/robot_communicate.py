#!/usr/bin/env python
import rospy
import time
import serial
import string
from std_msgs.msg import String
import robot_listener 
import json
import math
import webbrowser
import os

def callback(data):
    json_str    = str(data.data)
    rospy.loginfo(json_str)
    try:
        decoded     = json.loads(json_str)
        url         = decoded['url']
        my_id       = decoded['robot_id']
        web_id      = decoded['control_id']
        # Open URL in a new tab, if a browser window is already open.
        #webbrowser.register('mozilla', Mozilla('mozilla'))
        os.system("killall firefox");

        time.sleep(0.1)
        webbrowser.open_new(url + '?robot_id=' + my_id + '&control_id=' + web_id)
        #webbrowser.open_new(url)
    except (ValueError, KeyError, TypeError):
        rospy.loginfo('JSON format error:')
        rospy.loginfo(json_str)
        
def executer():
    rospy.init_node('communication', anonymous=True)
    rospy.Subscriber("communicate", String, callback)
        #@yuqing_sendcommanddelay
        #while not rospy.is_shutdown():
        #   execute_command()

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    try:
        executer()
    except rospy.ROSInterruptException:
    #ser.close()  #this doesn't seem to work well
        pass
