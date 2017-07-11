import rospy
import serial
import string
import robot_drive
import robot_obstacle
import json
from std_msgs.msg import String

pub_param 		= rospy.Publisher('parameters', String, queue_size = 1)
pub_gps			= rospy.Publisher('gps', 	String, queue_size=10)
pub_command 	= rospy.Publisher('command', 	String, queue_size=1)
pub_chat 		= rospy.Publisher('chat', String, queue_size = 1)

# Used to publish parameters
def publish_parameters():
	if(not robot_drive.robot_moving and not robot_drive.robot_turning):
		return
	#@yuqing_publishparam
	info={}
	info["ENABLE"]      =    robot_drive.robot_enabled
	info["MOVING"]      =    robot_drive.robot_moving
	info["PAUSED"]      =    robot_drive.robot_paused
	info["MISSION"]     =    robot_drive.robot_on_mission
	info["OBSTACLE"]    =    robot_obstacle.robot_on_obstacle
	info["DIRECTION"]   =    robot_drive.move_direction
	info["SPEED"]       =    robot_drive.speed_now
	info["LONG"]        =    robot_drive.lon_now
	info["LAT"]         =    robot_drive.lat_now
	info["BEARING"]     =    robot_drive.bearing_now
	info["BATTERY"] 	= 	 robot_drive.battery_level #aaron added
	data={}
	data["parameters"]  =    info

	parameters = json.dumps(data)
	#rospy.loginfo(parameters)
	pub_param.publish(parameters)

#Used to publish chat related parameters, based on the parameters, the web page would decide what to do
def publish_chat():
	info = {}
	info["TYPE"]  	= 0
	info["ACTION"] 	= 1
	data={}
	data["chat"] = info
	chat_para = json.dumps(data)
	rospy.loginfo(chat_para)
	pub_chat.publish(chat_para)

def publish_gps():
	stringToSend = '%f %f %f' % (robot_drive.lon_now, robot_drive.lat_now, robot_drive.bearing_now) #might need to add \n behind the E
	pub_gps.publish(stringToSend)


# Helper function which can send commands to robot
def publish_command(command_string, speed):
	#sending the string
	#handle the format of the string
	stringToSend = 'S%s00000%dE\n' % (command_string, speed) #might need to add \n behind the E
	pub_command.publish(stringToSend)
	rospy.loginfo(str(stringToSend))
