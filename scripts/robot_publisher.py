import rospy
import serial
import string
import robot_drive
import robot_obstacle
import robot_correction
import json
import coordTransform_utils
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Vector3

pub_param 		= rospy.Publisher('parameters', String, queue_size = 1)
pub_gps			= rospy.Publisher('gps', 	String, queue_size=10)
pub_command 	= rospy.Publisher('command', 	String, queue_size=1)
pub_chat 		= rospy.Publisher('chat', String, queue_size = 1)
# pub_gps_gaode	= rospy.Publisher('gps_gaode', NavSatFix, queue_size = 10)
pub_pose_pf 	= rospy.Publisher('pose_bef_pf', Vector3, queue_size = 10)
count = 0

def publish_pose_pf(lon, lat, bearing):
	xy_theta = Vector3()
	xy_theta.x = lon
	xy_theta.y = lat
	xy_theta.z = bearing
	pub_pose_pf.publish(xy_theta)

# Used to publish gcj02 coordinates for gaode_map
# def publish_gps_gaode(lon, lat):
# 	nsf = NavSatFix()
# 	nsf.longitude = lon
# 	nsf. latitude = lat
# 	pub_gps_gaode.publish(nsf)


# Used to publish parameters
def publish_parameters():
	global count 
	if(not robot_drive.robot_moving and not robot_drive.robot_turning and count < 10):
		count = count + 1
		return
	#@yuqing_publishparam
#-------------------------------------------------------------------------------------------------------------------------------------------chengyuen11/10
	if not robot_correction.map_wgs84 and not robot_correction.follow_map_gps:
		lon_lat = coordTransform_utils.wgs84_to_gcj02(robot_drive.lon_now, robot_drive.lat_now) 
	else:
		lon_lat = [robot_drive.lon_now, robot_drive.lat_now]
#-------------------------------------------------------------------------------------------------------------------------------------------
		
	info={}
	info["ENABLE"]      =    robot_drive.robot_enabled
	info["MOVING"]      =    robot_drive.robot_moving
	info["PAUSED"]      =    robot_drive.robot_paused
	info["MISSION"]     =    robot_drive.robot_on_mission
	info["OBSTACLE"]    =    robot_obstacle.robot_on_obstacle
	info["DIRECTION"]   =    robot_drive.move_direction
	info["SPEED"]       =    robot_drive.speed_now
	info["LONG"]        =    lon_lat[0]
	info["LAT"]         =    lon_lat[1]
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
	info["ID"]      = robot_drive.robot_id 
	userType 	= 'admin';
	info["CLIENT"]  =  userType + str(robot_drive.robot_id); 
	data={}
	data["chat"] = info
	chat_para = json.dumps(data)
	rospy.loginfo(chat_para)
	pub_chat.publish(chat_para)

def publish_gps():
#-------------------------------------------------------------------------------------------------------------------------------------------chengyuen11/10
	if not robot_correction.map_wgs84 and not robot_correction.follow_map_gps:
		lon_lat = coordTransform_utils.wgs84_to_gcj02(robot_drive.lon_now, robot_drive.lat_now) 
	else:
		lon_lat = [robot_drive.lon_now, robot_drive.lat_now]
#-------------------------------------------------------------------------------------------------------------------------------------------
		
	stringToSend = '%.10f %.10f %.10f' % (lon_lat[0], lon_lat[1], robot_drive.bearing_now) #might need to add \n behind the E
	pub_gps.publish(stringToSend)


def publish_command(odd_even_location, dist, angle):
	dist = int(round(dist/10.0))
	if dist > 255:
		dist = 255
	dist_str = str(dist)
	if len(dist_str) < 3:
		dist_str = (3-len(dist_str))*'0' + dist_str
	angle_str = str(angle)
	if len(angle_str) < 3:
		angle_str = (3-len(angle_str))*'0' + angle_str
	stringToSend = 'S%d%s%sE\n'%(odd_even_location, dist_str, angle_str)
	pub_command.publish(stringToSend)
	rospy.loginfo(str(stringToSend))

# Helper function which can send commands to robot
def publish_command_old(command_string, speed):
	#sending the string
	#handle the format of the string
	stringToSend = 'S%s00000%dE\n' % (command_string, speed) #might need to add \n behind the E
	pub_command.publish(stringToSend)
	rospy.loginfo(str(stringToSend))
