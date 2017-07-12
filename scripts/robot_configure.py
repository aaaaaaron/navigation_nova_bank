#!/usr/bin/env python
import rospy
import string
import ConfigParser
import os
import robot_configure
import robot_drive
import robot_job
import robot_turn
import robot_move
import robot_correction

from std_msgs.msg import String

def read_config(config_file_path, field, key):
    cf = ConfigParser.ConfigParser()
    ret = True
    result = None
    try:
        cf.read(config_file_path)
        result = cf.get(field, key)
    except:
    	ret = False
        rospy.loginfo("Failed to read %s-%s", field, key)
    return ret, result

def read_config_float(config_file_path, field, key):
    cf = ConfigParser.ConfigParser()
    ret = True
    val = 0
    try:
        cf.read(config_file_path)
        result = cf.get(field, key)
        val = float(result)
    except:
        ret = False
        rospy.loginfo("Failed to read %s-%s", field, key)
    return ret, val

def write_config(config_file_path, field, key, value):
    cf = ConfigParser.ConfigParser()
    ret = True
    try:
        cf.read(config_file_path)
        cf.set(field, key, value)
        cf.write(open(config_file_path,'w'))
    except:
    	ret = False
        rospy.loginfo("Failed to write parameters")
    return ret


def read_system_config():
    # Read configure path
    print("Read configuration file")
    config_path = os.path.dirname(os.path.abspath(__file__)) + '/robot.cfg'
    size_para   = 24
    ret         = [None] * size_para

    # Now reading configurable parameters
    # [tuning parameters] related
    ret[0], robot_drive.linear_encode_to_mm             = read_config_float(config_path, 'tuning', 'linear_encode_to_mm')
    ret[1], robot_drive.turning_encode_to_mm            = read_config_float(config_path, 'tuning', 'turning_encode_to_mm')
    # [mechanical related]
    ret[2], robot_drive.turn_radius                     = read_config_float(config_path, 'mechanic', 'turn_radius')
    # [correction]
    ret[3], robot_correction.min_correction_distance    = read_config_float(config_path, 'correction', 'min_correction_distance')
    ret[4], robot_correction.min_correction_angle       = read_config_float(config_path, 'correction', 'min_correction_angle')
    ret[5], robot_correction.max_correction_runs        = read_config_float(config_path, 'correction', 'max_correction_runs')
    # [move]
    ret[6], robot_move.dist_to_correct                  = read_config_float(config_path, 'move', 'dist_to_correct')
    ret[7], robot_move.dist_lower_speed                 = read_config_float(config_path, 'move', 'dist_lower_speed')
    ret[8], robot_move.dist_lowest_speed                = read_config_float(config_path, 'move', 'dist_lowest_speed')
    ret[9], robot_move.linear_full_speed                = read_config_float(config_path, 'move', 'linear_full_speed')
    ret[10], robot_move.linear_lower_speed              = read_config_float(config_path, 'move', 'linear_lower_speed')
    ret[11], robot_move.linear_lowest_speed             = read_config_float(config_path, 'move', 'linear_lowest_speed')
    # [turn]
    ret[12], robot_turn.angle_lower_speed               = read_config_float(config_path, 'turn', 'angle_lower_speed')
    ret[13], robot_turn.angle_lowest_speed              = read_config_float(config_path, 'turn', 'angle_lowest_speed')
    ret[14], robot_turn.turn_full_speed                 = read_config_float(config_path, 'turn', 'turn_full_speed')
    ret[15], robot_turn.turn_lower_speed                = read_config_float(config_path, 'turn', 'turn_lower_speed')
    ret[16], robot_turn.turn_lowest_speed               = read_config_float(config_path, 'turn', 'turn_lowest_speed')
    # [init]
    ret[17], robot_drive.obstacle_mode                  = read_config_float(config_path, 'init', 'obstacle_mode')
    ret[18], robot_drive.robot_enabled                  = read_config_float(config_path, 'init', 'robot_enabled')
    ret[19], robot_drive.robot_paused                   = read_config_float(config_path, 'init', 'robot_paused')
    ret[20], robot_job.init_lon                         = read_config_float(config_path, 'init', 'init_lon')
    ret[21], robot_job.init_lat                         = read_config_float(config_path, 'init', 'init_lat')
    ret[22], robot_job.init_bearing                     = read_config_float(config_path, 'init', 'init_bearing')

    ret[23], robot_drive.bank_radius                    = read_config_float(config_path, 'mechanic', 'bank_radius')

    # check whether the reading is successful or not
    for index in range(size_para):
        if not ret[index]:
            print("The ",index,"parameter is not correct")

    print("Finished read configure file")
    return
