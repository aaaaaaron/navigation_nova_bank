#!/usr/bin/env python
import rospy
import string
import math
import gpsmath
import robot_drive
import robot_turn
import robot_move
import robot_correction
from datetime import datetime
import os

yy = str(datetime.now().year)
mm = str(datetime.now().month)
if len(mm) < 2:
	mm = "0%s"%mm
dd = str(datetime.now().day)
if len(dd) < 2:
	dd = "0%s"%dd
hh = str(datetime.now().hour)
if len(hh) < 2:
	hh = "0%s"%hh
m  = str(datetime.now().minute)
if len(m) < 2:
	m = "0%s"%m
ss = str(datetime.now().second)
if len(ss) < 2:
	ss = "0%s"%ss
filename = "%s%s%s_%s%s%s"%(yy,mm,dd,hh,m,ss)
f = open(os.path.dirname(os.path.abspath(__file__)) + '/log/' + filename + '.txt', 'a')

def write_to_file(data):
	f.write(data)