#!/usr/bin/env python

import sys
import rospy
from navigation_nova_bank.srv import *

def client():
    rospy.wait_for_service('get_ip')
    try:
        get_ip = rospy.ServiceProxy('get_ip', GetIP)
        resp1 = get_ip() 
        return resp1.ip
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def usage():
    return "%s [x y]"%sys.argv[0]

if __name__ == "__main__":
    print "%s"%(client())
