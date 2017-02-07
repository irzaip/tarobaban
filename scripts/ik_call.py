#!/usr/bin/env python

import sys
import rospy
from tarobaban.srv import *


ret0 = 0.0
ret1 = 0.0
ret2 = 0.0

def give_angles(x, y, z):
    rospy.wait_for_service('get_angle')
    try:
        get_angles = rospy.ServiceProxy('get_angle', get_angle)
        resp1 = get_angles(x, y, z)
        return 
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


if __name__ == "__main__":

	for x in range(len(a)):
		give_angles(a[x], b[x], c[x])
		print "Requesting %s,%s,%s"%(a[x],b[x], c[x])
    