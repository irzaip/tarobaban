#!/usr/bin/env python
from tarobaban.srv import *
import random
import time
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

ang0 = 0.0
ang1 = 0.0
ang2 = 0.0
ang0_to = 0.0
ang1_to = 0.0
ang2_to = 0.0



def handle_get_angle(req):
	ang0 = req.a
	ang1 = req.b
	ang2 = req.c
	a_to = req.a
	b_to = req.b
	c_to = req.c
	hello_str = JointState()
	hello_str.header = Header()
	hello_str.header.stamp = rospy.Time.now()
	hello_str.name = ['base_link_to_base_body', 'base_to_lower_arm', 'lower_arm_to_upper_arm']
	hello_str.position = [ang0, ang1, ang2]
	hello_str.velocity = []
	hello_str.effort = []
	hello_str.header.stamp = rospy.Time.now()
	pub.publish(hello_str)
	rate.sleep()

	return get_angleResponse(a_to,b_to,c_to)


def talker():
    
	s = rospy.Service('get_angle', get_angle, handle_get_angle)
	#pub = rospy.Publisher('joint_states', JointState, queue_size=10)


	#while not rospy.is_shutdown():

	rospy.spin()

if __name__ == '__main__':
	rospy.init_node('joint_state_publisher')
   	pub = rospy.Publisher('joint_states', JointState, queue_size=10)
	rate = rospy.Rate(10) # 10hz

	try:
		talker()
	except rospy.ROSInterruptException:
		pass
