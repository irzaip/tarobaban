#!/usr/bin/env python
from tarobaban.srv import *
import random
import time
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import numpy as np


def sin_step_ramp(currentPosition,targetPosition,div=1):
	''' buat STEP dari currentPosition sampai dengan targetPosition
	dengan bantuan fungsi sinus. berguna membuat ramp yang gradasi
	seperti kurva sinus. 
	div dipakai sebagai resolusi pembagian,
	semakin besar div, semakin kecil jarak step
	return sebagai array
	'''
	arr = []
	r = targetPosition - currentPosition
	angles = np.array( (range(190)) [0::1*div]) - 90
	m = ( np.sin( angles * np.pi / 180. ) + 1 ) /2

	for mi in np.nditer(m):
		pos = currentPosition + mi*r
		arr.append(pos)
		#print "pos: ", pos 
	return arr



ang0 = 0.0
ang1 = 0.0
ang2 = 0.0

a_max = 2.2
b_max = 2.2
c_max = 2.2

a_min = -2.2
b_min = -2.2
c_min = -2.2



def handle_get_angle(req):
	global ang0,ang1,ang2,a_max,b_max,c_max,a_min,b_min,c_min

	#set target angle
	a_to = req.a
	b_to = req.b
	c_to = req.c

	#rubah target apabila melebihi kapasitas robot
	if (a_to > a_max): a_to = a_max
	if (b_to > b_max): b_to = b_max
	if (c_to > c_max): c_to = c_max
	if (a_to < a_min): a_to = a_min
	if (b_to < b_min): b_to = b_min
	if (c_to < c_min): c_to = c_min

	#buat matrix step
	a_step = sin_step_ramp(ang0,a_to,3)
	b_step = sin_step_ramp(ang1,b_to,3)
	c_step = sin_step_ramp(ang2,c_to,3)

	#lakukan perubahan
	for x in range(len(a_step)):
		instruction = JointState()
		instruction.header = Header()
		instruction.header.stamp = rospy.Time.now()
		instruction.name = ['base_link_to_base_body', 'base_to_lower_arm', 'lower_arm_to_upper_arm']
		instruction.position = [a_step[x], b_step[x], c_step[x]]
		instruction.velocity = []
		instruction.effort = []
		instruction.header.stamp = rospy.Time.now()
		pub.publish(instruction)
		ang0 = a_step[x]
		ang1 = b_step[x]
		ang2 = c_step[x]
		rate.sleep()

	ang0 = a_to
	ang1 = b_to
	ang2 = c_to
	return get_angleResponse(ang0,ang1,ang2)


def talker():
    
	s = rospy.Service('get_angle', get_angle, handle_get_angle)
	#pub = rospy.Publisher('joint_states', JointState, queue_size=10)


	#while not rospy.is_shutdown():
	rospy.loginfo("Service /get_angle started")
	rospy.spin()

if __name__ == '__main__':
	rospy.init_node('joint_state_publisher')
   	pub = rospy.Publisher('joint_states', JointState, queue_size=10)
	rate = rospy.Rate(40) # 10hz

	try:
		talker()
	except rospy.ROSInterruptException:
		pass
