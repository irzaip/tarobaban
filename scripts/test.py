#!/usr/bin/env python
# license removed for brevity
import rospy
from visualization_msgs.msg import *


def update_IM(x,y,z):
    while not rospy.is_shutdown():
        command = InteractiveMarkerFeedback()
        command.event_type = 1
        command.pose.position.x = x
        command.pose.position.y = y
        command.pose.position.z = z
        command.marker_name = "simple_6dof_MOVE_3D"
        rospy.loginfo(command)
        pub.publish(command)
        rate.sleep()

if __name__ == '__main__':

    pub = rospy.Publisher('/basic_controls/feedback', InteractiveMarkerFeedback, queue_size=10)
    rospy.init_node('IM_joystick', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    try:
       update_IM()
    except rospy.ROSInterruptException:
        pass