#!/usr/bin/env python  
import roslib
roslib.load_manifest('learning_tf')
import rospy

import tf
robot = "Tarobaban"

def arm_pose(msg, robot):
    br = tf.TransformBroadcaster()
    br.sendTransform((msg.x, msg.y, 0),
                     tf.transformations.quaternion_from_euler(0, 0, 0),
                     rospy.Time.now(),
                     robot,
                     "world")

if __name__ == '__main__':
    rospy.init_node('tf_inv_kin')
    
    rospy.Subscriber('/joint_states', robot)
    rospy.spin()