#!/usr/bin/env python
import rospy

import tf
from geometry_msgs.msg import *

def handle_robot_pose(msg, name):
    br = tf.TransformBroadcaster()
    br.sendTransform((msg.x, msg.y, 0),
                     tf.transformations.quaternion_from_euler(0, 0, msg.theta),
                     rospy.Time.now(),
                     name,
                     "world")

if __name__ == '__main__':
    rospy.init_node('robot_tf_broadcaster')
    robot_name = rospy.get_param('~robot_name')
    rospy.Subscriber('/robot_pose',
                     Pose2D,
                     handle_robot_pose,
                     robot_name)
    rospy.spin()
