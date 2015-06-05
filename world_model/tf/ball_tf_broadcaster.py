#!/usr/bin/env python
import rospy

import tf
from geometry_msgs.msg import *

def handle_ball_pose(msg, name):
    br = tf.TransformBroadcaster()
    br.sendTransform((msg.x, msg.y, 0),
                     tf.transformations.quaternion_from_euler(0, 0, 0),
                     rospy.Time.now(),
                     name,
                     "world")

if __name__ == '__main__':
    rospy.init_node('ball_tf_broadcaster')
    rospy.Subscriber('/soccer/pose',
                     Pose2D,
                     handle_ball_pose,
                     'ball_frame')
    rospy.spin()
