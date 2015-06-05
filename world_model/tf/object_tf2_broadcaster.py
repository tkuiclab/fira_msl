#!/usr/bin/env python
import rospy

import tf
import tf2_ros

from std_msgs.msg import *
from geometry_msgs.msg import *

def handle_object_pose(msg, name):
    br = tf2_ros.TransformBroadcaster()
    [x, y, z, w] = tf.transformations.quaternion_from_euler(0, 0, msg.theta)
    br.sendTransform(TransformStamped(
            Header(None, rospy.Time.now(), "world"),
            name,
            Transform(Vector3(msg.x, msg.y, 0), Quaternion(x, y, z, w))))

if __name__ == '__main__':
    rospy.init_node('object_tf_broadcaster')
    object_name = rospy.get_param('~object_name')
    rospy.Subscriber('/%s/pose'%object_name,
                     Pose2D,
                     handle_object_pose,
                     object_name)
    rospy.spin()
