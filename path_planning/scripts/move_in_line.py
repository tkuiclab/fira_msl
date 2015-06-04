#!/usr/bin/env python
import rospy
import tf
import math

from actionlib import *
from geometry_msgs.msg import *
from fira_msl_msgs.msg import *

import numpy as np

class MoveInLine:
    def __init__(self, name):
        self._as = SimpleActionServer(name, GoToPoseAction, execute_cb=self.execute_cb, auto_start=False)
        self._as.start()

        self.tf_listener = tf.TransformListener()
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

        rospy.loginfo("Creating ActionServer [%s]", name)

    def execute_cb(self, goal):
        rate = rospy.Rate(10.0)
        while True:
            if self._as.is_preempt_requested():
                self._as.set_preempted()
                break

            try:
                now = rospy.Time.now()
                self.tf_listener.waitForTransform('/self_frame', '/ball_frame', now, rospy.Duration(4.0))
                (trans, rot) = self.tf_listener.lookupTransform('/self_frame', '/ball_frame', now)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

            if np.linalg.norm(np.array([trans[0], trans[1]])) < 0.3:
                linear = Vector3(0.0, 0.0, 0.0)
            else:
                linear = Vector3(trans[0], trans[1], 0)

            angular = math.atan2(trans[1], trans[0])

            if abs(angular) < 0.2:
                angular = 0.0
            else:
                angular = 1.5 if angular > 0 else -1.5

            self.pub.publish(Twist(linear, Vector3(0, 0, angular)))

            if linear.x == 0.0 and linear.y == 0.0 and angular == 0.0:
                self._as.set_succeeded()
                break

            rate.sleep()


def main():
    rospy.init_node("move_in_line")
    MoveInLine(rospy.get_name())
    rospy.spin()

if __name__ == "__main__":
    main()
