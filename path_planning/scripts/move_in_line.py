#!/usr/bin/env python
import rospy
import tf
import tf2_ros
import tf2_geometry_msgs
import math

from actionlib import *
from geometry_msgs.msg import *
from fira_msl_msgs.msg import *

import numpy as np

import speed_control as sc

SPD_MAX = 1.0
SPD_MIN = 0.1
DIS_MAX = 1.0
DIS_MIN = 0.1

W_MAX = math.pi*2
W_MIN = 0.1
ANG_MAX = math.pi/2
ANG_MIN = 0.005

class MoveInLine:
    def __init__(self, name):
        self._as = SimpleActionServer(name, MovingByPoseAction, execute_cb=self.execute_cb, auto_start=False)
        self._as.start()

        self.tf_buff = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buff)
        self.pub = rospy.Publisher("/fira_msl1/cmd_vel", Twist, queue_size=1)

        rospy.loginfo("Creating ActionServer [%s]", name)

    def execute_cb(self, goal):
        rate = rospy.Rate(10.0)
        while True:
            if self._as.is_preempt_requested():
                self._as.set_preempted()
                break

            try:
                target = self.tf_buff.transform(
                        PointStamped(Header(None, rospy.Time(), goal.frame), Point(goal.pose2d.x, goal.pose2d.y, 0.0)),
                        'fira_msl1')
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rate.sleep()
                continue

            target_dis = np.linalg.norm([target.point.x, target.point.y])
            vel_unit = [target.point.x, target.point.y]/target_dis
            if np.linalg.norm([target.point.x, target.point.y]) < goal.dis_err:
                linear = Vector3(0.0, 0.0, 0.0)
            else:
                vel = vel_unit*sc.s_func(DIS_MAX, DIS_MIN, SPD_MAX, SPD_MIN, target_dis)
                linear = Vector3(vel[0], vel[1], 0)

            angular = math.atan2(target.point.y, target.point.x) + goal.pose2d.theta

            if abs(angular) < 0.01:
                angular = 0.0
            else:
                angular = sc.s_func(ANG_MAX, ANG_MIN, W_MAX, W_MIN, abs(angle))
                angular = angular if angular > 0 else -angular

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
