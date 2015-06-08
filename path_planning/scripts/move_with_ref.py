#!/usr/bin/env python
import rospy
import tf.transformations as tf_trans
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

class RefServer:
    def __init__(self, name):
        self._as = SimpleActionServer(name, MovingByPoseWithRefAction, execute_cb=self.execute_cb, auto_start=False)
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
                ref_target = self.tf_buff.transform(
                        PointStamped(Header(None, rospy.Time(), goal.ref_frame), Point(goal.pose2d.x, goal.pose2d.y, 0.0)),
                        'fira_msl1')
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rate.sleep()
                continue

            target_dis = np.linalg.norm([target.point.x, target.point.y])
            rotate_ang = math.tan(goal.dis_err/target_dis)
            vel = np.array([
                    target.point.x*math.cos(rotate_ang) - target.point.y*math.sin(rotate_ang),
                    target.point.x*math.sin(rotate_ang) + target.point.y*math.cos(rotate_ang)])

            vel_unit = vel/np.linalg.norm(vel)

            ref_angle = math.atan2(ref_target.point.y, ref_target.point.x)
            if abs(ref_angle) < 0.1:
                linear = Vector3(0.0, 0.0, 0.0)
            else:
                vel = vel_unit*sc.s_func(DIS_MAX, DIS_MIN, SPD_MAX, SPD_MIN, target_dis - goal.dis_err)
                linear = Vector3(vel[0], vel[1], 0)

            angle = math.atan2(target.point.y, target.point.x)

            if abs(angle) < 0.01:
                angular = 0.0
            else:
                angular = sc.s_func(ANG_MAX, ANG_MIN, W_MAX, W_MIN, abs(angle))
                angular = angular if angle > 0 else -angular

            self.pub.publish(Twist(linear, Vector3(0, 0, angular)))

            if np.linalg.norm(vel) == 0.0 and angular == 0.0:
                self._as.set_succeeded()
                break

            rate.sleep()


def main():
    rospy.init_node("move_with_ref")
    RefServer(rospy.get_name())
    rospy.spin()

if __name__ == "__main__":
    main()
