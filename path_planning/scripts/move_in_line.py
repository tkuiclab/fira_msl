#!/usr/bin/env python
import rospy

from actionlib import *
from geometry_msgs.msg import *
from fira_msl_msgs.msg import *

import numpy as np

class MoveInLine:
    def __init__(self, name):
        self.current_goal = None

        self._as = SimpleActionServer(name, GoToPoseAction, auto_start=False)
        self._as.register_goal_callback(self.goal_cb)
        self._as.register_preempt_callback(self.preempt_cb)
        self._as.start()

        self.sub = rospy.Subscriber("/robot_pose", Pose2D, self.go_to_cb)
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

        rospy.loginfo("Creating ActionServer [%s]", name)

    def goal_cb(self):
        self.current_goal = self._as.accept_new_goal()

    def preempt_cb(self):
        self._as.set_preempted()

    def go_to_cb(self, msg):
        if not self._as.is_active():
            return

        np_target = np.array([self.current_goal.pose2d.x, self.current_goal.pose2d.y])
        target_ang = np.arctan2([self.current_goal.pose2d.x], [self.current_goal.pose2d.y])[0] \
            if self.current_goal.face_dir else self.current_goal.pose2d.theta
        np_robot = np.array([msg.x, msg.y])

        if (np.linalg.norm(np_target - np_robot) > self.current_goal.dis_err):
            np_vector = np_target - np_robot
            np_norm = np_vector/np.linalg.norm(np_vector)
            np_vel = np_norm * 0.3
        else:
            np_vel = [0, 0]

        if (abs(target_ang - msg.theta) > 0.1):
            ang_vel = 0.2
            ang_vel = ang_vel if (target_ang - msg.theta > 0) else -ang_vel
        else:
            ang_vel = 0.0

        if np.linalg.norm(np_vel) == 0 and ang_vel == 0:
            self.pub.publish(Twist(Vector3(0, 0, 0), Vector3(0, 0, 0)))
            self._as.set_succeeded()
            return

        self.pub.publish(Twist(Vector3(np_vel[0], np_vel[1], 0), Vector3(0, 0, ang_vel)))


def main():
    rospy.init_node("move_in_line")
    MoveInLine(rospy.get_name())
    rospy.spin()

if __name__ == "__main__":
    main()
