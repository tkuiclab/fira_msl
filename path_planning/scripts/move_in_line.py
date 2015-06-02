#!/usr/bin/env python
import rospy
from rospy.numpy_msg import *

from actionlib import *
from geometry_msgs.msg import *
from fira_msl_msgs.msg import *

import numpy

class MoveInLine:
    def __init__(self, name):
        self.target = None

        self.sub = rospy.Subscriber("/robot_pose", Pose2D, self.go_to_cb)
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

        self._as = SimpleActionServer(name, GoToPoseAction, auto_start=False)
        self._as.register_goal_callback(self.goal_cb)
        self._as.register_preempt_callback(self.preempt_cb)
        self._as.start()

        rospy.loginfo("Creating ActionServer [%s]", name)

    def goal_cb(self):

        self.target = self._as.accept_new_goal().pose2d

    def preempt_cb(self):
        self._as.set_preempted()

    def go_to_cb(self, msg):
        if not self._as.is_active():
            return

        np_target = numpy.array([self.target.x, self.target.y])
        np_robot = numpy.array([msg.x, msg.y])

        if numpy.linalg.norm(np_target - np_robot) < 0.0001 and abs(self.target.theta - msg.theta) < 0.01:
            self.pub.publish(Twist(Vector3(0, 0, 0), Vector3(0, 0, 0)))
            self._as.set_succeeded()
            return

        np_vector = np_target - np_robot
        np_norm = np_vector/numpy.linalg.norm(np_vector)
        np_vel = np_norm * 0.3

        ang_dis = numpy.unwrap(numpy.array([self.target.theta - msg.theta]))

        self.pub.publish(Twist(Vector3(np_vel[0], np_vel[1], 0), Vector3(0, 0, ang_dis[0])))

def main():
    rospy.init_node("move_in_line")
    MoveInLine(rospy.get_name())
    rospy.spin()

if __name__ == "__main__":
    main()
