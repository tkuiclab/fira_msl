#!/usr/bin/env python
import rospy
from rospy.numpy_msg import *

from actionlib import *
from geometry_msgs.msg import *
from fira_msl_msgs.msg import *

import numpy

class MoveInLine:
    def __init__(self, name):
        self.pub = None
        self.sub = None
        self.target = None

        self._as = SimpleActionServer(name, GoToPoseAction, auto_start=False)
        self._as.register_goal_callback(self.goal_cb)
        self._as.register_preempt_callback(self.preempt_cb)
        self._as.start()

        rospy.loginfo("Creating ActionServer [%s]", name)

    def goal_cb(self):
        self.sub = rospy.Subscriber("/world_model/robot_pose", Pose2D, self.go_to_cb)
        self.pub = rospy.Publisher("/cmd_vel", Twist)

        self.target = accept_new_goal().pose2d

    def preempt_cb():
        self.pub.unregister()
        self.sub.unregister()
        self._as.set_preempted()

    def go_to_cb(self, msg):
        if target == msg:
            self.pub.unregister()
            self.sub.unregister()
            self._as.set_succeeded()
        np_target = numpy.array([self.target.x, self.target.y])
        np_robot = numpy.array([msg.x, msg.y])
        np_vel = np_target - np_robot
        ang_vel = msg.theta - target.theta

        self.pub.publish(Twist(Vector3(np_vel[0], np_vel[1], 0), Vector3(0, 0, ang_vel)))

def main():
    rospy.init_node("move_in_line")
    MoveInLine(rospy.get_name())
    rospy.spin()

if __name__ == "__main__":
    main()
