#!/usr/bin/env python
import rospy

from actionlib import *
from actionlib.msg import TestAction
from geometry_msgs.msg import *
from fira_msl_msgs.msg import *

class RefServer (object):

    def __init__(self, name):
        self.sub = rospy.Subscriber("/world_model/ball_pose", Pose2D, self.found_ball_cb)

        self._ac = SimpleAction('move_in_line', GoToPoseAction)
        self._ac.wait_for_server()

        self._as = SimpleActionServer(name, TestAction, auto_start=False)
        self._as.register_goal_callback(self.goal_cb)
        self._as.register_preempt_callback(self.preempt_cb)
        self._as.start()
        rospy.loginfo("Creating ActionServer [%s]", name)

    def goal_cb(self):
        pass

    def preempt_cb(self):
        self._as.set_preempted()

    def found_ball_cb(self, msg):
        if not self._as.is_active():
            return

        self._ac.send_goal(GoToPoseGoal(msg))
        self._as.set_succeeded()
        return

def main():
    rospy.init_node("bhv_chase_ball")
    RefServer(rospy.get_name())
    rospy.spin()

if __name__=='__main__':
    main()
