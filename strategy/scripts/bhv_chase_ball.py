#!/usr/bin/env python
import rospy

from actionlib import *
from actionlib.msg import TestAction
from geometry_msgs.msg import *
from fira_msl_msgs.msg import *

class RefServer (object):

    def __init__(self, name):
        self.past_ball = Pose2D()

        self._as = SimpleActionServer(name, TestAction, auto_start=False)
        self._as.register_goal_callback(self.goal_cb)
        self._as.register_preempt_callback(self.preempt_cb)
        self._as.start()

        self.sub_ball_pose = rospy.Subscriber("/world_model/ball_pose", Pose2D, self.found_ball_cb)

        self._ac = SimpleActionClient('move_in_line', GoToPoseAction)
        self._ac.wait_for_server()

        rospy.loginfo("Creating ActionServer [%s]", name)

    def goal_cb(self):
        self._as.accept_new_goal()
        self._ac.send_goal(GoToPoseGoal(Pose2D(0, 0, 0), 0.5, True), done_cb=self.done_cb)

    def preempt_cb(self):
        self._as.set_preempted()

    def found_ball_cb(self, msg):
        if not self._as.is_active():
            return

        #if msg.x != self.past_ball.x or msg.y != self.past_ball.y:
        #    self.past_ball = msg
        #    self._ac.send_goal(GoToPoseGoal(msg, 0.5, True), done_cb=self.done_cb)

    def done_cb(self, state, result):
        self._as.set_succeeded()

def main():
    rospy.init_node("bhv_chase_ball")
    RefServer(rospy.get_name())
    rospy.spin()

if __name__=='__main__':
    main()
