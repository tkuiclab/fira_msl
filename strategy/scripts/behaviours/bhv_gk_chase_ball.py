#!/usr/bin/env python
import rospy
import tf
import tf2_ros
import tf2_geometry_msgs

from actionlib import *
from actionlib.msg import TestAction
from geometry_msgs.msg import *
from fira_msl_msgs.msg import *

class RefServer (object):

    def __init__(self, name):
        self._as = SimpleActionServer(name, TestAction, auto_start=False)
        self._as.register_goal_callback(self.goal_cb)
        self._as.register_preempt_callback(self.preempt_cb)
        self._as.start()

        self.tf_buff = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buff)

        self._ac = SimpleActionClient('move_in_line', MovingByPoseAction)
        self._ac.wait_for_server()

        rospy.logdebug("Creating ActionServer [%s]", name)

    def goal_cb(self):
        rospy.loginfo("get_goal")
        self._as.accept_new_goal()

        try:
            target = self.tf_buff.transform(
                    PointStamped(Header(None, rospy.Time(), 'ball_frame'), Point(0.0, 0.0, 0.0)),
                    'BlueGoal')
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            pass

        self._ac.send_goal(MovingByPoseGoal('BlueGoal', Pose2D(0, target.point.y, 0), 0.3), done_cb=self.done_cb)

    def preempt_cb(self):
        self._as.set_preempted()

    def found_ball_cb(self, msg):
        if not self._as.is_active():
            return

    def done_cb(self, state, result):
        self._as.set_succeeded()

def main():
    rospy.init_node("bhv_gk_chase_ball")
    RefServer(rospy.get_name())
    rospy.spin()

if __name__=='__main__':
    main()
