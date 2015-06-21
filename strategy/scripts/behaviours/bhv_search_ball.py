#!/usr/bin/env python
import rospy
import tf
import tf2_ros

from actionlib import *
from actionlib.msg import TestAction
from geometry_msgs.msg import *

class RefServer (object):

    def __init__(self, name):
        self.tf_buff = tf2_ros.Buffer()
        self.tf_lisener = tf2_ros.TransformListener(self.tf_buff)
        self._as = SimpleActionServer(name, TestAction, execute_cb=self.execute_cb, auto_start=False)
        self._as.start()
        rospy.loginfo("Creating ActionServer [%s]", name)

    def execute_cb(self, goal):
        rate = rospy.Rate(10)
        while self._as.is_active():
            if self._as.is_preempt_requested():
                self._as.set_preempted()
                break

            if self.tf_buff.can_transform('fira_msl1', 'ball_frame', rospy.Time.now()):
                self._as.set_succeeded()
                break

            rate.sleep()

def main():
    rospy.init_node("bhv_search_ball")
    RefServer(rospy.get_name())
    rospy.spin()

if __name__=='__main__':
    main()
