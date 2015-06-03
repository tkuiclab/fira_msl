#!/usr/bin/env python
import rospy

from actionlib import *
from actionlib.msg import TestAction
from geometry_msgs.msg import *

class RefServer (object):

    def __init__(self, name):
        self._as = SimpleActionServer(name, TestAction, self.execute_cb, False)
        self.sub = rospy.Subscriber("/world_model/ball_pose", Pose2D, self.found_ball_cb)
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self._as.start()
        rospy.loginfo("Creating ActionServer [%s]", name)

    def found_ball_cb(self, msg):
        if not self._as.is_active():
            return
        self._as.set_succeeded()
        return

    def execute_cb(self, goal):
        while not rospy.is_shutdown():
            if self._as.is_preempt_requested():
                self._as.set_preempted()
                break
            #self.pub.publish(Twist(Vector3(0.1, 0.1, 0), Vector3(0, 0, 0.1)))
            rospy.Rate(10)

def main():
    rospy.init_node("bhv_search_ball")
    RefServer(rospy.get_name())
    rospy.spin()

if __name__=='__main__':
    main()
