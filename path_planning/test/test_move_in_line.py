#!/usr/bin/env python

import rospy

from actionlib import *
from geometry_msgs.msg import *
from fira_msl_msgs.msg import *

def test_client():

    client = SimpleActionClient('move_in_line', GoToPoseAction)

    client.wait_for_server()

    goal = GoToPoseGoal(Pose2D(-1, 2, 1.5))

    client.send_goal(goal)

    client.wait_for_result()

    return

if __name__=='__main__':
    rospy.init_node('test_move_in_line')
    test_client()

