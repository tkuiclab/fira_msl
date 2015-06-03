#!/usr/bin/env python

import rospy

import actionlib

from actionlib.msg import *
from actionlib_msgs.msg import *
from geometry_msgs.msg import *

from std_srvs.srv import *

# State machine classes
import smach
from smach import *
from smach_ros import *


def main():
    rospy.init_node("role_offensive")

    # Construct state machine
    offensive_sm = StateMachine(
            outcomes=['succeeded','aborted','preempted'])

    # Set the initial state explicitly
    offensive_sm.set_initial_state(['SEARCH_BALL'])

    with offensive_sm:
        StateMachine.add('SEARCH_BALL',
                SimpleActionState('bhv_search_ball', TestAction),
                { 'succeeded':'CHASE_BALL' })

        StateMachine.add('CHASE_BALL',
                SimpleActionState('bhv_chase_ball', TestAction),
                { 'succeeded':'succeeded', 'aborted': 'SEARCH_BALL' })



    # Run state machine introspection server
    intro_server = IntrospectionServer('offensive',offensive_sm,'/ROLE_SELECTOR/OFFENSIVE')
    intro_server.start()

    # Run state machine action server
    sms = ActionServerWrapper(
            'role_offensive', TestAction, offensive_sm,
            succeeded_outcomes = ['succeeded'],
            aborted_outcomes = ['aborted'],
            preempted_outcomes = ['preempted'],
            goal_slots_map = {},
            result_slots_map = {})
    sms.run_server()

    rospy.spin()

    intro_server.stop()

if __name__ == "__main__":
    main()
