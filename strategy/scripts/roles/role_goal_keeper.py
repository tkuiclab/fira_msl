#!/usr/bin/env python
#import roslib
#roslib.load_manifest('pr2_plugs_actions')

import rospy

import actionlib

from actionlib.msg import *
from actionlib_msgs.msg import *
from geometry_msgs.msg import *

# State machine classes
import smach
from smach import *
from smach_ros import *


def main():
    rospy.init_node("role_goal_keeper")

    # Construct state machine
    goal_keeper_sm = StateMachine(
            outcomes=['succeeded','aborted','preempted'],
            input_keys = ['input'],
            output_keys = ['output'])

    # Set the initial state explicitly
    goal_keeper_sm.set_initial_state(['SEARCH_BALL'])

    with goal_keeper_sm:
        StateMachine.add('SEARCH_BALL',
                SimpleActionState('search_ball', TestAction),
                { 'succeeded':'CHASE_BALL' })

        StateMachine.add('CHASE_BALL',
                SimpleActionState('chase_ball', TestAction),
                { 'succeeded':'succeeded', 'aborted': 'SEARCH_BALL' })



    # Run state machine introspection server
    intro_server = IntrospectionServer('goal_keeper',goal_keeper_sm,'/ROLE_SELECTOR/GOAL_KEEPER')
    intro_server.start()

    # Run state machine action server
    sms = ActionServerWrapper(
            'goal_keeper', TestAction, goal_keeper_sm,
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
