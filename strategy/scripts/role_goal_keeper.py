#!/usr/bin/env python
#import roslib
#roslib.load_manifest('pr2_plugs_actions')

import rospy

import actionlib

from actionlib_msgs.msg import *
from geometry_msgs.msg import *

from std_srvs.srv import *

# State machine classes
import smach
from smach import *
from smach_ros import *


def main():
    rospy.init_node("role_goal_keeper")

    # Construct state machine
    goal_keeper_sm = StateMachine(
            outcomes=['goal','aborted','preempted'],
            input_keys = ['input'],
            output_keys = ['output'])

    # Set the initial state explicitly
    goal_keeper_sm.set_initial_state(['SEARCH_BALL'])

    with goal_keeper_sm:
        StateMachine.add('SEARCH_BALL',
                SimpleActionState('search_ball', SearchBallAction,
                    goal),
                { 'succeeded':'CHASE_BALL' })

        StateMachine.add('CHASE_BALL',
                SimpleActionState('chase_ball', ChaseBallAction),
                { 'succeeded':'' })



    # Run state machine introspection server
    intro_server = IntrospectionServer('goal_keeper',goal_keeper_sm,'/GOAL_KEEPER')
    intro_server.start()

    # Run state machine action server
    sms = ActionServerWrapper(
            'goal_keeper', OffensiveAction, goal_keeper_sm,
            succeeded_outcomes = ['goal'],
            aborted_outcomes = ['aborted'],
            preempted_outcomes = ['preempted'],
            goal_slots_map = {},
            result_slots_map = {})
    sms.run_server()

    rospy.spin()

    intro_server.stop()

if __name__ == "__main__":
    main()
