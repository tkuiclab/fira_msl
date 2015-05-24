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
    rospy.init_node("role_offensive")

    # Construct state machine
    offensive_sm = StateMachine(
            outcomes=['goal','aborted','preempted'],
            input_keys = ['input'],
            output_keys = ['output'])

    # Set the initial state explicitly
    offensive_sm.set_initial_state(['SEARCH_BALL'])

    with offensive_sm:
        StateMachine.add('SEARCH_BALL',
                SimpleActionState('search_ball', SearchBallAction,
                    goal),
                { 'succeeded':'CHASE_BALL' })

        StateMachine.add('CHASE_BALL',
                SimpleActionState('chase_ball', ChaseBallAction),
                { 'succeeded':'' })



    # Run state machine introspection server
    intro_server = IntrospectionServer('offensive',offensive_sm,'/OFFENSIVE')
    intro_server.start()

    # Run state machine action server
    sms = ActionServerWrapper(
            'offensive', OffensiveAction, offensive_sm,
            succeeded_outcomes = ['goal'],
            aborted_outcomes = ['aborted'],
            preempted_outcomes = ['preempted'],
            goal_slots_map = {'command':'offensive_command'},
            result_slots_map = {'state':'offensive_state'})
    sms.run_server()

    rospy.spin()

    intro_server.stop()

if __name__ == "__main__":
    main()
