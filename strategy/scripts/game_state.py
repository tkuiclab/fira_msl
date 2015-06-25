#!/usr/bin/env python
import rospy

import actionlib

from actionlib.msg import *
from actionlib_msgs.msg import *
from geometry_msgs.msg import *
from strategy.msg import *

# State machine classes
import smach
from smach import *
from smach_ros import *


def main():
    rospy.init_node("game_state")

    # Construct state machine
    game_state_sm = StateMachine(
            outcomes=['succeeded', 'aborted','preempted'],
            input_keys = ['game_state'])

    game_state_sm.set_initial_state(['GAME_STATE'])

    with game_state_sm:
        StateMachine.add('GAME_STATE',
                CBState(cb = lambda ud: ud.game_state,
                    input_keys = ['game_state'],
                    outcomes = ['kick_off', 'free_kick', 'free_ball', 'penalty_kick', 'goalkeeper_kick', 'no_state']),
                transitions = {
                    'kick_off': 'aborted',
                    'free_kick': 'FREE_KICK',
                    'free_ball': 'FREE_BALL',
                    'penalty_kick': 'PENALTY_KICK',
                    'goalkeeper_kick': 'GOALKEEPER_KICK',
                    'no_state': 'preempted'})
        StateMachine.add('FREE_KICK',
                SimpleActionState('free_kick', TestAction),
                {'succeeded': 'succeeded', 'aborted': 'aborted'})
        StateMachine.add('FREE_BALL',
                SimpleActionState('free_ball', TestAction),
                {'succeeded': 'succeeded', 'aborted': 'aborted'})
        StateMachine.add('PENALTY_KICK',
                SimpleActionState('penalty_kick', TestAction),
                {'succeeded': 'succeeded', 'aborted': 'aborted'})
        StateMachine.add('GOALKEEPER_KICK',
                SimpleActionState('goalkeeper_kick', TestAction),
                {'succeeded': 'succeeded', 'aborted': 'aborted'})


    # Run state machine introspection server
    intro_server = IntrospectionServer('game_state',game_state_sm,'/GAME_STATE')
    intro_server.start()

    # Run state machine action server
    sms = ActionServerWrapper(
            'game_state', GameStateAction, game_state_sm,
            succeeded_outcomes = ['succeeded'],
            aborted_outcomes = ['aborted'],
            preempted_outcomes = ['preempted'],
            goal_slots_map = {'game_state':'game_state'},
            result_slots_map = {})
    sms.run_server()

    rospy.spin()

    intro_server.stop()

if __name__ == "__main__":
    main()
