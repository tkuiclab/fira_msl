#!/usr/bin/env python
import rospy

import actionlib

from actionlib_msgs.msg import *
from geometry_msgs.msg import *
from strategy.msg import *

# State machine classes
import smach
from smach import *
from smach_ros import *


def main():
    rospy.init_node("game_state")

    def state_cb(outcome_map):
        if outcome_map['IS_KICK_OFF'] == 'true':
            return 'kick_off'
        if outcome_map['IS_FREE_BALL'] == 'true':
            return 'free_ball'
        if outcome_map['IS_FREE_KICK'] == 'true':
            return 'free_kick'
        return 'no_state'

    state_con = Concurrence(outcomes = ['kick_off', 'free_kick', 'free_ball'],
            default_outcome = 'kick_off',
            child_termination_cb = lambda state_outcomes: True,
            outcome_cb = state_cb,
            input_keys = ['game_state'])

    with state_con:
        Concurrence.add('IS_KICK_OFF',
                ConditionState(cond_cb = lambda ud: ud.game_state == 'KICK_OFF',
                    input_keys = ['game_state']))
        Concurrence.add('IS_FREE_KICK',
                ConditionState(cond_cb = lambda ud: ud.game_state == 'FREE_KICK',
                    input_keys = ['game_state']))
        Concurrence.add('IS_FREE_BALL',
                ConditionState(cond_cb = lambda ud: ud.game_state == 'FREE_BALL',
                    input_keys = ['game_state']))



    # Construct state machine
    game_state_sm = StateMachine(
            outcomes=['goal', 'game_start', 'aborted','preempted'],
            input_keys = ['game_state'])

    game_state_sm.set_initial_state(['GAME_STATE'])

    with game_state_sm:
        StateMachine.add('GAME_STATE', state_con,
                transitions = {'kick_off': 'game_start',
                    'free_kick': 'FREE_KICK',
                    'free_ball': 'FREE_BALL'})
        StateMachine.add('FREE_KICK',
                SimpleActionState('free_kick', TestAction),
                {'succeeded': 'goal'})
        StateMachine.add('FREE_BALL',
                SimpleActionState('free_ball', TestAction),
                {'succeeded': 'goal'})


    # Run state machine introspection server
    intro_server = IntrospectionServer('game_state',game_state_sm,'/GAME_STATE')
    intro_server.start()

    # Run state machine action server
    sms = ActionServerWrapper(
            'game_state', GameStateAction, game_state_sm,
            succeeded_outcomes = ['goal', 'game_start'],
            aborted_outcomes = ['aborted'],
            preempted_outcomes = ['preempted'],
            goal_slots_map = {'game_state':'game_state'},
            result_slots_map = {})
    sms.run_server()

    rospy.spin()

    intro_server.stop()

if __name__ == "__main__":
    main()
