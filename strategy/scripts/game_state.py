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
    rospy.init_node("game_state")

    def state_cb(outcom_map):
        if outcome_map['IS_KICK_OFF'] == 'true':
            return 'kick_off'
        if outcome_map['IS_FREE_BALL'] == 'true':
            return 'free_ball'
        if outcome_map['IS_FREE_KICK'] == 'true':
            return 'free_kick'
        return 'no_state'

    state_con = Concurrence(outcoms = ['kick_off', 'free_kick', 'free_ball'],
            default_outcome = 'no_state',
            child_termination_cb = lambda state_outcomes: True,
            out_cb = role_cb,
            input_keys = ['game_state'])

    with state_con:
        Concurrence.add('IS_KICK_OFF',
                ConditionState(cond_cb = lambda ud: ud.game_state == 'KICK_OFF'))
        Concurrence.add('IS_FREE_KICK',
                ConditionState(cond_cb = lambda ud: ud.game_state == 'FREE_KICK'))
        Concurrence.add('IS_FREE_BALL',
                ConditionState(cond_cb = lambda ud: ud.game_state == 'FREE_BALL'))



    # Construct state machine
    game_state_sm = StateMachine(
            outcomes=['successed','aborted','preempted'],
            input_keys = ['game_state'])

    game_state_sm.set_initial_state(['STAND_BY'])

    with game_state_sm:
        StateMachine.add('GAME_STATE', state_con,
                transistions = {'goal_keeper': 'GOAL_KEEPER',
                    'offensive': 'OFFENSIVE',
                    'deffensive': 'DEFFENSIVE'})
        StateMachine.add('KICK_OFF',
                SimpleActionState('kick_off', GoalKeeperAction)}
        StateMachine.add('FREE_KICK',
                SimpleActionState('free_kick', FreeKickAction)}
        StateMachine.add('FREE_BALL',
                SimpleActionState('free_ball', FreeBallAction)}


    # Run state machine introspection server
    intro_server = IntrospectionServer('game_state',game_state_sm,'/GAME_STATE')
    intro_server.start()

    # Run state machine action server
    sms = ActionServerWrapper(
            'game_state', GameStateAction, game_state_sm,
            succeeded_outcomes = ['goal'],
            aborted_outcomes = ['aborted'],
            preempted_outcomes = ['preempted'],
            goal_slots_map = {'game_state':'game_state'},
            result_slots_map = {})
    sms.run_server()

    rospy.spin()

    intro_server.stop()

if __name__ == "__main__":
    main()
