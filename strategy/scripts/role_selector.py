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
    rospy.init_node("role_selector")

    def role_selector_cb(outcom_map):
        if outcome_map['IS_GOAL_KEEPER'] == 'true':
            return 'goal_keeper'
        if outcome_map['IS_OFFENSIVE'] == 'true':
            return 'offensive'
        if outcome_map['IS_DEFENSIVE'] == 'true':
            return 'defensive'
        return 'no_role'

    role_selector_con = Concurrence(outcoms = ['goal_keeper', 'offensive', 'defensive'],
            default_outcome = 'offensive',
            child_termination_cb = lambda state_outcomes: True,
            out_cb = role_cb,
            input_keys = ['role'])

    with role_selector_con:
        Concurrence.add('IS_GOAL_KEEPER',
                ConditionState(cond_cb = lambda ud: ud.game_state == 'GOAL_KEEPER'))
        Concurrence.add('IS_OFFENSIVE',
                ConditionState(cond_cb = lambda ud: ud.game_state == 'OFFENSIVE'))
        Concurrence.add('IS_DEFENSIVE',
                ConditionState(cond_cb = lambda ud: ud.game_state == 'DEFENSIVE'))

    # Construct state machine
    role_sm = StateMachine(
            outcomes=['goal','aborted','preempted'],
            input_keys = ['role'])

    # Set the initial state explicitly
    offensive_sm.set_initial_state(['ROLE_SELECT'])

    with role_sm:
        StateMachine.add('ROLE_SELECT', role_selector_con,
                transistions = {'goal_keeper': 'GOAL_KEEPER',
                    'offensive': 'OFFENSIVE',
                    'deffensive': 'DEFFENSIVE'})
        StateMachine.add('GOAL_KEEPER',
                SimpleActionState('goal_keeper', GoalKeeperAction)}
        StateMachine.add('OFFENSIVE',
                SimpleActionState('goal_keeper', GoalKeeperAction)}
        StateMachine.add('DEFFENSIVE',
                SimpleActionState('goal_keeper', GoalKeeperAction)}


    # Run state machine introspection server
    intro_server = IntrospectionServer('role_selector',offensive_sm,'/ROLE_SELECTOR')
    intro_server.start()

    # Run state machine action server
    sms = ActionServerWrapper(
            'role_selector', RoleSelectorAction, role_sm,
            succeeded_outcomes = ['goal'],
            aborted_outcomes = ['aborted'],
            preempted_outcomes = ['preempted'],
            goal_slots_map = {'role':'role'},
            result_slots_map = {'state':'state'})
    sms.run_server()

    rospy.spin()

    intro_server.stop()

if __name__ == "__main__":
    main()
