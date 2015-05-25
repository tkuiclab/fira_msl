#!/usr/bin/env python
#import roslib
#roslib.load_manifest('pr2_plugs_actions')

import rospy

import actionlib

from actionlib.msg import *
from actionlib_msgs.msg import *
from geometry_msgs.msg import *
from strategy.msg import *

from std_srvs.srv import *

# State machine classes
import smach
from smach import *
from smach_ros import *


def main():
    rospy.init_node("role_selector")


    # Construct state machine
    role_sm = StateMachine(
            outcomes=['goal','aborted','preempted'],
            input_keys = ['role'])

    # Set the initial state explicitly
    role_sm.set_initial_state(['ROLE_SELECT'])

    with role_sm:
        StateMachine.add('ROLE_SELECT',
                CBState(cb = lambda ud: ud.role,
                    input_keys = ['role'],
                    outcomes = ['goal_keeper', 'offensive', 'defensive', 'no_role']),
                transitions = {
                    'goal_keeper': 'GOAL_KEEPER',
                    'offensive': 'OFFENSIVE',
                    'defensive': 'DEFENSIVE',
                    'no_role': 'aborted'})
        StateMachine.add('GOAL_KEEPER',
                SimpleActionState('role_goal_keeper', TestAction),
                {'succeeded': 'goal'})
        StateMachine.add('OFFENSIVE',
                SimpleActionState('role_offensive', TestAction),
                {'succeeded': 'goal'})
        StateMachine.add('DEFENSIVE',
                SimpleActionState('role_defensive', TestAction),
                {'succeeded': 'goal'})


    # Run state machine introspection server
    intro_server = IntrospectionServer('role_selector',role_sm,'/ROLE_SELECTOR')
    intro_server.start()

    # Run state machine action server
    sms = ActionServerWrapper(
            'role_selector', RoleAction, role_sm,
            succeeded_outcomes = ['goal'],
            aborted_outcomes = ['aborted'],
            preempted_outcomes = ['preempted'],
            goal_slots_map = {'role':'role'},
            result_slots_map = {})
    sms.run_server()

    rospy.spin()

    intro_server.stop()

if __name__ == "__main__":
    main()
