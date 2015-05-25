#!/usr/bin/env python
import rospy

from actionlib import *
from std_msgs.msg import String
from strategy.msg import *

class StrategyMain:
    def __init__(self):

        self.role = rospy.get_param('role', 'no_role')
        self.game_state = rospy.get_param('game_state', 'no_state')

        self.game_state_client = SimpleActionClient('game_state', GameStateAction)
        if self.game_state_client.wait_for_server(rospy.Duration(120.0)):
            rospy.loginfo("Recharge action web adapter connected to recharge action server.")
        else:
            rospy.logerr("Recharge action web adapter timed out while waiting for recharge action server.")
            raise Exception()

        self.role_selector_client = SimpleActionClient('role_selector', RoleAction)
        if self.role_selector_client.wait_for_server(rospy.Duration(120.0)):
            rospy.loginfo("Recharge action web adapter connected to recharge action server.")
        else:
            rospy.logerr("Recharge action web adapter timed out while waiting for recharge action server.")
            raise Exception()

        rospy.Subscriber('/world_model', String, self.world_model_cb)
        rospy.Subscriber('/referee_box/game_state', String, self.game_state_cb)
        rospy.Subscriber('/referee_box/role', String, self.role_cb)

    def role_cb(self, msg):
        self.role = msg.data
        self.role_selector_client.cancel_all_goals()
        self.role_selector_client.send_goal(
                RoleGoal(role = self.role))

    def world_model_cb(self, msg):
        rospy.loginfo(msg.data)

    def game_state_cb(self, msg):
        self.game_state = msg.data
        self.game_state_client.cancel_all_goals()
        self.role_selector_client.cancel_all_goals()

        self.game_state_client.send_goal(
                GameStateGoal(game_state = msg.data),
                done_cb = self.game_state_done_cb)

    def game_state_done_cb(self, result_state, result):
        self.role_selector_client.send_goal(
                RoleGoal(role = self.role))


def main():
    rospy.init_node('strategy', anonymous = True)
    StrategyMain()
    rospy.spin()

if __name__ == '__main__':
    main()
