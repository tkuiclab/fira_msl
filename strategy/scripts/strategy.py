#!/usr/bin/env python

import rospy

import actionlib
from std_msgs.msg import String

def world_model_cb(data):
    rospy.loginfo(data.data)

def game_state_cb(data):
    rospy.loginfo(data.data)


def main():
    rospy.init_node('strategy', anonymous = True)

    rospy.Subscriber('/world_model', String, world_model_cb)
    rospy.Subscriber('/referee_box/game_state', String, game_state_cb)

    rospy.spin()

if __name__ == '__main__':
    main()
