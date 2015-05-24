import rospy

from actionlib import *
from gemoetry_msgs.msg import *

class MoveInLine:
    def __init__(self):
        pass



def main():
    rospy.init_node("move_in_line")
    MoveInLine()
    rospy.spin()

if __name__ = "__main__":
    main()
