import rospy

import sys

from actionlib.action_server import ActionServer
from actionlib.msg import TestAction,TestFeedback,TestResult

class RefServer (ActionServer):

    def __init__(self,name):
        action_spec=TestAction
        ActionServer.__init__(self,name,action_spec,self.goalCallback,self.cancelCallback, False);
        self.start()
        rospy.loginfo("Creating ActionServer [%s]\n", name);

        self.saved_goals=[]

    def goalCallback(self,gh):
        goal = gh.get_goal();

        rospy.loginfo("Got goal %d", int(goal.goal))
        if goal.goal == 1:
            gh.set_accepted();
            gh.set_succeeded(None, "The ref server has succeeded");
        elif goal.goal == 2:
            gh.set_accepted();
            gh.set_aborted(None, "The ref server has aborted");
        elif goal.goal == 3:
            gh.set_rejected(None, "The ref server has rejected");


        elif goal.goal == 4:

            self.saved_goals.append(gh);
            gh.set_accepted();

        elif goal.goal == 5:

            gh.set_accepted();
            for g in self.saved_goals:
                g.set_succeeded();
            self.saved_goals = [];
            gh.set_succeeded();


        elif goal.goal == 6:
            gh.set_accepted();
            for g in self.saved_goals:
                g.set_aborted();
            self.saved_goals = [];
            gh.set_succeeded();

        elif goal.goal == 7:
            gh.set_accepted();
            n=len(self.saved_goals);
            for i,g in enumerate(self.saved_goals):
                g.publish_feedback(TestFeedback(n-i));

            gh.set_succeeded();

        elif goal.goal == 8:
            gh.set_accepted();
            n=len(self.saved_goals);
            for i,g in enumerate(self.saved_goals):
                if i % 2 ==0:
                    g.set_succeeded(TestResult(n-i), "The ref server has succeeded");
                else:
                    g.set_aborted(TestResult(n-i), "The ref server has aborted")
            self.saved_goals=[];
            gh.set_succeeded();


        else:
            pass

    def cancelCallback(self,gh):
        pass

if __name__=="__main__":
  rospy.init_node("bhv_dash");
  ref_server = RefServer("dash");

  rospy.spin();
