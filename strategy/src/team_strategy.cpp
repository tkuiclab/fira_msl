#include <ros/ros.h>
#include <robot_task/RobotTask.h>
#include <robot_task/RobotTaskAction.h>

#include "fsm/fsm_team_strategy.h"

using namespace std;
using namespace robot_task;


void eventTest(const std_msgs::String::ConstPtr& msg, decision_making::EventQueue* q) {
   q->raiseEvent(msg->data.c_str());
}

decision_making::TaskResult standBy(std::string, const decision_making::FSMCallContext& c, decision_making::EventQueue& e) {
    ROS_INFO("Stand By");
    e.raiseEvent("/KICK_OFF");
    return decision_making::TaskResult::SUCCESS();
}

decision_making::TaskResult kickOff(std::string, const decision_making::FSMCallContext& c, decision_making::EventQueue& e) {
    ROS_INFO("Kick Off");
    e.raiseEvent("/GOAL_KEEPER");
    return decision_making::TaskResult::SUCCESS();
}

void run_fsm(RosEventQueue *req) {
    FsmTeamStrategy(NULL, req);
}

int main (int argc, char **argv)
{
    ros::init(argc, argv, "team_strategy");
    ros::NodeHandle nh;

    RosEventQueue* req = new RosEventQueue;
    ros::Subscriber sub = nh.subscribe<void>("/event_test", 1000,
            boost::function<void(const std_msgs::String::ConstPtr)>(boost::bind(eventTest, _1, req))
            );
    ros_decision_making_init(argc, argv);

    LocalTasks::registrate("StandBy", standBy);
    LocalTasks::registrate("KickOff", kickOff);

    boost::thread_group threads;
	threads.add_thread(new boost::thread(boost::bind(&run_fsm, req)));

    ros::spin();

    return 0;
}
