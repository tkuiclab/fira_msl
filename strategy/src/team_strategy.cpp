#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <robot_task/RobotTask.h>
#include <robot_task/RobotTaskAction.h>

#include "fsm/fsm_team_strategy.h"

using namespace std;
using namespace robot_task;

int main (int argc, char **argv)
{
    ros::init(argc, argv, "team_strategy");
    ros::NodeHandle nh;
    //    = nh_.subscribe("/random_number", 1, );
    ros_decision_making_init(argc, argv);

    LocalTasks::registrate("goalie", callTask);

    //exit
    return 0;
}
