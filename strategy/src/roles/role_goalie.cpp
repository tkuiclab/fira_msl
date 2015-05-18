#include "roles/role_goalie.h"

GoalieServer::GoalieServer(const std::string& name):
    RobotTask(name)
{
}

RobotTask::TaskResult GoalieServer::task(const string& name, const string& uid, Arguments& args) {
    while(true) {

        if(isPreempt()) {
            ROS_INFO("Task STOP");
            return TaskResult::PREEMPTED();
        }

        ROS_INFO("Task RUN");
        sleep(1000);

    }

    //if( retValue > 0 ){
    //	return TaskResult(retValue, "ERROR");
    //}
    return TaskResult::SUCCESS();
}

