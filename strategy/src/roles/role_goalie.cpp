#include "roles/role_goalie.h"

GoalieServer::GoalieServer(const std::string& name):
    RobotTask(name)
{
}

RobotTask::TaskResult GoalieServer::task(const string& name, const string& uid, Arguments& args) {
    while(true) {

        if(isPreempt()) {

            return TaskResult::PREEMPTED();
        }

    }

    //if( retValue > 0 ){
    //	return TaskResult(retValue, "ERROR");
    //}
    return TaskResult(SUCCESS, "OK");
}

