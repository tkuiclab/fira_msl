#include <actionlib/server/simple_action_server.h>
#include <robot_task/RobotTask.h>
#include <robot_task/RobotTaskAction.h>
#include <robot_task/StringOperations.h>

using namespace std;
using namespace robot_task;

class GoalieServer : public RobotTask {
protected:

public:
    GoalieServer(const std::string&);

    TaskResult task(const string&, const string&, Arguments&);
};
