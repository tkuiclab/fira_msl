#include <actionlib/server/simple_action_server.h>
#include <robot_task/RobotTask.h>
#include <robot_task/RobotTaskAction.h>
#include <robot_task/StringOperations.h>

using namespace std;
using namespace robot_task;

class GoalieServer : public RobotTask {
protected:

	std::vector<string> params;
	
	int time;
	int retValue;
	string outputstr;
	
public:
    GoalieServer(std::string name, std::vector<string> par);

    bool exists(Arguments& args, std::string name);	

	std::string find(std::vector<string>& s, std::string key);

        std::string value(std::vector<string>& s, std::string key);
            template<class A> A cast(std::string name){
		std::stringstream n;n<<name;
		A a; n>>a;
		return a;
	}	
 
	template<class A> A cast(Arguments& args, std::string name){
		std::stringstream n;n<<args[name];
		A a; n>>a;
		return a;
	}
    
    TaskResult task(const string& name, const string& uid, Arguments& args);
};
