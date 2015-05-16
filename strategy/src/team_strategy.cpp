#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <robot_task/RobotTask.h>
#include <robot_task/RobotTaskAction.h>
using namespace std;
using namespace robot_task;

typedef actionlib::SimpleActionClient<RobotTaskAction> Client;

int main (int argc, char **argv)
{
    ros::init(argc, argv, "team_strategy");
    ros::NodeHandle nh;
    //    = nh_.subscribe("/random_number", 1, );

    // create the action client
    actionlib::SimpleActionClient<RobotTaskAction> ac("roles");

    ROS_INFO("Waiting for action server to start.");

	bool serverFound = false;
	while( !serverFound && ros::ok() ){
		ac.waitForServer(ros::Duration(1.0));
		if(ac.isServerConnected()){
			ROS_INFO("... Task is found.");
			serverFound = true;
		}else{
			ROS_INFO("... Task not found, retry...");
		}
	}

    while(ros::ok()) {

	// send a goal to the action
	RobotTaskGoal goal;
		goal.name = "goalie";
		stringstream suid; suid<<"task_id_tested_"<<::time(NULL);
		goal.uid = suid.str();
		goal.parameters="";
		//if(find(params,"arg=").size()!=0)
		//	goal.parameters = value(params, "arg=");
		//else if(find(params,"args=").size()!=0)
		//	goal.parameters = value(params, "args=");
        ac.sendGoal(goal);

        //wait for the action to return
        bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

        if (finished_before_timeout)
        {
            actionlib::SimpleClientGoalState state = ac.getState();
            ROS_INFO("Action finished: %s",state.toString().c_str());
        }
        else
            ROS_INFO("Action did not finish before the time out.");
    }

    //exit
    return 0;
}
