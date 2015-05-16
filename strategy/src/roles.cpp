#include <ros/ros.h>
#include "roles/role_goalie.h"

int main(int argc, char** argv){
	if(argc<0) return 1;
	std::string tname (argv[1]);
	
	if(tname=="-h" || tname=="--help"){
		cout<<"Dummy Robot Task for testing"<<endl;
		cout<<"   run empty robot task with any name"<<endl;
		cout<<"Syntax: dummy [TASK_NAME] [ARG1=VALUE] [ARG2=VALUE] ..."<<endl;
		cout<<"Args:"<<endl;
		cout<<"    time=INTEGER    : time (in millisec) for task running. if -1, wait forevere"<<endl;
		cout<<"    return=INTEGER  : return value of task: 0 is OK, -1 is PLAN, 0< is a ERROR CODE "<<endl;
		cout<<"    print=STRING    : print progress text. default is 'process...' and 'no_print' suppress printing."<<endl;
		return 0;
	}
	
	stringstream snodename; snodename<<"DummyTaskNode_"<<time(NULL);
	ROS_INFO("Start %s with Task %s", snodename.str().c_str(),argv[1]);
	
	ros::init(argc, argv, snodename.str().c_str());
	ros::NodeHandle node;
	
	ROS_INFO("craete task");
	

	std::vector<string> params;
	if(argc>2){
		for(size_t i=2; i<argc; i++){
			params.push_back(argv[i]);
		}
	}
	GoalieServer task(tname, params);

	ros::spin();

	return 0;
}
