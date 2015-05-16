#include "roles/role_goalie.h"

GoalieServer::GoalieServer(std::string name, std::vector<string> par):
        RobotTask(name), params(par)
    {
		time = -1;
		if( find(params, "time=").size()>0 ){
			time = cast<int>(value(params,"time="));
		}
		
		retValue = 0;
		if( find(params, "return=").size()>0 ){
			retValue = cast<int>(value(params,"return="));
		}
		
		outputstr="process...";
		if( find(params, "print=").size()>0 ){
			outputstr = value(params,"print=");
		}
    }

    bool GoalieServer::exists(Arguments& args, std::string name){
		return args.find(name) != args.end();
	}
	
	std::string GoalieServer::find(std::vector<string>& s, std::string key){
		for(size_t i=0;i<s.size();i++){
			if(s[i].find(key)==0) return s[i];
		}
		return "";
	}
	std::string GoalieServer::value(std::vector<string>& s, std::string key){
		for(size_t i=0;i<s.size();i++){
			if(s[i].find(key)==0) return s[i].substr(s[i].find('=')+1);
		}
		return "";
	}

RobotTask::TaskResult GoalieServer::task(const string& name, const string& uid, Arguments& args) {
		int time = this->time;
		
		while(time != 0) {
			time -- ;
			
			if(isPreempt()){
				
				return TaskResult::PREEMPTED();
			}
			
			if(outputstr!="no_print"){
				ROS_INFO("%s: %s", _name.c_str(), outputstr.c_str());
			}
			sleep(1);
		}
                
        
        //return TaskResult::Preempted();
        if( retValue > 0 ){
			return TaskResult(retValue, "ERROR");
		}
        return TaskResult(SUCCESS, "OK");
    }

