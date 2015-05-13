#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <boost/thread.hpp>
#include <strategy/RoleAction.h>

void spinThread()
{
    ros::spin();
}

void wold

int main (int argc, char **argv)
{
    ros::init(argc, argv, "team_strategy");
    ros::NodeHandle nh = nh_.subscribe("/random_number", 1, );

    // create the action client
    actionlib::SimpleActionClient<std_msgs::String> ac("roles");
    boost::thread spin_thread(&spinThread);

    ROS_INFO("Waiting for action server to start.");
    ac.waitForServer();

    while(ros::ok()) {
        //ROS_INFO("Action server started, sending goal.");
        // send a goal to the action
        strategy::RoleGoal goal;
        goal.name = "Goalie";
        ac.sendGoal(goal);

        //wait for the action to return
        //bool finished_before_timeout = ac_offensive.waitForResult(ros::Duration(30.0));

        if (finished_before_timeout)
        {
            actionlib::SimpleClientGoalState state = ac.getState();
            ROS_INFO("Action finished: %s",state.toString().c_str());
        }
        else
            ROS_INFO("Action did not finish before the time out.");
    }

    // shutdown the node and join the thread back before exiting
    ros::shutdown();
    spin_thread.join();

    //exit
    return 0;
}
