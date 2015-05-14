#include <ros/ros.h>
#include <pluginlib/class_loader.h>
#include <role_plugins/role_base.h>
#include <actionlib/server/simple_action_server.h>
#include <strategy/RoleAction.h>
#include <std_msgs/String.h>

class Roles
{
public:

    Roles(std::string name) :
        as_(nh_, name, false),
        action_name_(name),
        role_loader("strategy", "role_base::Role")
    {

        //register the goal and feeback callbacks
        as_.registerGoalCallback(boost::bind(&Roles::goalCB, this));
        as_.registerPreemptCallback(boost::bind(&Roles::preemptCB, this));

        //subscribe to the data topic of interest
        sub_ = nh_.subscribe("/random_number", 1, &Roles::executeCB, this);
        as_.start();
    }

    ~Roles(void)
    {
    }

    void goalCB()
    {
        // accept the new goal
        role_ = as_.acceptNewGoal()->name.c_str();
        try
        {
            current_role = role_loader.createInstance(role_);
            current_role->initialize(nh_, action_name_);

            //ROS_INFO("Triangle area: %.2f", current_role->area());
        }
        catch(pluginlib::PluginlibException& ex)
        {
            ROS_ERROR("The plugin failed to load for some reason. Error: %s", ex.what());
        }

    }

    void preemptCB()
    {
        ROS_INFO("%s: Preempted", action_name_.c_str());
        // set the action state to preempted
        as_.setPreempted();
    }


    void executeCB(const std_msgs::String::ConstPtr& msg)
    {
      // make sure that the action hasn't been canceled
      if (!as_.isActive())
        return;
    }

protected:

    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<strategy::RoleAction> as_;
    boost::shared_ptr<role_base::Role> current_role;
    std::string action_name_;
    std::string role_;
    ros::Subscriber sub_;
    pluginlib::ClassLoader<role_base::Role> role_loader;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "roles");

    Roles roles(ros::this_node::getName());
    ros::spin();

    return 0;
}
