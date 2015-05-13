#include <ros/ros.h>
#include <pluginlib/class_loader.h>
#include <behavior_plugins/behavior_base.h>

int main(int argc, char** argv)
{
    pluginlib::ClassLoader<behavior_base::behavior> behavior_loader("strategy", "behavior_base::Behavior");

    try
    {
        boost::shared_ptr<behavior_base::behavior> goalie = behavior_loader.createInstance("behavior_plugins::Goalie");
        goalie->initialize(10.0);

        ROS_INFO("Triangle area: %.2f", goalie->area());
    }
    catch(pluginlib::PluginlibException& ex)
    {
        ROS_ERROR("The plugin failed to load for some reason. Error: %s", ex.what());
    }

    return 0;
}
