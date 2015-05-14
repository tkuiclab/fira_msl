#include <pluginlib/class_list_macros.h>
#include <role_plugins/role_goalie.h>

PLUGINLIB_EXPORT_CLASS(role_plugins::Goalie, role_base::Role)

void role_plugins::Goalie::initialize(ros::NodeHandle& nh, std::string& name)
{
    nh_ = nh;
}

void role_plugins::Goalie::run()
{

}

