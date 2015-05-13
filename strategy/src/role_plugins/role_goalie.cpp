#include <pluginlib/class_list_macros.h>
#include <role_plugins/role_base.h>
#include <role_plugins/role_goalie.h>

PLUGINLIB_EXPORT_CLASS(role_plugins::Goalie, role_base::Role)

void role_plugins::Goalie::initialize(double side_length)
{
    side_length_ = side_length;
}

double role_plugins::Goalie::area()
{
    return 0.5 * side_length_ * getHeight();
}

double role_plugins::Goalie::getHeight()
{
    return sqrt((side_length_ * side_length_) - ((side_length_ / 2) * (side_length_ / 2)));
}

