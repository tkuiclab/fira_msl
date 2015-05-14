#ifndef _ROLE_PLUGINS_H_
#define _ROLE_PLUGINS_H_
#include <ros/ros.h>
#include <role_plugins/role_base.h>
#include <cmath>

namespace role_plugins
{
class Goalie : public role_base::Role
{
public:
    void initialize(ros::NodeHandle&, std::string&);

    void run();

private:
    ros::NodeHandle nh_;
};
};
#endif
