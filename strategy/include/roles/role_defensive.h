#ifndef _ROLE_PLUGINS_H_
#define _ROLE_PLUGINS_H_
#include <ros/ros.h>
#include <role_plugins/role_base.h>

namespace role_plugins
{
class Defensive : public role_base::Role
{
public:
    void initialize(ros::NodeHandle& nh, std::string& name) {}

    void run() {}

private:
};
};
#endif
