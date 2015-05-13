#ifndef _ROLE_PLUGINS_H_
#define _ROLE_PLUGINS_H_
#include <ros/ros.h>
#include <role_plugins/role_base.h>

namespace role_plugins
{
class Offensive : public role_base::Role
{
public:
    void initialize(double side_length)
    {
    }

    double area()
    {
    }

    double getHeight()
    {
    }

private:
};
};
#endif
