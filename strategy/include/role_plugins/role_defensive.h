#ifndef _ROLE_PLUGINS_H_
#define _ROLE_PLUGINS_H_
#include <ros/ros.h>
#include <role_plugins/role_base.h>

namespace role_plugins
{
class Defensive : public role_base::Role
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
    double side_length_;
};
};
#endif
