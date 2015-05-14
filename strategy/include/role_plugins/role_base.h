#ifndef _ROLE_BASE_H_
#define _ROLE_BASE_H_
#include "ros/ros.h"

namespace role_base
{
class Role
{
public:
    virtual void initialize(ros::NodeHandle&, std::string&) = 0;
    virtual void run() = 0;
    virtual ~Role() {}

protected:
    Role() {}
};
};
#endif
