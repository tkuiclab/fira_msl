#ifndef _ROLE_BASE_H_
#define _ROLE_BASE_H_

namespace role_base
{
class Role
{
public:
    virtual void initialize(double side_length) = 0;
    virtual double area() = 0;
    virtual ~Role() {}

protected:
    Role() {}
};
};
#endif
