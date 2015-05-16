#ifndef PF_NODE_HPP_
#define PF_NODE_HPP_

#include "../include/image_class.hpp"
#include "../include/particle_filter.hpp"
#include "../include/qClient.hpp"

class PFNode_Class : public Client
{
public:
    PFNode_Class(int argc, char** argv,const char *node_name);
    virtual ~PFNode_Class(){}

    void run();
private:
    Vector3d init_robot;
    Vector3d robot;

    bool ISmove;
    bool ISConvergence;
    int c;

    double Ismove;
    double Isrotate;
    geometry_msgs::Twist robot_shift;
    geometry_msgs::Twist robot_shift_tmp;
    geometry_msgs::Twist robot_shift_init;

    void pf_init();
    void pf_process();
};

#endif
