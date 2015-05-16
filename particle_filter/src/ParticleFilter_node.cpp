#include "ParticleFilter_node.hpp"

#define pNum 300
#define sensorlines 60
#define mapW 750
#define mapH 550
#define convergencefirst false
#define Simulate true


ParticleFilter pf(pNum,sensorlines,mapW,mapH);
paintImageClass image(pNum,sensorlines,mapW,mapH);

PFNode_Class::PFNode_Class(int argc, char **argv, const char *node_name):Client(argc,argv,node_name)
{
    this->pf_init();
}
void PFNode_Class::pf_init()
{
    this->ros_comms_init();
    pf.initParticle_Filter();
    pf.build_LikelihoodMap();
    image.readMap();
}

void PFNode_Class::pf_process()
{
    this->return_move();
}

void PFNode_Class::run()
{
    ISmove = false;
    ISConvergence = !convergencefirst;

    robot_shift_init.linear.x=0;
    robot_shift_init.linear.y=0;
    robot_shift_init.angular.z=0;
    robot_shift = robot_shift_init;
    robot_shift_tmp = robot_shift_init;

    init_robot(0) = 75;
    init_robot(1) = 275;
    init_robot(2) = 0;
    robot = init_robot;

    image.paintRobot_Particle(robot,pf.get_Particle());
    //image.paintRobot_Sensorlines_Particle(robot,pf.get_Particle(),pf.get_SensorWall());
    while(ros::ok())
    {
        robot_shift = this->return_move();

        Ismove = sqrt((robot_shift_tmp.linear.x-robot_shift.linear.x)*(robot_shift_tmp.linear.x-robot_shift.linear.x)
                    + (robot_shift_tmp.linear.y-robot_shift.linear.y)*(robot_shift_tmp.linear.x-robot_shift.linear.y));
        Isrotate = fabs(robot_shift_tmp.angular.z-robot_shift.angular.z);

        if(Simulate)
        {
            robot(2) = robot(2) + robot_shift.angular.z;
            robot(0) = robot(0) + robot_shift.linear.x*cos(robot(2)) + robot_shift.linear.y*cos(robot(2)+1.57);
            robot(1) = robot(1) + robot_shift.linear.x*sin(robot(2)) + robot_shift.linear.y*sin(robot(2)+1.57);
        }else{
            robot(0) = robot(0) + robot_shift.linear.x;
            robot(1) = robot(1) + robot_shift.linear.y;
            robot(2) = robot_shift.angular.z;
        }
        if(Ismove>=5||Isrotate>0.087)
        {
            pf.moveParticle(this->return_move());
            ISConvergence = false;
//            image.paintRobot_Particle(robot,pf.get_Particle());
            image.paintRobot_Sensorlines_Particle(robot,pf.get_Particle(),pf.get_SensorWall());
//            image.paintRobot_Particle_PSensor(robot,pf.get_Particle(),pf.get_SensorWall(),pf.get_tpwall());

        }
        if(!ISConvergence)
        {
            if(Simulate)
            {
                pf.Sim_SensorModel(robot);
                pf.rateGrade();
            }else{
                pf.rateGrade(robot,this->return_whiteline_dist());
            }

            //pf.roulette_wheel_selection();
            pf.tournament_selection();
            c++;
            if(c>25 || pf.ISConvergence())
            {
                ISConvergence = true;
                std::cout << "iteration:" << c << "\tIt's Convergence!" << std::endl;
                c=0;
            }
        }
//        image.paintRobot_Particle(robot,pf.get_Particle());
        image.paintRobot_Sensorlines_Particle(robot,pf.get_Particle(),pf.get_SensorWall());
//        image.paintRobot_Particle_PSensor(robot,pf.get_Particle(),pf.get_SensorWall(),pf.get_tpwall());

        robot_shift_tmp = robot_shift;
    }
}
