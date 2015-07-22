#include "../include/particle_filter.hpp"
#include "../include/qClient.hpp"
#include "../include/image_class.hpp"
#include "ParticleFilter_node.hpp"


int main(int argc, char **argv)
{
    PFNode_Class p_node(argc,argv,"Particle_Filter_node");
    p_node.start();
    ros::spin();
    return 0;
}



//    Client mClient(argc,argv,"Particle_Filter_node");
//    // mImage(pNum,SensorLines,mapH,mapW)
//    paintImageClass mImage(300,45,600,600);
//ParticleFilter pf(300,45,600,600);

//    mImage.readMap();
//    mImage.paint_particle(pf.get_Particle());
//    //mImage.refresh_window();
//    mClient.ros_comms_init();
//    mClient.start();
//    while(ros::ok())
//    {
//        QFuture<geometry_msgs::Twist> updateInfoThread = QtConcurrent::run(&mClient,&Client::return_move);
//        geometry_msgs::Twist tmp;
//        tmp = updateInfoThread.result();
//        std::cout << tmp.linear.x << std::endl;
//        updateInfoThread.waitForFinished();
//    }

//    //mImage.readMap();
//    //mImage.paint_slot();

//    ros::spin();
//    return 0;
