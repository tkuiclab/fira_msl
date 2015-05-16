/*
 *   qClient.cpp
 *
 *   Date:2015
 *
 *   Author: Chien-Ming Lin
 *
 */

/*****************************************************************************
** Includes
*****************************************************************************/

#include <string>
#include <sstream>
#include "../include/qClient.hpp"
#include <std_msgs/String.h>

/*****************************************************************************
** ROS
*****************************************************************************/

Client::Client(int argc, char** argv,const char* node_name)
{
    //std::cout << "Initializing Node...\n";
    ros::init(argc,argv,node_name);
    //ROS_INFO("Connected to roscore");
}
void Client::ros_comms_init() {
    n = new ros::NodeHandle();
//    laser_dist = new int[sensorlineNum];          //for sks project
    white_lineDist = new int[sensorlineNum];

    //laser_sub = n->subscribe("/scan",1,&Client::laserCallback,this);              //for sks project
    //move_sub = n->subscribe("/encoder_topic",1,&Client::moveCallback,this);       //for sks project
    move_sub = n->subscribe("/fira_speed_topic",1,&Client::moveCallback,this);
    whiteline_sub = n->subscribe("/WhitlRealDis",1000,&Client::whitelineCallback,this);

    //move_sub = n->subscribe("/imu_3d",1,&Client::imuCallback,this);


    shift_x=0;
    shift_y=0;
    rotation =0;
    //ros::spin();
}

/*void Client::run() {
    ros::Rate loop_rate(1000);
    while(ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
}*/
void Client::imuCallback(const imu_3d::inertia::ConstPtr& msg)
{
        shift_x = msg->shift_x*100;
        shift_y = msg->shift_y*100;
        rotation = msg->yaw/360*6.28;
        ROS_INFO("go");
}
void Client::moveCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
        shift_x = msg->linear.x;
        shift_y = msg->linear.y;
        rotation = msg->angular.z;
        //ROS_INFO("go");
}
void Client::whitelineCallback(const std_msgs::Int32MultiArray::ConstPtr& array)
{
    int i = 0;
    for(std::vector<int>::const_iterator it = array->data.begin(); it != array->data.end(); ++it)
    {
        white_lineDist[i] = *it;
        i++;
    }
}

/*                          for sks project                              */
//void Client::laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
//{

//    for(int i=0;i<sensorlineNum;i++)
//    {
//        ROS_INFO("%d:[%0.2f]",i,msg->ranges[(i*4)]*100);
//        laser_dist[i] = msg->ranges[i*4]*100;
//        //laser_dist[sensorlineNum-i-1] = msg->ranges[i*4]*100;

//    }

////    ros::Rate loop_rate(1000);
////    ros::spinOnce();
////    loop_rate.sleep();
//}

/*****************************************************************************
** Return Value
*****************************************************************************/
geometry_msgs::Twist Client::return_move()
{
    geometry_msgs::Twist tmp;
//    tmp.linear.x = shift_x*100;
//    tmp.linear.y = -shift_y*100;
    tmp.linear.x = shift_x;
    tmp.linear.y = -shift_y;
    tmp.angular.z = rotation;
    //ROS_INFO("x:%f\ty:%f",tmp.linear.x,tmp.linear.y);
    return tmp;
}

int* Client::return_whiteline_dist()
{
    return white_lineDist;
}

/*          for sks project         */
//int* Client::return_laser_dist()
//{
//    return laser_dist;
//}
