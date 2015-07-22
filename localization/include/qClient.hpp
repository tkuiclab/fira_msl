/*
 *   qClient.hpp
 *
 *   Date:2015
 *
 *   Author: Chien-Ming Lin
 *
 */
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef Client_HPP_
#define Client_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#endif

//#include "std_msgs/Header.h"          //for sks project
//#include "sensor_msgs/LaserScan.h"    //for sks project
#include <qt4/QtCore/QThread>
#include <string>
#include <std_msgs/String.h>
#include "geometry_msgs/Twist.h"
#include "imu_3d/inertia.h"
#include <QFuture>
#include <qtconcurrentrun.h>

#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Int32MultiArray.h"
#include "particle_filter.hpp"

#define sensorlineNum 60

/*****************************************************************************
** Class
*****************************************************************************/

class Client : public QThread {

private:

    /****************************/
    /************Var*************/
    /****************************/
    double shift_x;
    double shift_y;
    double rotation;
//    int *laser_dist;                          //for sks project
    int *white_lineDist;
    geometry_msgs::Twist estimate_pos;

    /****************************/
    /************ROS*************/
    /****************************/
    ros::NodeHandle *n;
    ros::Subscriber laser_sub;

public:
    Client(int argc, char** argv,const char* node_name);
    virtual ~Client() {}
    //void run();
    void ros_comms_init();

//    void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg);          //for sks project
    void moveCallback(const geometry_msgs::Twist::ConstPtr& msg);
    void imuCallback(const imu_3d::inertia::ConstPtr& msg);
    void whitelineCallback(const std_msgs::Int32MultiArray::ConstPtr& array);

    void pub_estimate_pos(geometry_msgs::Twist);
    //bool reset_MCL(localization_ui::Envpoint::Request &req,localization_ui::Envpoint::Response &res);

    ros::Subscriber move_sub;
    ros::Subscriber whiteline_sub;
    ros::Publisher pos_pub;
    ros::ServiceServer reset_MCL_service;

//    int *return_laser_dist();                 //for sks project
    int *return_whiteline_dist();
    geometry_msgs::Twist return_move();

};

#endif /* QClient_NODE_HPP_ */
