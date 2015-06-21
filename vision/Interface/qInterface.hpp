/**
 * @file /QServer_server/QServer.hpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef QServer_NODE_HPP_
#define QServer_NODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include "../common/qnode.hpp"
#include <QObject>
#endif

#include <ros/ros.h>
#include "../common/qnode.hpp"
#include <string>
#include <std_msgs/String.h>
#include <sensor_msgs/image_encodings.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
/*****************************************************************************
** Class
*****************************************************************************/
namespace enc = sensor_msgs::image_encodings;

Q_DECLARE_METATYPE(std::string);

class QInterface : public QNode {
 Q_OBJECT


public:
    QInterface(int argc, char** argv);
    virtual ~QInterface() {}
    void run();
    void ros_comms_init();

    void imageCb(const sensor_msgs::ImageConstPtr& msg);
    cv_bridge::CvImagePtr cv_ptr;

    //-------------------center-------------------------
    void sent_center(int center_x, int center_y, int center_inner, int center_outer, int center_front){
        nh->setParam("/FIRA/Center/X",center_x);
        nh->setParam("/FIRA/Center/Y",center_y);
        nh->setParam("/FIRA/Center/Inner",center_inner);
        nh->setParam("/FIRA/Center/Outer",center_outer);
        nh->setParam("/FIRA/Center/Front",center_front);
    }
    int center_get_x, center_get_y, center_get_inner, center_get_outer, center_get_front;
    void get_center(){
        center_get_x = 0;
        nh->getParam("/FIRA/Center/X",center_get_x);
        nh->getParam("/FIRA/Center/Y",center_get_y);
        nh->getParam("/FIRA/Center/Inner",center_get_inner);
        nh->getParam("/FIRA/Center/Outer",center_get_outer);
        nh->getParam("/FIRA/Center/Front",center_get_front);
    }
    //--------------------------------------------------
    //---------------------scan-------------------------
    void sent_scan(std::vector<int>scan_para, std::vector<int>scan_near, std::vector<int>scan_middle, std::vector<int>scan_far){
        nh->setParam("/FIRA/Scan/Parameter",scan_para);
        nh->setParam("/FIRA/Scan/Near",scan_near);
        nh->setParam("/FIRA/Scan/Middle",scan_middle);
        nh->setParam("/FIRA/Scan/Far",scan_far);
    }
    std::vector<int>scan_get_para;
    void get_scan(){
        nh->getParam("/FIRA/Scan/Parameter",scan_get_para);
    }
    //--------------------------------------------------
    //--------------------distance----------------------
    void sent_dis(int dis_gap,std::vector<int>dis_space, std::vector<int>dis_pixel){
        nh->setParam("/FIRA/Distance/Gap",dis_gap);
        nh->setParam("/FIRA/Distance/Space",dis_space);
        nh->setParam("/FIRA/Distance/Pixel",dis_pixel);
    }
    //--------------------------------------------------
    //----------------------HSV-------------------------
    void sent_hsv(std::vector<int>HSV_red, std::vector<int>HSV_green, std::vector<int>HSV_blue, std::vector<int>HSV_yellow){
        nh->setParam("/FIRA/HSV/Redrange",HSV_red);
        nh->setParam("/FIRA/HSV/Greenrange",HSV_green);
        nh->setParam("/FIRA/HSV/Bluerange",HSV_blue);
        nh->setParam("/FIRA/HSV/Yellowrange",HSV_yellow);
    }
    std::vector<int>Redmap,Greenmap,Bluemap,Yellowmap;
    void get_hsv(){
        nh->getParam("/FIRA/HSV/Redrange",Redmap);
        nh->getParam("/FIRA/HSV/Greenrange",Greenmap);
        nh->getParam("/FIRA/HSV/Bluerange",Bluemap);
        nh->getParam("/FIRA/HSV/Yellowrange",Yellowmap);
    }
    //--------------------------------------------------

Q_SIGNALS:

private:
    ros::NodeHandle *nh;
    image_transport::ImageTransport *it_;
    image_transport::Subscriber image_sub_;

};

#endif /* QServer_NODE_HPP_ */
