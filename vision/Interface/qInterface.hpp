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

Q_SIGNALS:

private:
    ros::NodeHandle *nh;
    image_transport::ImageTransport *it_;
    image_transport::Subscriber image_sub_;

};

#endif /* QServer_NODE_HPP_ */
