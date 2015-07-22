/**
 * @file /qadd_server/qadd.hpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef QADD_NODE_HPP_
#define QADD_NODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include "../common/qnode.hpp"
#include "localization_ui/AddTwoInts.h"
#include "geometry_msgs/Twist.h"
#include <QMouseEvent>
#include <QEvent>
#include <QDebug>
#include <QLabel>
#include <QApplication>
#include <QPainter>
#include <QHBoxLayout>
#include <QPixmap>
#endif

#include <string>
#include <std_msgs/String.h>
#include <cstdlib>


/*****************************************************************************
** Class
*****************************************************************************/

class QAdd : public QNode {


public:
	QAdd(int argc, char** argv);
	virtual ~QAdd() {}
	void run();
    void ros_comms_init();


    void sendstd();
    void chatterCallback(const geometry_msgs::Twist::ConstPtr& vector);
    void robot_pos();
    void call();

//    void mouseMoveEvent(QMouseEvent *ev);
//    void MousePressEvent(QMouseEvent *ev);
//    void MouseleaveEvent(QEvent *);
//    int x,y;

//Q_SIGNALS:
//    void Mouse_Pressed();
//    void Mouse_Pos();
//    void Mouse_Left();
    double all_x,all_y,all_head;

private:
    ros::ServiceClient add_cli;
    ros::NodeHandle *n;
    ros::Subscriber chatter;
    ros::Publisher chatter_pub;
    geometry_msgs::Twist vec ;


};

#endif /* QADD_NODE_HPP_ */

