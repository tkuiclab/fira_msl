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
//#include "localization_ui/TwoInts.h"
#include "geometry_msgs/Twist.h"
#endif
#define pi 3.141593;
#include <string>
#include <std_msgs/String.h>

/*****************************************************************************
** Class
*****************************************************************************/

class QAdd : public QNode {

public:
	QAdd(int argc, char** argv);
	virtual ~QAdd() {}
    void run();
	void ros_comms_init();
    void setpoint();
    double rot_x,rot_y;
private:
//	bool add(qt_tutorials::TwoInts::Request  &req, qt_tutorials::TwoInts::Response &res);
	ros::ServiceServer add_server;
    ros::Publisher point_pub;
    QNode *qnode;

};

#endif /* QADD_NODE_HPP_ */
