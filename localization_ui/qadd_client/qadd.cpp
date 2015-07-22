/**
 * @file /qadd_server/qadd.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <string>
#include <sstream>
#include "qadd.hpp"
#include <std_msgs/String.h>
#include <QtGui>
#include <iostream>

/*****************************************************************************
** Implementation
*****************************************************************************/

QAdd::QAdd(int argc, char** argv ) :
	QNode(argc,argv,"qadd_client")
	{}

void QAdd::ros_comms_init() {
//    ros::NodeHandle n;
//    add_cli = n.serviceClient<qt_tutorials::TwoInts>("add_two_ints");

    n = new ros::NodeHandle();
    add_cli = n->serviceClient<localization_ui::AddTwoInts>("add_two_ints_service");

    chatter = n->subscribe<geometry_msgs::Twist>("estimate_pos",1000,&QAdd::chatterCallback,this);
    std::cout<<"qclient init \n";
}

void QAdd::sendstd(){
    int last_sum = 0;
    int count = 1;
//    std::cout<<"hi \n";
//    std::cout<<"ros is OK \n";
    localization_ui::AddTwoInts srv;
    srv.request.a = 2;
    srv.request.b = 1;
//    std::cout<<"client.call \n";
    if (add_cli.call(srv))
    {
        ROS_INFO("Sum: %ld", (long int)srv.response.sum);
    }
    else
    {
      ROS_ERROR("Failed to call service add_two_ints");
    }


}
void QAdd::chatterCallback(const geometry_msgs::Twist::ConstPtr& vector){
     vec = *vector;
     ROS_INFO("linear_robot(x,y,head)=(%f,%f,%f)", vector->linear.x,vector->linear.y,vector->angular.z);
    //geometry_msgs::Twist vector;
     all_x = vec.linear.x;
     all_y = vec.linear.y;
     all_head = vec.angular.z;
     std::cout<<"in chatterCallback\n";
     ros::spinOnce();
    // fprintf(stderr,"(x,y,head)=(%d,%d,%d)\n",vector->linear.x,vector->linear.y,vector->angular.z);
}

void QAdd::robot_pos(){
    //void chatterCallback(const geometry_msgs::Twist::ConstPtr& vector);
 //   geometry_msgs::Twist vec = *vector;
    //vector = *vector;
    all_x = vec.linear.x;
    all_y = vec.linear.y;
    all_head = vec.angular.z;
    //std::cout<<"in";
    fprintf(stderr,"(x,y,head)=(%f,%f,%f)\n",all_x,all_y,all_head);


    //fprintf(stderr,"(x,y,head)=(%d,%d,%d)\n",x,y,head);

}

//void QAdd::mouseMoveEvent(QMouseEvent *ev){
//    this->x = ev->x();
//    this->y = ev->y();
//    emit Mouse_Pos();
//}

//void QAdd::MousePressEvent(QMouseEvent *ev){
//    emit Mouse_Pressed();
//}





void QAdd::run() {
    //robot_pos();
    //ros::spin();
//    ros::Rate loop_rate(1);
//    sendstd();
//    int last_sum = 0;
//    int count = 1;
//    std::cout<<"hi";
//    while( ros::ok() ) {
//        qt_tutorials::TwoInts srv;
//        srv.request.a = count;
//        srv.request.b = last_sum;
//        std::stringstream logging_msg;
//        if ( add_cli.call(srv) ) {
//            ROS_INFO_STREAM("Sum: " << srv.response.sum);
//            last_sum = srv.response.sum;
//            logging_msg << "[ INFO] ["  << "]: " << srv.request.a << " + " << srv.request.b << " = " << srv.response.sum;
//            ++count;
//        } else {
//            ROS_INFO_STREAM("failed to contact the server");
//            logging_msg << "[ INFO] [" << "]: failed to contact the server";
//        }
//        logging.insertRows(0,1);
//        QVariant new_row(QString(logging_msg.str().c_str()));
//        logging.setData(logging.index(0),new_row);
//        ros::spinOnce();
//        loop_rate.sleep();
//    }
    ros::spin();
    std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
    Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
//	std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
//	Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}




