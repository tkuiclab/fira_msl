/**
 * @file common/main_window.hpp
 *
 * @brief Qt based gui for eros_qtalker.
 *
 * @date November 2010
 **/
#ifndef QTUTORIALS_MAIN_WINDOW_H
#define QTUTORIALS_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui/QMainWindow>


#ifndef Q_MOC_RUN

#endif
#include "common/ui_qclinet_window.h"
#include "qadd.hpp"
#include "geometry_msgs/Twist.h"
#include "localization_ui/Envpoint.h"
/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class ClientWindow : public QMainWindow {
Q_OBJECT

public:
    ClientWindow(QAdd *node, QWidget *parent = 0);
    ~ClientWindow();

	void ReadSettings(); // Load up qt program settings at startup
	void WriteSettings(); // Save qt program settings when closing

	void closeEvent(QCloseEvent *event); // Overloaded function
	void showNoMasterMessage();
    class Point{
    public:

        Point setPoint(int _x, int _y){
            this->x = _x;
            this->y = _y;
        }

        int x;
        int y;
    };

    //void keyPressEvent(QKeyEvent *);
//    void keyPressEvent(QKeyEvent *);
//    void mouseMoveEvent(QMouseEvent *);
//    void MousePressEvent(QMouseEvent *);
    //void mousePressEvent( QMouseEvent * event );

public Q_SLOTS:
	// Put automatically triggered slots here (because of connectSlotsByName())
	// void on_button_enable_clicked(bool check); // example only
	void on_actionAbout_triggered();
	void on_button_connect_clicked(bool check );
	void on_checkbox_use_environment_stateChanged(int state);

    void mousePressEvent( QMouseEvent* ev );

    /******************************************
    ** Manual connections
    *******************************************/
    void updateLoggingView(); // no idea why this can't connect automatically

//private slots:
//    void Mouse_Current_pos();
//    void Mouse_pressed();
//    void Mouse_left();
//signals:
//    void mousePressed( const QPoint& );

    void drawLabel();
    void on_Point_1_clicked();

    void on_Point_2_clicked();
    void on_Point_3_clicked();

    void on_Point_4_clicked();

    void on_Point_5_clicked();

    void on_Point_6_clicked();

    void on_Point_7_clicked();

    void on_Point_8_clicked();

    void on_Point_9_clicked();

    void on_Point_10_clicked();
    void on_sendpoint_clicked();
    void pathplan_strategy();
    void on_robot_pos_clicked();
    void on_LP1_clicked();
    void on_LP2_clicked();
    void on_LP3_clicked();
    void on_LP4_clicked();
    void on_LP5_clicked();
    void on_LP6_clicked();
    void on_LP7_clicked();
    void on_LP8_clicked();
    void on_LP9_clicked();
    void on_LP10_clicked();
    void on_LP11_clicked();
    void on_LP12_clicked();
    void on_LP13_clicked();
    void on_LP14_clicked();
    void on_LP15_clicked();
    void on_LP16_clicked();
    void on_LP17_clicked();
    void on_LP18_clicked();
    void on_LP19_clicked();
    void on_LP20_clicked();
    void on_LP21_clicked();
    void on_LP22_clicked();
    void on_LP23_clicked();
    void on_reset_button_clicked();
    void locpoint();

 protected:
    void timerEvent(QTimerEvent *event);
    int m_nTimerId;

private:
    Ui::ClientWindowDesign ui;
    QNode *qnode;
    QAdd *mClient;
    ros::Publisher vector_pub;
    ros::ServiceClient client;
    int loc_stream[20];
    int loc_x,loc_y;
    int area_num;
    int number;
    double forward_x,forward_y,forward_deg;
    int LPx,LPy;
    int LP[20];
    int No[10];

};

#endif // QTUTORIALS_MAIN_WINDOW_H
