/**
 * @file common/main_window.hpp
 *
 * @brief Qt based gui for eros_qtalker.
 *
 * @date November 2010
 **/
#ifndef QTUTORIALS_MAIN_WINDOW_H
#define QTOTORIALS_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui/QMainWindow>

#ifndef Q_MOC_RUN
#include "common/ui_qserver_window.h"
//#include "qnode.hpp"
#include "qadd.hpp"
#endif

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class ServerWindow : public QMainWindow {
Q_OBJECT

public:
    ServerWindow(QAdd *node, QWidget *parent = 0);
    ~ServerWindow();

	void ReadSettings(); // Load up qt program settings at startup
	void WriteSettings(); // Save qt program settings when closing

	void closeEvent(QCloseEvent *event); // Overloaded function
	void showNoMasterMessage();
//    double rot_x,rot_y;
public Q_SLOTS:
	// Put automatically triggered slots here (because of connectSlotsByName())
	// void on_button_enable_clicked(bool check); // example only
	void on_actionAbout_triggered();
	void on_button_connect_clicked(bool check );
	void on_checkbox_use_environment_stateChanged(int state);

    /******************************************
    ** Manual connections
    *******************************************/
    void updateLoggingView(); // no idea why this can't connect automatically



    void on_point1_clicked();
    void on_point2_clicked();

private:
    Ui::ServerWindowDesign ui;
    QNode *qnode;
    QAdd *mServer;
};

#endif // QTUTORIALS_MAIN_WINDOW_H
