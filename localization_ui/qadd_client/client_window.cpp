/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date November 2010
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtCore>
#include <QtGui>
#include <QMessageBox>
#include <QImage>
#include <QLabel>
#include <QKeyEvent>
#include <QtDebug>
#include <Qt>
#include <QPainter>
#include <iostream>
#define pi 3.141593;
#include "client_window.hpp"


using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

ClientWindow::ClientWindow(QAdd *node, QWidget *parent) :
    QMainWindow(parent),
    mClient(node)
{
    ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
    startTimer(100);
    QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt())); // qApp is a global variable for the application
	setWindowIcon(QIcon(":/images/icon.png"));
	ui.tab_manager->setCurrentIndex(0); // ensure the first tab is showing - qt-designer should have this already hardwired, but often loses it (settings?).

    setWindowTitle(QApplication::translate("MainWindowDesign", mClient->nodeName().c_str(), 0, QApplication::UnicodeUTF8));

    /*********************
    ** Logging
    **********************/
    QObject::connect(mClient, SIGNAL(loggingUpdated()), this, SLOT(updateLoggingView()));
    QObject::connect(mClient, SIGNAL(rosShutdown()), this, SLOT(close()));

    /*********************
    ** Auto Start
    **********************/
    if ( ui.checkbox_remember_settings->isChecked() ) {
        on_button_connect_clicked(true);
    }

    area_num=0;
    for(int i=0 ; i<20 ;i++) LP[i]=0;
    for(int i=0 ; i<10 ;i++) No[i]=0;

}

ClientWindow::~ClientWindow() {}

/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/

void ClientWindow::showNoMasterMessage() {
	QMessageBox msgBox;
	msgBox.setText("Couldn't find the ros master.");
	msgBox.exec();
    close();
}

/*
 * These triggers whenever the button is clicked, regardless of whether it
 * is already checked or not.
 */

void ClientWindow::on_button_connect_clicked(bool check ) {
	if ( ui.checkbox_use_environment->isChecked() ) {
        if ( !mClient->on_init() ) {
			showNoMasterMessage();
		} else {
			ui.button_connect->setEnabled(false);
		}
	} else {
        if ( ! mClient->on_init(
					ui.line_edit_master->text().toStdString(),
					ui.line_edit_host->text().toStdString() )
				) {
			showNoMasterMessage();
		} else {
			ui.button_connect->setEnabled(false);
			ui.line_edit_master->setReadOnly(true);
			ui.line_edit_host->setReadOnly(true);
		}
	}
}

void ClientWindow::on_checkbox_use_environment_stateChanged(int state) {
	bool enabled;
	if ( state == 0 ) {
		enabled = true;
	} else {
		enabled = false;
	}
	ui.line_edit_master->setEnabled(enabled);
	ui.line_edit_host->setEnabled(enabled);
}

/*****************************************************************************
** Implemenation [Slots][manually connected]
*****************************************************************************/

/**
 * This function is signalled by the underlying model. When the model changes,
 * this will drop the cursor down to the last line in the QListview to ensure
 * the user can always see the latest log message.
 */
void ClientWindow::updateLoggingView() {

}

/*****************************************************************************
** Implementation [Menu]
*****************************************************************************/

void ClientWindow::on_actionAbout_triggered() {
    QMessageBox::about(this, tr("About ..."),tr("<h2>PACKAGE_NAME Test Program 0.10</h2><p>Copyright Yujin Robot</p><p>This package needs an about description.</p>"));
}

/*****************************************************************************
** Implementation [Configuration]
*****************************************************************************/

void ClientWindow::ReadSettings() {
    QSettings settings("Qt-Ros Package", mClient->nodeName().c_str());
    restoreGeometry(settings.value("geometry").toByteArray());
    restoreState(settings.value("windowState").toByteArray());
    QString master_url = settings.value("master_url",QString("http://192.168.1.2:11311/")).toString();
    QString host_url = settings.value("host_url", QString("192.168.1.3")).toString();
    QString topic_name = settings.value("topic_name", QString("/chatter")).toString();
    ui.line_edit_master->setText(master_url);
    ui.line_edit_host->setText(host_url);
    bool remember = settings.value("remember_settings", false).toBool();
    ui.checkbox_remember_settings->setChecked(remember);
    bool checked = settings.value("use_environment_variables", false).toBool();
    ui.checkbox_use_environment->setChecked(checked);
    if ( checked ) {
    	ui.line_edit_master->setEnabled(false);
    	ui.line_edit_host->setEnabled(false);
    }
}

void ClientWindow::WriteSettings() {
    QSettings settings("Qt-Ros Package", mClient->nodeName().c_str());
    settings.setValue("geometry", geometry());
    settings.setValue("master_url",ui.line_edit_master->text());
    settings.setValue("host_url",ui.line_edit_host->text());
   	settings.setValue("use_environment_variables",QVariant(ui.checkbox_use_environment->isChecked()));
    settings.setValue("windowState", saveState());
    settings.setValue("remember_settings",QVariant(ui.checkbox_remember_settings->isChecked()));
}

void ClientWindow::closeEvent(QCloseEvent *event)
{
    mClient->shutdown();
	WriteSettings();
	QMainWindow::closeEvent(event);
}


//void ClientWindow::keyPressEvent(QKeyEvent *ke)
//{
//  qDebug()<< "nativeScanCode: " << ke->nativeScanCode() <<endl;
//  qDebug()<< "key: "<< ke->key() <<endl;

//}
//void ClientWindow::mouseMoveEvent(QMouseEvent *me)
//{
//  setMouseTracking(true);
//  if (me->buttons() == Qt::RightButton)     { qDebug() << "RightButtom"; }
//  if (me->buttons() == Qt::LeftButton) { qDebug() << "LeftButton";  }
//  else if (me->buttons() == Qt::MidButton)  { qDebug() << "MidButton";   }

//  qDebug() << "Mouse(x,y):" << me->x() << me->y();
//}
//void ClientWindow::MousePressEvent(QMouseEvent *me)
//{
//  setMouseTracking(true);

//  qDebug() << "Mouse(x,y):" << me->x() << me->y();
//}

void ClientWindow::mousePressEvent( QMouseEvent * event ){

    if(event->pos().x()-22<0 || event->pos().y()-71<0){
        std::cout<<"error";
    }else if(event->pos().x()-22>750 || event->pos().y()-71>550){
        std::cout<<"error";
    }else{

    fprintf(stderr,"(x,y)=(%d,%d)\n",event->pos().x()-22,event->pos().y()-71);
    ros::NodeHandle n;
    client = n.serviceClient<localization_ui::Envpoint>("env_point");
    localization_ui::Envpoint server;
    server.request.x = event->pos().x()-22;
    server.request.y = event->pos().y()-71;
    if (client.call(server))
     {
       ROS_INFO("Response: %ld", (long int)server.response.resp);
     }
    else
     {
       ROS_ERROR("Failed to call service Envpoint");

     }
    }

//    int x = event->pos().x();
//    int y = event->pos().y();


//    // Calculate scale and image sizes
//    int h = ui.lbIMouse->height();
//    int w = ui.lbIMouse->width();

//    if(x>w || y>h)
//    return;

//    x = x/m_scaleFactor;
//    y = y/m_scaleFactor;
}
//void ClientWindow::mousePressEvent( QMouseEvent* ev )
//{
//    const QPoint p = ev->pos();
//    emit mousePressed( p );
//}



void ClientWindow::drawLabel()
{
    QPixmap mypix("/home/iclab/FIRA_ws/src/localization_ui/localization_Ground.jpg");
    QSize(750,550);
    QImage image=mypix.toImage();
    mypix = mypix.fromImage(image);
    QPainter painter(&mypix);
//    double pi = 3.141593;
    double deg = 0-mClient->all_head;
    painter.setPen(QPen(Qt::red,6,SolidLine));
    painter.drawLine(mClient->all_x,mClient->all_y,mClient->all_x+(25*cos(deg)),mClient->all_y+(25*sin(deg)));


    painter.setPen(QPen(Qt::yellow,3,SolidLine));
    painter.drawRect(loc_x-10,loc_y-10,20,20);
    painter.setBrush( Qt::red );
    painter.drawEllipse(QPointF(mClient->all_x,mClient->all_y), 15, 15);
    ui.lbIMouse->setPixmap(mypix);

}

void ClientWindow::on_robot_pos_clicked()
{
    mClient->robot_pos();
    std::cout<<"abc";
}


void ClientWindow::on_Point_1_clicked()
{
   loc_x = ui.textEdit_px1->toPlainText().toDouble();
   loc_y = ui.textEdit_py1->toPlainText().toDouble();
   area_num = 0;
   drawLabel();

}
void ClientWindow::timerEvent(QTimerEvent *event)
{
   drawLabel();

}

void ClientWindow::on_Point_2_clicked()
{
    loc_x = ui.textEdit_px2->toPlainText().toDouble();
    loc_y = ui.textEdit_py2->toPlainText().toDouble();
    area_num = 1;
    drawLabel();

}

void ClientWindow::on_Point_3_clicked()
{
    loc_x = ui.textEdit_px3->toPlainText().toDouble();
    loc_y = ui.textEdit_py3->toPlainText().toDouble();
    area_num = 2;
    drawLabel();
}

void ClientWindow::on_Point_4_clicked()
{
    loc_x = ui.textEdit_px4->toPlainText().toDouble();
    loc_y = ui.textEdit_py4->toPlainText().toDouble();
    area_num = 3;
    drawLabel();
}

void ClientWindow::on_Point_5_clicked()
{
    loc_x = ui.textEdit_px5->toPlainText().toDouble();
    loc_y = ui.textEdit_py5->toPlainText().toDouble();
    area_num = 4;
    drawLabel();
}

void ClientWindow::on_Point_6_clicked()
{
    loc_x = ui.textEdit_px6->toPlainText().toDouble();
    loc_y = ui.textEdit_py6->toPlainText().toDouble();
    area_num = 5;
    drawLabel();
}

void ClientWindow::on_Point_7_clicked()
{
    loc_x = ui.textEdit_px7->toPlainText().toDouble();
    loc_y = ui.textEdit_py7->toPlainText().toDouble();
    area_num = 6;
    drawLabel();
}

void ClientWindow::on_Point_8_clicked()
{
    loc_x = ui.textEdit_px8->toPlainText().toDouble();
    loc_y = ui.textEdit_py8->toPlainText().toDouble();
    area_num = 7;
    drawLabel();
}

void ClientWindow::on_Point_9_clicked()
{
    loc_x = ui.textEdit_px9->toPlainText().toDouble();
    loc_y = ui.textEdit_py9->toPlainText().toDouble();
    area_num = 8;
    drawLabel();
}

void ClientWindow::on_Point_10_clicked()
{
    loc_x = ui.textEdit_px10->toPlainText().toDouble();
    loc_y = ui.textEdit_py10->toPlainText().toDouble();
    area_num = 9;
    drawLabel();
}

void ClientWindow::on_sendpoint_clicked()
{
    loc_stream[0] = ui.textEdit_px1->toPlainText().toDouble();
    loc_stream[1] = ui.textEdit_py1->toPlainText().toDouble();
    loc_stream[2] = ui.textEdit_px2->toPlainText().toDouble();
    loc_stream[3] = ui.textEdit_py2->toPlainText().toDouble();
    loc_stream[4] = ui.textEdit_px3->toPlainText().toDouble();
    loc_stream[5] = ui.textEdit_py3->toPlainText().toDouble();
    loc_stream[6] = ui.textEdit_px4->toPlainText().toDouble();
    loc_stream[7] = ui.textEdit_py4->toPlainText().toDouble();
    loc_stream[8] = ui.textEdit_px5->toPlainText().toDouble();
    loc_stream[9] = ui.textEdit_py5->toPlainText().toDouble();
    loc_stream[10] = ui.textEdit_px6->toPlainText().toDouble();
    loc_stream[11] = ui.textEdit_py6->toPlainText().toDouble();
    loc_stream[12] = ui.textEdit_px7->toPlainText().toDouble();
    loc_stream[13] = ui.textEdit_py7->toPlainText().toDouble();
    loc_stream[14] = ui.textEdit_px8->toPlainText().toDouble();
    loc_stream[15] = ui.textEdit_py8->toPlainText().toDouble();
    loc_stream[16] = ui.textEdit_px9->toPlainText().toDouble();
    loc_stream[17] = ui.textEdit_py9->toPlainText().toDouble();
    loc_stream[18] = ui.textEdit_px10->toPlainText().toDouble();
    loc_stream[19] = ui.textEdit_py10->toPlainText().toDouble();
    pathplan_strategy();

}
void ClientWindow::pathplan_strategy()
{
    ros::NodeHandle n;
    vector_pub = n.advertise<geometry_msgs::Twist>("robot_vector",1000);
    geometry_msgs::Twist vector;
    for(int i=0;i<10;i++)
    {
        loc_x = loc_stream[2*i+0];
        loc_y = loc_stream[2*i+1];
        forward_x = loc_x-mClient->all_x;
        forward_y = loc_y-mClient->all_y;
        forward_deg = atan2(forward_y,forward_x);
        double dist = sqrt(forward_x*forward_x+forward_y*forward_y);
        std::cout<<"ok";
        printf("%f/n",dist);
        int speed;
        if(dist<30)
            speed = 30;
        else if(dist<60&&dist>30)
            speed = 40;
        else
            speed = 65;
        vector.linear.x = cos(forward_deg)*speed;
        vector.linear.y = sin(forward_deg)*speed;
        vector_pub.publish(vector);

    }
    vector.linear.x = 0;
    vector.linear.y = 0;

}

void ClientWindow::on_LP1_clicked()
{
    LPx = 100;
    LPy = 100;
    number = 1;
     locpoint();

}

void ClientWindow::on_LP2_clicked()
{
    LPx = 298;
    LPy = 100;
    number = 2;

     locpoint();

}

void ClientWindow::on_LP3_clicked()
{
    LPx = 450;
    LPy = 100;
    number = 3;
     locpoint();

}

void ClientWindow::on_LP4_clicked()
{
    LPx = 650;
    LPy = 100;
    number = 4;
     locpoint();

}

void ClientWindow::on_LP5_clicked()
{
    LPx = 150;
    LPy = 175;
    number = 5;
    locpoint();

}

void ClientWindow::on_LP6_clicked()
{
    LPx = 247;
    LPy = 195;
    number = 6;
    locpoint();

}

void ClientWindow::on_LP7_clicked()
{
    LPx = 375;
    LPy = 175;
    number = 7;
    locpoint();

}

void ClientWindow::on_LP8_clicked()
{
    LPx = 500;
    LPy = 195;
    number = 8;
    locpoint();

}

void ClientWindow::on_LP9_clicked()
{
    LPx = 600;
    LPy = 175;
    number = 9;
    locpoint();

}

void ClientWindow::on_LP10_clicked()
{
    LPx = 115;
    LPy = 275;
    number = 10;
    locpoint();

}

void ClientWindow::on_LP11_clicked()
{
    LPx = 212;
    LPy = 275;
    number = 11;
    locpoint();

}

void ClientWindow::on_LP12_clicked()
{
    LPx = 375;
    LPy = 275;
    number = 12;
     locpoint();

}

void ClientWindow::on_LP13_clicked()
{
    LPx = 540;
    LPy = 275;
    number = 13;
     locpoint();

}

void ClientWindow::on_LP14_clicked()
{
    LPx = 635;
    LPy = 275;
    number = 14;
     locpoint();

}

void ClientWindow::on_LP15_clicked()
{
    LPx = 150;
    LPy = 375;
    number = 15;
     locpoint();

}

void ClientWindow::on_LP16_clicked()
{
    LPx = 247;
    LPy = 355;
    number = 16;
     locpoint();

}

void ClientWindow::on_LP17_clicked()
{
    LPx = 375;
    LPy = 375;
    number = 17;
     locpoint();

}

void ClientWindow::on_LP18_clicked()
{
    LPx = 500;
    LPy = 355;
    number = 18;
     locpoint();

}

void ClientWindow::on_LP19_clicked()
{
    LPx = 600;
    LPy = 375;
    number = 19;
     locpoint();

}

void ClientWindow::on_LP20_clicked()
{
    LPx = 100;
    LPy = 450;
    number = 20;
     locpoint();

}

void ClientWindow::on_LP21_clicked()
{
    LPx = 298;
    LPy = 450;
    number = 21;
     locpoint();

}

void ClientWindow::on_LP22_clicked()
{
    LPx = 450;
    LPy = 450;
    number = 22;
     locpoint();

}

void ClientWindow::on_LP23_clicked()
{
    LPx = 650;
    LPy = 450;
    number = 23;
     locpoint();

}

void ClientWindow::locpoint()
{

    if(area_num==0||area_num<10)
    {
        No[area_num] = number;
        LP[2*area_num] = LPx;
        LP[2*area_num+1] = LPy;
    }
    ui.No1->setPlainText(QString::number(No[0]));
    ui.textEdit_px1->setPlainText(QString::number(LP[0]));
    ui.textEdit_py1->setPlainText(QString::number(LP[1]));
    ui.No2->setPlainText(QString::number(No[1]));
    ui.textEdit_px2->setPlainText(QString::number(LP[2]));
    ui.textEdit_py2->setPlainText(QString::number(LP[3]));
    ui.No3->setPlainText(QString::number(No[2]));
    ui.textEdit_px3->setPlainText(QString::number(LP[4]));
    ui.textEdit_py3->setPlainText(QString::number(LP[5]));
    ui.No4->setPlainText(QString::number(No[3]));
    ui.textEdit_px4->setPlainText(QString::number(LP[6]));
    ui.textEdit_py4->setPlainText(QString::number(LP[7]));
    ui.No5->setPlainText(QString::number(No[4]));
    ui.textEdit_px5->setPlainText(QString::number(LP[8]));
    ui.textEdit_py5->setPlainText(QString::number(LP[9]));
    ui.No6->setPlainText(QString::number(No[5]));
    ui.textEdit_px6->setPlainText(QString::number(LP[10]));
    ui.textEdit_py6->setPlainText(QString::number(LP[11]));
    ui.No7->setPlainText(QString::number(No[6]));
    ui.textEdit_px7->setPlainText(QString::number(LP[12]));
    ui.textEdit_py7->setPlainText(QString::number(LP[13]));
    ui.No8->setPlainText(QString::number(No[7]));
    ui.textEdit_px8->setPlainText(QString::number(LP[14]));
    ui.textEdit_py8->setPlainText(QString::number(LP[15]));
    ui.No9->setPlainText(QString::number(No[8]));
    ui.textEdit_px9->setPlainText(QString::number(LP[16]));
    ui.textEdit_py9->setPlainText(QString::number(LP[17]));
    ui.No10->setPlainText(QString::number(No[9]));
    ui.textEdit_px10->setPlainText(QString::number(LP[18]));
    ui.textEdit_py10->setPlainText(QString::number(LP[19]));
    area_num++;
  if(area_num>=10){area_num=0;}
}

void ClientWindow::on_reset_button_clicked()
{
    mClient->sendstd();
    area_num = 0;
    ui.No1->setPlainText(QString::number(No[0]=0));
    ui.textEdit_px1->setPlainText(QString::number(LP[0]=0));
    ui.textEdit_py1->setPlainText(QString::number(LP[1]=0));
    ui.No2->setPlainText(QString::number(No[1]=0));
    ui.textEdit_px2->setPlainText(QString::number(LP[2]=0));
    ui.textEdit_py2->setPlainText(QString::number(LP[3]=0));
    ui.No3->setPlainText(QString::number(No[2]=0));
    ui.textEdit_px3->setPlainText(QString::number(LP[4]=0));
    ui.textEdit_py3->setPlainText(QString::number(LP[5]=0));
    ui.No4->setPlainText(QString::number(No[3]=0));
    ui.textEdit_px4->setPlainText(QString::number(LP[6]=0));
    ui.textEdit_py4->setPlainText(QString::number(LP[7]=0));
    ui.No5->setPlainText(QString::number(No[4]=0));
    ui.textEdit_px5->setPlainText(QString::number(LP[8]=0));
    ui.textEdit_py5->setPlainText(QString::number(LP[9]=0));
    ui.No6->setPlainText(QString::number(No[5]=0));
    ui.textEdit_px6->setPlainText(QString::number(LP[10]=0));
    ui.textEdit_py6->setPlainText(QString::number(LP[11]=0));
    ui.No7->setPlainText(QString::number(No[6]=0));
    ui.textEdit_px7->setPlainText(QString::number(LP[12]=0));
    ui.textEdit_py7->setPlainText(QString::number(LP[13]=0));
    ui.No8->setPlainText(QString::number(No[7]=0));
    ui.textEdit_px8->setPlainText(QString::number(LP[14]=0));
    ui.textEdit_py8->setPlainText(QString::number(LP[15]=0));
    ui.No9->setPlainText(QString::number(No[8]=0));
    ui.textEdit_px9->setPlainText(QString::number(LP[16]=0));
    ui.textEdit_py9->setPlainText(QString::number(LP[17]=0));
    ui.No10->setPlainText(QString::number(No[9]=0));
    ui.textEdit_px10->setPlainText(QString::number(LP[18]=0));
    ui.textEdit_py10->setPlainText(QString::number(LP[19]=0));
}
