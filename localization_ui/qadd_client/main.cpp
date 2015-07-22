/**
 * @file /qadd_server/main.cpp
 *
 * @brief Qt based gui.
 *
 * @date November 2010
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QApplication>
#include "client_window.hpp"
#include "qadd.hpp"

/*****************************************************************************
** Main
*****************************************************************************/

int main(int argc, char **argv) {

    /*********************
    ** Qt
    **********************/
    QApplication app(argc, argv);
    QAdd qadd(argc,argv);
    ClientWindow w(&qadd);
    w.show();
    app.connect(&app, SIGNAL(lastWindowClosed()), &app, SLOT(quit()));
    int result = app.exec();

	return result;
}
