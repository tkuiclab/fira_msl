#include <QtGui/QApplication>
#include "interface_window.hpp"
#include "qInterface.hpp"

int main(int argc, char *argv[])
{
    QInterface m(argc,argv);
    QApplication a(argc, argv);
    interface_window w(&m);
    w.show();
    return a.exec();
}
