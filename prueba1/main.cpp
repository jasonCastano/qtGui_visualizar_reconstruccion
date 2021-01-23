#include "pclviewer_1.h"
#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    PCLViewer_1 w;
    w.show();

    return a.exec();
}
