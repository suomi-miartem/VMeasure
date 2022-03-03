#include "QPCLVisualizer.h"
#include <QtWidgets/QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    QPCLVisualizer w;
    w.show();
    return a.exec();
}
