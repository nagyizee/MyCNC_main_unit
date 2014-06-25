#include "mainw.h"
#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    mainw w;
//    graph_disp g;
//    g.show();
    w.show();
//    w.setup_graphic( &g );
    
    return a.exec();
}
