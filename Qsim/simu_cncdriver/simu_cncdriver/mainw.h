#ifndef MAINW_H
#define MAINW_H

#include <QMainWindow>
#include <QGraphicsScene>
#include <QGraphicsPixmapItem>
#include <QPixmap>
#include <QTimer>


#define DISPSIM_MAX_W      1300
#define DISPSIM_MAX_H      460


namespace Ui {
class mainw;
}

class mainw : public QMainWindow
{
    Q_OBJECT
    
public:
    explicit mainw(QWidget *parent = 0);
    ~mainw();

private slots:
    void TimerTick();


private:
    Ui::mainw *ui;
    QTimer *ticktimer;

    QGraphicsScene *scene_xy;
    QGraphicsEllipseItem *pointer_xy;
    QGraphicsLineItem *pointer_z;
    QGraphicsPixmapItem *G_item_xy;
    uchar *gmem_xy;                    // graphic memory for display simulator

    void dispsim_mem_clean();

    void HW_wrapper_setup( int interval );            // set up the this pointer in the hardware wrapper (simulation module)

public:

    void HW_wrapper_DBG(int val);

    void HW_wrapper_LED(int led, bool on);


    void Disp_Redraw();
    void HW_assertion(const char *reason);
    void HW_wrapper_update_display( void );


private:
    void Application_MainLoop( bool tick );

};

#endif // MAINW_H
