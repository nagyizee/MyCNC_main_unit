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


    void on_pb_pause_pressed();

    void on_pb_pause_released();

    void on_pb_resume_pressed();

    void on_pb_resume_released();

    void on_pb_home_pressed();

    void on_pb_home_released();

    void on_nm_real_x_valueChanged(double arg1);

    void on_nm_real_y_valueChanged(double arg1);

    void on_nm_real_z_valueChanged(double arg1);

    void on_nm_real_a_valueChanged(double arg1);

    void on_nm_rotor_x_valueChanged(int arg1);

    void on_nm_rotor_y_valueChanged(int arg1);

    void on_nm_rotor_z_valueChanged(int arg1);

    void on_nm_rotor_a_valueChanged(int arg1);

    void on_pb_bloc_execution_clicked();


    void on_vs_simu_speed_valueChanged(int value);

private:
    Ui::mainw *ui;
    QTimer *ticktimer;
    bool buttons[3];

    QGraphicsScene *scene_xy;
    QGraphicsEllipseItem *pointer_xy;
    QGraphicsLineItem *pointer_z;
    QGraphicsPixmapItem *G_item_xy;
    uchar *gmem_xy;                    // graphic memory for display simulator



public:

    void HW_wrapper_DBG(int val);

    void HW_wrapper_LED(int led, bool on);
    bool HW_wrapper_button_state(int button);


    void Disp_Redraw();
    void dispsim_add_point();
    void dispsim_mem_clean();

    void HW_assertion(const char *reason);
    void HW_wrapper_update_display( void );



private:
    void Application_MainLoop( bool tick );

    void HW_wrapper_setup( int interval );            // set up the this pointer in the hardware wrapper (simulation module)
    void HW_wrp_setcoord( int coord, bool num, double num_val, int step_val );


};

#endif // MAINW_H
