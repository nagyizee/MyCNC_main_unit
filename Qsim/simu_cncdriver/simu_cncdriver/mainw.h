#ifndef MAINW_H
#define MAINW_H

#include <QMainWindow>
#include <QGraphicsScene>
#include <QGraphicsPixmapItem>
#include <QPixmap>
#include <QTimer>
#include <QtNetwork/QTcpServer>
#include <QtNetwork/QTcpSocket>


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

    void on_pb_start_clicked();

    void on_pb_stop_clicked();

    void on_pb_fe_spindle_jam_clicked();

    void on_pb_cmd_feed_line_clicked();

    void on_pb_cmd_restart_clicked();

    void on_pb_get_coord_clicked();

    void on_pb_get_status_clicked();

    void on_pb_pause_2_clicked();

private:
    Ui::mainw *ui;
    QTimer *ticktimer;
    QTcpServer *comm;
    QTcpSocket *client;
    bool buttons[3];

    QGraphicsScene *scene_xy;
    QGraphicsEllipseItem *pointer_xy;
    QGraphicsLineItem *pointer_z;
    QGraphicsPixmapItem *G_item_xy;
    uchar *gmem_xy;                    // graphic memory for display simulator

    struct SCommandList
    {
        int crt_line;

    } cmd_list;


    QTextDocument *doc_commands;
    QTextDocument *doc_comm_log;


private slots:
    void tcp_newConnection();
    void tcp_disconnected();
    void tcp_readyRead();
private:
    void tcp_Send(unsigned char *buffer, int length);

public:

    void HW_wrapper_DBG(int val);

    void HW_wrapper_LED(int led, bool on);
    bool HW_wrapper_button_state(int button);


    void Disp_Redraw( bool redrw_ui );
    void dispsim_add_point();
    void dispsim_mem_clean();

    void HW_assertion(const char *reason);
    void HW_wrapper_update_display( bool redrw_ui );
    bool HW_wrp_fe_broken_link();

    void HW_wrp_front_end_simu();


private:
    bool commwait_getstatus;
    bool commwait_getcoord;

    void Application_MainLoop( bool tick );

    void HW_wrapper_setup( int interval );            // set up the this pointer in the hardware wrapper (simulation module)
    void HW_wrp_setcoord( int coord, bool num, double num_val, int step_val );

    void HW_wrp_motion_start();
    void HW_wrp_feed_seq();
    void HW_wrp_stop();
    void HW_wrp_set_speedFactor( int factor );

    int HW_wrp_insert_message( unsigned char *buffer, int size, int tx_type );
    int HW_wrp_input_line(QString line);
    int HW_wrp_inject_command( unsigned char *outdata, int size );
    int HW_wrp_simu_datafeed();


    void HW_wrp_spindle_jam(void);

};

#endif // MAINW_H
