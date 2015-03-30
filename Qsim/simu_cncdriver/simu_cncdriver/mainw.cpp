
#include <QTextBlock>

#include "string.h"
#include "mainw.h"
#include "ui_mainw.h"
#include "hw_stuff.h"
#include "events_ui.h"


// timing - 50us timer interrupt -> obtaining 100us clock -> 10kHz -> 25mm/sec -> 1500mm/min max
#define TIMER_INTERVAL          40       // 40ms -> 25 fps
#define TICKS_TO_SIMULATE       2000      // 800 timer cycles to be simulated for 40ms

int tick_to_sim = TICKS_TO_SIMULATE;

mainw::mainw(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::mainw)
{
    ui->setupUi(this);
    HW_wrapper_setup( TIMER_INTERVAL );

    memset( buttons, 0, sizeof(bool)*3 );
    memset( &cmd_list, 0, sizeof(cmd_list) );


    // setup text boxes
    QTextOption text_opt;
    QFont fnt;

    doc_commands = new QTextDocument("# command list", this);
    doc_comm_log = new QTextDocument("---- communication log ----", this);

    fnt = ui->txt_commands->font();
    doc_commands->setDefaultFont( fnt );
    doc_comm_log->setDefaultFont( fnt );

    ui->txt_commands->setDocument( doc_commands );
    text_opt = doc_commands->defaultTextOption();
    text_opt.setWrapMode( QTextOption::NoWrap );
    doc_commands->setDefaultTextOption( text_opt );
    doc_commands->setMaximumBlockCount(-1);

    ui->txt_comm_window->setDocument( doc_comm_log );
    ui->txt_comm_window->setFont( fnt );
    text_opt = doc_comm_log->defaultTextOption();
    text_opt.setWrapMode( QTextOption::NoWrap );
    doc_comm_log->setDefaultTextOption( text_opt );
    doc_comm_log->setMaximumBlockCount(1000);


    // set up the graphic display simulator
    gmem_xy    = (uchar*)malloc( DISPSIM_MAX_W * DISPSIM_MAX_H * 3 );
    scene_xy   = new QGraphicsScene(0, 0, 1330, 460, ui->gw_xy );

    QImage image( gmem_xy, DISPSIM_MAX_W, DISPSIM_MAX_H, DISPSIM_MAX_W * 3, QImage::Format_RGB888 );
    image.fill(Qt::black );
    G_item_xy  = new QGraphicsPixmapItem( QPixmap::fromImage( image ));

    pointer_xy = new QGraphicsEllipseItem( QRect(-3, -3, 5, 5), 0 );
    pointer_xy->setPen( QPen(Qt::red) );
    pointer_xy->setBrush( QBrush(Qt::NoBrush) );

    pointer_z = new QGraphicsLineItem( 1300, 0, 1330, 0 );
    pointer_z->setPen( QPen(Qt::red) );

    scene_xy->setBackgroundBrush( QBrush(QColor(0x33,0x33,0x33)) );
    scene_xy->addItem(G_item_xy);
    scene_xy->addItem(pointer_xy);
    scene_xy->addItem(pointer_z);

    ui->gw_xy->setAlignment(Qt::AlignLeft);
    ui->gw_xy->setScene(scene_xy);
//    ui->gw_xy->setScene(scene_z);

    // init display
    Disp_Redraw(false);

    ticktimer = new QTimer( this );
    connect(ticktimer, SIGNAL(timeout()), this, SLOT(TimerTick()));
    ticktimer->start( TIMER_INTERVAL );

    //--- test phase
    qsrand(0x64892354);
    main_entry( NULL );

}

mainw::~mainw()
{
    delete ui;
}



/////////////////////////////////////////////////////////////
//  Main app. simulation
/////////////////////////////////////////////////////////////
struct SEventStruct ev;

void mainw::Application_MainLoop( bool tick )
{
    main_loop();
}

void mainw::TimerTick()
{
    int i;
    int j;
    static int ctdown = 0;
    
    for (i=0; i<tick_to_sim; i++)
    {

        // simulated code section ----
        bool    send_tick   = true;
        int     loop;

        loop = (qrand() & 0x03); // it can make 1 - 3 main loops

        for ( j=0; j<loop; j++ )
        {
            Application_MainLoop( send_tick );
            send_tick = false;
        }

        StepTimerIntrHandler();

        if ( i % 0x01 )
            HW_wrp_front_end_simu();        // 25khz rate  ~235kbps
        if ( i % 5 )                        // 10khz rate  - close to 11kbps
            HW_wrp_simu_datafeed();

    }

    if ( ctdown == 0 )
    {
        HW_wrapper_update_display( true );
        ctdown = 1;
    }
    else
    {
        HW_wrapper_update_display( false );
        ctdown--;
    }
}


/////////////////////////////////////////////////////////////
// UI elements
/////////////////////////////////////////////////////////////


void mainw::on_pb_pause_pressed()
{
    buttons[0] = true;
}

void mainw::on_pb_pause_released()
{
    buttons[0] = false;
}

void mainw::on_pb_resume_pressed()
{
    buttons[1] = true;
}

void mainw::on_pb_resume_released()
{
    buttons[1] = false;
}

void mainw::on_pb_home_pressed()
{
    buttons[2] = true;
}

void mainw::on_pb_home_released()
{
    buttons[2] = false;
}

void mainw::on_nm_real_x_valueChanged(double arg1)
{
    if ( ui->pb_bloc_execution->isChecked() )
        HW_wrp_setcoord( COORD_X, true, arg1, 0 );
}

void mainw::on_nm_real_y_valueChanged(double arg1)
{
    if ( ui->pb_bloc_execution->isChecked() )
        HW_wrp_setcoord( COORD_Y, true, arg1, 0 );
}

void mainw::on_nm_real_z_valueChanged(double arg1)
{
    if ( ui->pb_bloc_execution->isChecked() )
        HW_wrp_setcoord( COORD_Z, true, arg1, 0 );
}

void mainw::on_nm_real_a_valueChanged(double arg1)
{
    if ( ui->pb_bloc_execution->isChecked() )
        HW_wrp_setcoord( COORD_A, true, arg1, 0 );
}


void mainw::on_nm_rotor_x_valueChanged(int arg1)
{
    if ( ui->pb_bloc_execution->isChecked() )
        HW_wrp_setcoord( COORD_X, false, 0, arg1 );
}

void mainw::on_nm_rotor_y_valueChanged(int arg1)
{
    if ( ui->pb_bloc_execution->isChecked() )
        HW_wrp_setcoord( COORD_Y, false, 0, arg1 );
}

void mainw::on_nm_rotor_z_valueChanged(int arg1)
{
    if ( ui->pb_bloc_execution->isChecked() )
        HW_wrp_setcoord( COORD_Z, false, 0, arg1 );
}

void mainw::on_nm_rotor_a_valueChanged(int arg1)
{
    if ( ui->pb_bloc_execution->isChecked() )
        HW_wrp_setcoord( COORD_A, false, 0, arg1 );
}


void mainw::on_pb_bloc_execution_clicked()
{
    if ( ui->pb_bloc_execution->isChecked() )
        ticktimer->stop();
    else
        ticktimer->start();

}


void mainw::on_vs_simu_speed_valueChanged(int value)
{
    double ts[] = { 1.0/TICKS_TO_SIMULATE,                  // 1 tick / sim
                    0.125, 0.25, 0.375,
                    0.5,                                // 0.5 from max ticks
                    0.625, 0.75, 0.875,
                    1 };                                    // max ticks / sim

    tick_to_sim =  (int)(TICKS_TO_SIMULATE * ts[value]);
    if ( tick_to_sim == 0)
        tick_to_sim = 1;
}

void mainw::on_pb_start_clicked()
{
    HW_wrp_motion_start();
}

void mainw::on_pb_feedSeq_clicked()
{
    HW_wrp_feed_seq();
}


void mainw::on_pb_stop_clicked()
{
    HW_wrp_stop();
}

void mainw::on_nm_scale_editingFinished()
{
    HW_wrp_set_speedFactor( ui->nm_scale->value() );
}

void mainw::on_pb_fe_spindle_jam_clicked()
{
    HW_wrp_spindle_jam();
}

void mainw::on_pb_cmd_feed_line_clicked()
{
    QTextBlock tb;
    QTextCharFormat format;
    QString line;
    int res;

    tb = doc_commands->findBlockByNumber( cmd_list.crt_line );
    QTextCursor tc( doc_commands->findBlockByNumber( cmd_list.crt_line ) );
    line = tb.text();

    res = HW_wrp_input_line( line );
    if (res == -1)
    {
        format.setForeground( Qt::red );
    }
    else if ( res == 0 )
    {
        format.setForeground( Qt::lightGray );
        cmd_list.crt_line++;
    }

    tc.movePosition( QTextCursor::StartOfBlock);
    ui->txt_commands->setTextCursor(tc);
    tc.select(QTextCursor::BlockUnderCursor);
    tc.setCharFormat( format );

    // format.setBackground( Qt::transparent );

}

void mainw::on_pb_cmd_restart_clicked()
{
    QTextCharFormat format;
    QTextCursor tc( doc_commands );

    cmd_list.crt_line = 0;

    tc.movePosition( QTextCursor::StartOfBlock);
    ui->txt_commands->setTextCursor(tc);
    tc.select(QTextCursor::Document);
    format.setForeground( Qt::black );
    tc.setCharFormat( format );
}
