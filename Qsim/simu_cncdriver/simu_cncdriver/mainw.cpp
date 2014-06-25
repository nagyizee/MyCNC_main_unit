
#include <QTextBlock>

#include "string.h"
#include "mainw.h"
#include "ui_mainw.h"
#include "hw_stuff.h"
#include "events_ui.h"


// timing - 50us timer interrupt -> obtaining 100us clock -> 10kHz -> 25mm/sec -> 1500mm/min max
#define TIMER_INTERVAL          40       // 40ms -> 25 fps
#define TICKS_TO_SIMULATE       800      // 800 timer cycles to be simulated for 40ms


mainw::mainw(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::mainw)
{
    ui->setupUi(this);
    HW_wrapper_setup( TIMER_INTERVAL );

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
    Disp_Redraw();

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

    for (i=0; i<TICKS_TO_SIMULATE; i++)
    {

        // simulated code section ----
        bool    send_tick   = true;
        int     loop;

        loop = (qrand() & 0x03) + 1; // it can make 1 - 3 main loops

        for ( j=0; j<loop; j++ )
        {
            Application_MainLoop( send_tick );
            send_tick = false;
        }

        StepTimerIntrHandler();
    }

    HW_wrapper_update_display();
}


/////////////////////////////////////////////////////////////
// UI elements
/////////////////////////////////////////////////////////////

