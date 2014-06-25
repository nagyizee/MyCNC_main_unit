#include "mainw.h"
#include "ui_mainw.h"
#include "cnc_defs.h"
#include "hw_stuff.h"


#define MAX_AXIS_CHANNELS 4


mainw *pClass;
int tiv;

/// display stuff /////////////
#define MAX_LINES       2
#define MAX_COLOUMNS    16
#define EEPROM_SIZE     0x200

char line[MAX_LINES*MAX_COLOUMNS];
static bool disp_changed = false;
/////////////////////////////////

uint8 eeprom_cont[ EEPROM_SIZE ];


struct SStepCoordinates hw_coords;


void InitHW(void)
{
}


void HW_ASSERT(const char *reason)
{
    pClass->HW_assertion(reason);
}

void HWDBG( int val )
{
    pClass->HW_wrapper_DBG( val );
}


void LED_On( int i )
{
    pClass->HW_wrapper_LED( i, true );
}

void LED_Off( int i )
{
    pClass->HW_wrapper_LED( i, false );
}


bool BtnGet_Emerg()
{
    return false;
}

bool BtnGet_Resume()
{
    return false;
}

bool BtnGet_Home()
{
    return false;
}

void HW_SetDirX_Plus()
{
}

void HW_SetDirX_Minus()
{
}

void HW_SetDirY_Plus()
{
}

void HW_SetDirY_Minus()
{
}

void HW_SetDirZ_Plus()
{
}

void HW_SetDirZ_Minus()
{
}

void HW_SetDirA_Plus()
{
}

void HW_SetDirA_Minus()
{
}

void HW_StepClk_X()
{
}

void HW_StepClk_Y()
{
}

void HW_StepClk_Z()
{
}

void HW_StepClk_A()
{
}

void HW_StepClk_Reset()
{
}




///////////////////////////////////////////////////

void mainw::HW_assertion(const char *reason)
{
    // ui->cb_syserror->setChecked(true);
    // TODO
}


////////////////////////////////////////////////////

void mainw::HW_wrapper_setup( int interval )
{
    pClass = this;
    tiv    = interval;

    hw_coords.coord[COORD_X] = 130*400;
    hw_coords.coord[COORD_Y] = 46*400;
    hw_coords.coord[COORD_Z] = 80*400;
    hw_coords.coord[COORD_A] = 0*400;

}

// custom debug interface
void mainw::HW_wrapper_DBG( int val )
{


}


void mainw::HW_wrapper_LED( int led, bool on )
{
    switch (led)
    {
        case 0:
            ui->rb_led1->setChecked( on );
            break;
        case 1:
            ui->rb_led2->setChecked( on );
            break;
        case 2:
            ui->rb_led3->setChecked( on );
            break;
    }
}


void mainw::HW_wrapper_update_display( void )
{
    Disp_Redraw();

}

/////////////////////////////////////////////////////
// Display simulation
/////////////////////////////////////////////////////

void mainw::dispsim_mem_clean()
{
}


void mainw::Disp_Redraw()
{
    int x,y;

    memset( gmem_xy, 0, DISPSIM_MAX_W * DISPSIM_MAX_H *3 );

    QImage image( gmem_xy, DISPSIM_MAX_W, DISPSIM_MAX_H, DISPSIM_MAX_W * 3, QImage::Format_RGB888 );

    scene_xy->removeItem(G_item_xy);
    delete G_item_xy;
    G_item_xy  = new QGraphicsPixmapItem( QPixmap::fromImage( image ));
    scene_xy->addItem(G_item_xy);

    pointer_z->setPos( 0, 460 - hw_coords.coord[COORD_Z] * 460 / ( 80 * 400 ) );
    pointer_xy->setPos( hw_coords.coord[COORD_X] / 40 , 460 - hw_coords.coord[COORD_Y] / 40 );
    pointer_xy->setZValue(1);

    ui->dl_rotor_x->setValue( hw_coords.coord[COORD_X] % 400 );
    ui->dl_rotor_y->setValue( hw_coords.coord[COORD_Y] % 400 );
    ui->dl_rotor_z->setValue( hw_coords.coord[COORD_Z] % 400 );
    ui->dl_rotor_a->setValue( hw_coords.coord[COORD_A] % 400 );

    ui->nm_rotor_x->setValue( hw_coords.coord[COORD_X] );
    ui->nm_rotor_y->setValue( hw_coords.coord[COORD_Y] );
    ui->nm_rotor_z->setValue( hw_coords.coord[COORD_Z] );
    ui->nm_rotor_a->setValue( hw_coords.coord[COORD_A] );

    ui->nm_real_x->setValue( hw_coords.coord[COORD_X] / 400.0 );
    ui->nm_real_y->setValue( hw_coords.coord[COORD_Y] / 400.0 );
    ui->nm_real_z->setValue( hw_coords.coord[COORD_Z] / 400.0 );
    ui->nm_real_a->setValue( hw_coords.coord[COORD_A] / 400.0 );

}
