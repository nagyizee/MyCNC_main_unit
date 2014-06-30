#include "mainw.h"
#include "ui_mainw.h"
#include "motion_core.h"
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
bool dirs[CNC_MAX_COORDS] = { false, };


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
    return pClass->HW_wrapper_button_state(0);
}

bool BtnGet_Resume()
{
    return pClass->HW_wrapper_button_state(1);
}

bool BtnGet_Home()
{
    return pClass->HW_wrapper_button_state(2);
}

void HW_SetDirX_Plus()
{
    dirs[COORD_X] = true;
}

void HW_SetDirX_Minus()
{
    dirs[COORD_X] = false;
}

void HW_SetDirY_Plus()
{
    dirs[COORD_Y] = true;
}

void HW_SetDirY_Minus()
{
    dirs[COORD_Y] = false;
}

void HW_SetDirZ_Plus()
{
    dirs[COORD_Z] = true;
}

void HW_SetDirZ_Minus()
{
    dirs[COORD_Z] = false;
}

void HW_SetDirA_Plus()
{
    dirs[COORD_A] = true;
}

void HW_SetDirA_Minus()
{
    dirs[COORD_A] = false;
}

void HW_StepClk_X()
{
    hw_coords.coord[COORD_X] += dirs[COORD_X] ? 1 : -1;
    pClass->dispsim_add_point();
}

void HW_StepClk_Y()
{
    hw_coords.coord[COORD_Y] += dirs[COORD_Y] ? 1 : -1;
    pClass->dispsim_add_point();
}

void HW_StepClk_Z()
{
    hw_coords.coord[COORD_Z] += dirs[COORD_Z] ? 1 : -1;
    pClass->dispsim_add_point();
}

void HW_StepClk_A()
{
    hw_coords.coord[COORD_A] += dirs[COORD_A] ? 1 : -1;
    pClass->dispsim_add_point();
}

void HW_StepClk_Reset()
{
    //dummy
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


bool mainw::HW_wrapper_button_state(int button)
{
    return buttons[button];
}


void mainw::HW_wrapper_update_display( void )
{
    Disp_Redraw();

}


void mainw::HW_wrp_setcoord( int coord, bool num, double num_val, int step_val )
{
    if (num)
        hw_coords.coord[coord] = (TStepCoord)( num_val * 400 );
    else
        hw_coords.coord[coord] = step_val;

    Disp_Redraw();
}


/////////////////////////////////////////////////////
// Display simulation
/////////////////////////////////////////////////////

void mainw::dispsim_mem_clean()
{
    memset( gmem_xy, 0, DISPSIM_MAX_W * DISPSIM_MAX_H *3 );
}


void mainw::dispsim_add_point()
{
    uint32 mempoz;
    mempoz = 3 * ( (hw_coords.coord[COORD_X] / 40) + (460 - hw_coords.coord[COORD_Y] / 40) * DISPSIM_MAX_W );
    if ( (mempoz + 2) >= DISPSIM_MAX_W * DISPSIM_MAX_H *3 )
        return;

    gmem_xy[ mempoz ] = 0x50;
    gmem_xy[ mempoz + 1 ] = 0xff;
    gmem_xy[ mempoz + 2 ] = 0x30;
}


void mainw::Disp_Redraw()
{
    int x,y;


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

    SStepCoordinates soft_coord;

    motion_get_crt_coord( &soft_coord );
    ui->nm_stepx->setValue( soft_coord.coord[COORD_X] );
    ui->nm_stepy->setValue( soft_coord.coord[COORD_Y] );
    ui->nm_stepz->setValue( soft_coord.coord[COORD_Z] );
    ui->nm_stepa->setValue( soft_coord.coord[COORD_A] );

}
