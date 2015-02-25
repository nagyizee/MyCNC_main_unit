#include <math.h>
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
/////////////////////////////////

uint8 eeprom_cont[ EEPROM_SIZE ];


struct SStepCoordinates hw_coords;
bool dirs[CNC_MAX_COORDS] = { false, };

struct SDebugStep
{
    FILE *log_file;
    struct SStepCoordinates c1;
    struct SStepCoordinates c2;
    int step_count;
    int tick_count;
};

struct SDebugStep dbg_step;

struct SAccelDebug
{
    int phase;
    int sense;
    bool dirty;
} accel;



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




struct SPoint
{
    double x;
    double y;
    double z;
};

struct SLine
{
    struct SPoint P0;
    struct SPoint P1;
};

typedef SPoint TVector;

#define dot(u,v)   ( u.x * v.x + u.y * v.y + u.z * v.z )
#define norm(v)    sqrt(dot(v,v))     // norm = length of  vector
#define dst(u,v)     norm(u-v)          // distance = norm of difference


TVector vect_minus( TVector *v1, TVector *v2 )
{
    TVector v;
    v.x = v1->x - v2->x;
    v.y = v1->y - v2->y;
    v.z = v1->z - v2->z;
    return v;
}

TVector vect_plus( TVector *v1, TVector *v2 )
{
    TVector v;
    v.x = v1->x + v2->x;
    v.y = v1->y + v2->y;
    v.z = v1->z + v2->z;
    return v;
}

TVector vect_scalardot( double sc, TVector *v1 )
{
    TVector v;
    v.x = v1->x * sc;
    v.y = v1->y * sc;
    v.z = v1->z * sc;
    return v;
}

void point_setval( SPoint *p, struct SStepCoordinates *c )
{
    p->x = c->coord[COORD_X];
    p->y = c->coord[COORD_Y];
    p->z = c->coord[COORD_Z];
}


void StepDBG_AddPoint()
{
    double dist;

    if ( dbg_step.log_file == NULL )
        return;
/* Calculate distance of the current step point from the theoretical 3D line defined
 * by the start and end coordinates
 *
 *original code:
 dist_Point_to_Line(Point P, Line L)
{
     Vector v = L.P1 - L.P0;
     Vector w = P - L.P0;

     double c1 = dot(w,v);
     double c2 = dot(v,v);
     double b = c1 / c2;

     Point Pb = L.P0 + b * v;
     return d(P, Pb);
}
    */

    struct SPoint P;
    struct SLine L;

    point_setval( &P, &hw_coords );
    point_setval( &L.P0, &dbg_step.c1 );
    point_setval( &L.P1, &dbg_step.c2 );

    TVector v = vect_minus( &L.P1, &L.P0 );
    TVector w = vect_minus( &P,    &L.P0 );

    double c1 = dot(w,v);
    double c2 = dot(v,v);
    double b = c1 / c2;

    SPoint Psc = vect_scalardot( b , &v );
    SPoint Pb = vect_plus( &L.P0 , &Psc );
    TVector Vfinal = vect_minus( &P, &Pb );
    dist = norm( Vfinal );                      //dst(P, Pb)  -> norm( P - Pb ) -> norm( vect minus( P, Pb )

    dbg_step.step_count++;
    fprintf( dbg_step.log_file, "    %06i,  %2.4lf, %s\n", dbg_step.step_count, dist, (dist > 1.0) ?  (dist > 2.0) ? "<--- 2" : "<--- 1" :" " );

}


void HW_StepClk_X()
{
    hw_coords.coord[COORD_X] += dirs[COORD_X] ? 1 : -1;
    pClass->dispsim_add_point();
    StepDBG_AddPoint();
}

void HW_StepClk_Y()
{
    hw_coords.coord[COORD_Y] += dirs[COORD_Y] ? 1 : -1;
    pClass->dispsim_add_point();
    StepDBG_AddPoint();
}

void HW_StepClk_Z()
{
    hw_coords.coord[COORD_Z] += dirs[COORD_Z] ? 1 : -1;
    pClass->dispsim_add_point();
    StepDBG_AddPoint();
}

void HW_StepClk_A()
{
    hw_coords.coord[COORD_A] += dirs[COORD_A] ? 1 : -1;
    pClass->dispsim_add_point();
    StepDBG_AddPoint();
}

void HW_StepClk_Reset() {  /*dummy*/  }
void HW_ResetClk_X() {  /*dummy*/  }
void HW_ResetClk_Y() {  /*dummy*/  }
void HW_ResetClk_Z() {  /*dummy*/  }
void HW_ResetClk_A() {  /*dummy*/  }
 
void StepDBG_Accelerations( int phase, int sense )
{
    if ( (phase != accel.phase) || (sense != accel.sense) )
    {
        accel.phase = phase;
        accel.sense = sense;
        accel.dirty = true;
    }
}

void StepDBG_LineSegment( struct SStepCoordinates *c1, struct SStepCoordinates *c2 )
{
    char filename[128];
    sprintf( filename, "segment_line__%d-%d-%d__%d-%d-%d.log", c1->coord[0], c1->coord[1], c1->coord[2], c2->coord[0], c2->coord[1], c2->coord[2]);
    dbg_step.log_file = fopen( filename, "w" );
    if ( dbg_step.log_file )
    {
        dbg_step.step_count = 0;
        dbg_step.tick_count = 0;
        fprintf(dbg_step.log_file, "L: xyz[%06d,%06d,%06d] <-> xyz[%06d,%06d,%06d]\n", c1->coord[0], c1->coord[1], c1->coord[2], c2->coord[0], c2->coord[1], c2->coord[2]);
        dbg_step.c1 = *c1;
        dbg_step.c2 = *c2;
    }
}

void StepDBG_SegmentFinished()
{
    accel.dirty = true;
    accel.phase = 3;
    if ( dbg_step.log_file )
    {
        fprintf(dbg_step.log_file, "\n");
        fclose(dbg_step.log_file);
    }
}

void StepDBG_TickCount()
{
    dbg_step.tick_count++;
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


void mainw::HW_wrapper_update_display( bool redrw_ui )
{
    Disp_Redraw(redrw_ui);

}


void mainw::HW_wrp_setcoord( int coord, bool num, double num_val, int step_val )
{
    if (num)
        hw_coords.coord[coord] = (TStepCoord)( num_val * 400 );
    else
        hw_coords.coord[coord] = step_val;

    Disp_Redraw(false);
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


void mainw::Disp_Redraw( bool redrw_ui )
{
    QImage image( gmem_xy, DISPSIM_MAX_W, DISPSIM_MAX_H, DISPSIM_MAX_W * 3, QImage::Format_RGB888 );

    scene_xy->removeItem(G_item_xy);
    delete G_item_xy;
    G_item_xy  = new QGraphicsPixmapItem( QPixmap::fromImage( image ));
    scene_xy->addItem(G_item_xy);

    pointer_z->setPos( 0, 460 - hw_coords.coord[COORD_Z] * 460 / ( 80 * 400 ) );
    pointer_xy->setPos( hw_coords.coord[COORD_X] / 40 , 460 - hw_coords.coord[COORD_Y] / 40 );
    pointer_xy->setZValue(1);

    if ( redrw_ui )
    {
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

        if ( accel.dirty )
        {
            switch (accel.phase)
            {
                case 0:
                    ui->cb_start->setChecked(true);
                    ui->cb_const->setChecked(false);
                    ui->cb_end->setChecked(false);
                    ui->cb_acc->setChecked( accel.sense ? true : false );
                    ui->cb_dec->setChecked( accel.sense ? false : true );
                    break;
                case 1:
                    ui->cb_start->setChecked(false);
                    ui->cb_const->setChecked(true);
                    ui->cb_end->setChecked(false);
                    ui->cb_acc->setChecked( false );
                    ui->cb_dec->setChecked( false );
                    break;
                case 2:
                    ui->cb_start->setChecked(false);
                    ui->cb_const->setChecked(false);
                    ui->cb_end->setChecked(true);
                    ui->cb_acc->setChecked( accel.sense ? true : false );
                    ui->cb_dec->setChecked( accel.sense ? false : true );
                    break;
                case 3:
                    ui->cb_start->setChecked(false);
                    ui->cb_const->setChecked(false);
                    ui->cb_end->setChecked(false);
                    ui->cb_acc->setChecked( false );
                    ui->cb_dec->setChecked( false );
                    break;
            }
            accel.dirty = false;
        }
    }

}
