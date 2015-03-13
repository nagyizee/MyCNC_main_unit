#include <math.h>
#include "mainw.h"
#include "ui_mainw.h"
#include "motion_core.h"
#include "cnc_defs.h"
#include "hw_stuff.h"
#include "comm_fe.h"
#include "frontend_internals.h"

#define MAX_AXIS_CHANNELS 4


mainw *pClass;
int tiv;

/// test stuff ///////////


#define C(a)  (a * 400)

struct SStepCoordinates CoordList[] = { { C(10), C(0), C(0), 0 },
                                        { C(10), C(10), C(0), 0 },
                                        { C(15), C(20), C(0), 0 },
                                        { C(12), C(20), C(0), 0 },
                                        { 0, 0, 0, 0 },                 // dummy for hold time
                                        { C(10), C(20), C(0), 0 }
};
TFeedSpeed speeds[]     = { 1200, 1200, 1200, 1200, 0x8000 | 5 ,1200 };

/*
// long run stuff
struct SStepCoordinates CoordList[] = { { C(10), C(10), C(0), 0 },
                                        { C(120), C(10), C(0), 0 },
                                        { C(120), C(40), C(0), 0 },
                                        { C(10), C(40), C(0), 0 },
                                        { C(120), C(10), C(0), 0 },
                                        { C(70), C(20), C(0), 0 },
};
TFeedSpeed speeds[]     = { 1200, 1200, 1200, 1200, 1200, 1200 };
*/



/*
// sharp angles on x direction ( x+, x- )
struct SStepCoordinates CoordList[] = { { C(20), C(0), C(0), 0 },
                                        { C(10), C(5), C(0), 0 },       // cosT -0.89442719099991587856366946749251
                                        { C(20), C(6), C(0), 0 }        // cosT  -0.84548890303097109985616766032539
};
TFeedSpeed speeds[]     = { 1200, 1200, 1200, 1200, 1200 };
*/

/*
// sharp angles - star from 20,20 coordinate
struct SStepCoordinates CoordList[] = { { C(20), C(20), C(0), 0 },
                                        { C(18), C(10), C(0), 0 },  // -0.83205029448567147
                                        { C(20), C(20), C(0), 0 },  
                                        { C(25), C(10), C(0), 0 },  // -0.78935221764145469
                                        { C(20), C(20), C(0), 0 },  
                                        { C(27), C(16), C(0), 0 },  // -0.83205029460874491
                                        { C(20), C(20), C(0), 0 },  
                                        { C(27), C(25), C(0), 0 },  // -0.41814360575714005
                                        { C(20), C(20), C(0), 0 },  
                                        { C(24), C(30), C(0), 0 },  // -0.84187913940282599
                                        { C(20), C(20), C(0), 0 },  
                                        { C(17), C(30), C(0), 0 },  // -0.78260105442445083
                                        { C(20), C(20), C(0), 0 },  
                                        { C(10), C(25), C(0), 0 },  // -0.68536469935377853
                                        { C(20), C(20), C(0), 0 },  
                                        { C(10), C(18), C(0), 0 }   // -0.78935221764145469
                                                                    // -0.38461538468517131 on return
};
TFeedSpeed speeds[]     = { 1200, 1200, 1200, 1200, 1200, 1200, 1200, 1200, 1200, 1200, 1200, 1200, 1200, 1200, 1200, 1200 };
*/

/*
// sharp angles - star from 20,20 coordinate
struct SStepCoordinates CoordList[] = { { C(20), C(20), C(0), 0 },   
                                        { C(35), C(10), C(0), 0 },  // 0.1961161351666954

                                        { C(20), C(20), C(0), 0 },  
                                        { C(20), C(30), C(0), 0 },  // 0.55470019625764344

                                        { C(20), C(20), C(0), 0 },  
                                        { C(10), C(17), C(0), 0 },  // 0.28734788564213337
                                                                    //  on return 0.63756771423964109
};
TFeedSpeed speeds[]     = { 1200, 1200, 1200, 1200, 1200, 1200, 1200, 1200, 1200, 1200, 1200, 1200, 1200, 1200, 1200, 1200 };
*/

/*
// line - different speeds
struct SStepCoordinates CoordList[] = { { 1765, C(0), C(0), 0 },
                                        { C(30), C(0), C(0), 0 },   // same speed
                                        { C(40), C(0), C(0), 0 },   // slower speed
                                        { C(80), C(0), C(0), 0 },   // faster speed
};
TFeedSpeed speeds[]     = { 1200, 1200, 300, 1400 };
*/










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
    int pwr[CNC_MAX_COORDS];
    int segment_ticks;
} accel;

int seq_ctr = 0;

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


void HW_SetPower( int axis, int pwr_level )
{
    if ( accel.pwr[axis] == pwr_level )
        return;
    accel.pwr[axis] = pwr_level;
    accel.dirty = true;
}

bool HW_IsPowerSet( int axis )
{
    return true;
}


void StepDBG_Accelerations( int phase, int sense )
{
    if ( (phase != accel.phase) || (sense != accel.sense) )
    {
        accel.phase = phase;
        accel.sense = sense;
        accel.dirty = true;
    }
}

void StepDBG_LineSegment( struct SStepCoordinates *c1, struct SStepCoordinates *c2, int secuenceID )
{
    char filename[128];
    sprintf( filename, "segment_line__%d-%d-%d__%d-%d-%d.log", c1->coord[0], c1->coord[1], c1->coord[2], c2->coord[0], c2->coord[1], c2->coord[2]);
    dbg_step.log_file = fopen( filename, "w" );
    if ( dbg_step.log_file )
    {
        dbg_step.step_count = 0;
        fprintf(dbg_step.log_file, "L: xyz[%06d,%06d,%06d] <-> xyz[%06d,%06d,%06d]\n", c1->coord[0], c1->coord[1], c1->coord[2], c2->coord[0], c2->coord[1], c2->coord[2]);
        dbg_step.c1 = *c1;
        dbg_step.c2 = *c2;
    }
    seq_ctr = secuenceID;
}

void StepDBG_SegmentFinished()
{
    accel.dirty = true;
    accel.segment_ticks = dbg_step.tick_count;
    accel.phase = 3;
    dbg_step.tick_count = 0;
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

    hw_coords.coord[COORD_X] = CNC_DEFAULT_X;
    hw_coords.coord[COORD_Y] = CNC_DEFAULT_Y;
    hw_coords.coord[COORD_Z] = CNC_DEFAULT_Z;
    hw_coords.coord[COORD_A] = CNC_DEFAULT_A;

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




//////////////// test routines
static int lst = 0;

void mainw::HW_wrp_motion_start()
{
    motion_sequence_start();
}

void mainw::HW_wrp_feed_seq()
{
    struct SMotionSequence m;
    int str_size = (sizeof(CoordList) / sizeof(struct SStepCoordinates));
    m.cmdID = lst & 0xff;
    m.seqID = lst & 0xff;

    if ( speeds[lst] & 0x8000 )
    {
        m.seqType = SEQ_TYPE_HOLD;
        m.params.hold = speeds[lst] & ~0xC000;
    }
    else if ( speeds[lst] & 0x4000 )
    {
        m.seqType = SEQ_TYPE_SPINDLE;
        m.params.spindle = speeds[lst] & ~0xC000;
    }
    else
    {
        m.seqType = SEQ_TYPE_GOTO;
        m.params.go_to.feed = speeds[lst];
        m.params.go_to.coord = CoordList[lst];
    }

    motion_sequence_insert( &m );

    lst++;
    if ( lst >= str_size )
         lst = 0;

}


void mainw::HW_wrp_stop()
{
    motion_sequence_stop();
}

void mainw::HW_wrp_set_speedFactor( int factor )
{
    motion_feed_scale( factor );
}

bool mainw::HW_wrp_fe_broken_link()
{
    return ui->pb_fe_break_link->isChecked();
}

/////////////////////////////////////////////////////
// front end simulation
/////////////////////////////////////////////////////

const char cmd_spindle[] =  { 0xA9, 0x14 };
const char cmd_getevents[] = { 0xAA, 0xA9 };                // has no cksum
const char cmd_setevmask[] = { 0xA9, 0x13 };
const char cmd_setspeed[] = { 0xA9, 0x11 };
const char cmd_getrpm[] = { 0xA9, 0x00 };
const char cmd_getcoord[] = { 0xAA, 0x9A };                 // no checksum here also
const char cmd_resetcoord[] = { 0xA9, 0x05 };

#define SSIMU_CMD_SPINDLE_PWR   1
#define SSIMU_CMD_GET_EVENTS    2
#define SSIMU_CMD_SET_EVMASK    3
#define SSIMU_CMD_SET_SPEED     4
#define SSIMU_CMD_GET_RPM       5
#define SSIMU_CMD_GET_COORD     6
#define SSIMU_CMD_RESET_COORD   7

struct SSpindleSimu
{
    int cmd_in_run;
    bool cmd_finished;

    int resp_time;
    int poll_ctr;

    bool    spindle_on;
    int     spindle_set_speed;
    int     spindle_crt_speed;
    uint32  evmask;

    uint32  sensor_status;

    uint32  events;
    bool    flag;              // event flag
    uint8   resp_data[20];
    int     resp_len;

} ssimu;


void mainw::HW_wrp_spindle_jam(void)
{
    if ( ssimu.spindle_set_speed == 0 )
        return;

    ssimu.spindle_crt_speed = 0;
    ssimu.spindle_set_speed = 0;

    if ( (ssimu.evmask & FE_EV_SPINDLE_JAM) &&
         ((ssimu.events & FE_EV_SPINDLE_JAM) == 0 ) )
        ssimu.flag = true;                  // if event mask was active - set the flag
    ssimu.events |= FE_EV_SPINDLE_JAM;
}


bool HW_FrontEnd_Event(void)
{
    return ssimu.flag;
}

void mainw::HW_wrp_front_end_simu()
{
    uint32 sstat = 0;

    // Set up events
    sstat |= ui->cb_fe_x->isChecked() ? FE_TOUCH_AXIS_X : 0;
    sstat |= ui->cb_fe_y->isChecked() ? FE_TOUCH_AXIS_Y : 0;
    sstat |= ui->cb_fe_z->isChecked() ? FE_TOUCH_AXIS_Z : 0;
    sstat |= ui->cb_fe_a->isChecked() ? FE_TOUCH_AXIS_A : 0;
    sstat |= ui->cb_fe_p->isChecked() ? FE_TOUCH_PROBE : 0;
    // check for endpoints
    if ( ((sstat & (FE_TOUCH_AXIS_X | FE_TOUCH_AXIS_Y | FE_TOUCH_AXIS_Z)) & ssimu.sensor_status) ^ (sstat & (FE_TOUCH_AXIS_X | FE_TOUCH_AXIS_Y | FE_TOUCH_AXIS_Z)) )
    {
        if ( (ssimu.evmask & FE_EV_ENDPOINT) &&
             ((ssimu.events & FE_EV_ENDPOINT) == 0 ) )
            ssimu.flag = true;                  // if event mask was active - set the flag
        ssimu.events |= FE_EV_ENDPOINT;
    }
    // check for a axis
    if ( ((sstat & FE_TOUCH_AXIS_A) & ssimu.sensor_status) ^ (sstat & FE_TOUCH_AXIS_A) )
    {
        if ( (ssimu.evmask & FE_EV_A_AXIS) &&
             ((ssimu.events & FE_EV_A_AXIS) == 0 ) )
            ssimu.flag = true;                  // if event mask was active - set the flag
        ssimu.events |= FE_EV_A_AXIS;
    }
    // check for probe
    if ( ((sstat & FE_TOUCH_PROBE) & ssimu.sensor_status) ^ (sstat & FE_TOUCH_PROBE) )
    {
        if ( (ssimu.evmask & FE_EV_PROBE) &&
             ((ssimu.events & FE_EV_PROBE) == 0 ) )
            ssimu.flag = true;                  // if event mask was active - set the flag
        ssimu.events |= FE_EV_PROBE;
    }
    ssimu.sensor_status = sstat;

    // simulate spindle setup
    if ( ssimu.spindle_crt_speed != ssimu.spindle_set_speed )
    {
        static int subdiv = 0;
        if (subdiv >= 4)
        {
            if ( ssimu.spindle_crt_speed < ssimu.spindle_set_speed )
                ssimu.spindle_crt_speed++;
            else
                ssimu.spindle_crt_speed--;
            if ( ssimu.spindle_crt_speed == ssimu.spindle_set_speed )
            {
                if ( ((ssimu.events & FE_EV_SPINDLE_OK) == 0) && (ssimu.evmask & FE_EV_SPINDLE_OK ) )
                    ssimu.flag = true;
                ssimu.events |= FE_EV_SPINDLE_OK;
            }
            subdiv = 0;
        }
        else
            subdiv++;
    }


    if ( (ssimu.cmd_in_run == 0) || (ssimu.cmd_finished) )
        return;

    ssimu.poll_ctr++;

    // make message delay simulation
    if ( ssimu.resp_time )
        ssimu.resp_time--;
    else
    {
        ssimu.cmd_finished = true;
        switch ( ssimu.cmd_in_run )
        {
            case SSIMU_CMD_SPINDLE_PWR:
                ui->cb_fe_pwr->setChecked( ssimu.spindle_on );
                ssimu.resp_data[0] = 0x50;  // acknowledge
                ssimu.resp_len = 1;
                break;
            case SSIMU_CMD_GET_EVENTS:
                ssimu.resp_data[0] = 0x53;
                ssimu.resp_data[1] = ssimu.events;
                ssimu.resp_data[2] = ssimu.sensor_status;
                ssimu.resp_len = 3;
                ssimu.events  &= ~( FE_EV_PROBE | FE_EV_ENDPOINT | FE_EV_A_AXIS | FE_EV_SPINDLE_OK );
                ssimu.flag = 0;
                break;
            case SSIMU_CMD_SET_EVMASK:
                ssimu.resp_data[0] = 0x50;  // acknowledge
                ssimu.resp_len = 1;
                break;
            case SSIMU_CMD_SET_SPEED:
                ssimu.events  &= ~( FE_EV_SPINDLE_JAM | FE_EV_SPINDLE_OK );
                ssimu.resp_data[0] = 0x50;  // acknowledge
                ssimu.resp_len = 1;
                break;
            case SSIMU_CMD_GET_RPM:
                {
                    uint32 rpm = ssimu.spindle_crt_speed;
                    // rpm -> interval:   rps = rpm / 60,  ivs = 1/rps,  ivs4us = ivs * 250kHz =>  ivs4us =250kHz * 60 / rpm
                    if ( rpm )
                        rpm = 250000*60 / rpm;

                    ssimu.resp_data[0] = 0x53;
                    ssimu.resp_data[1] = (uint8)(rpm & 0xff);
                    ssimu.resp_data[2] = (uint8)(rpm >> 8) & 0xff;
                    ssimu.resp_len = 3;
                }
                break;
            case SSIMU_CMD_GET_COORD:
                {
                    struct SStepCoordinates coords;
                    uint8 sign[3];
                    uint8 i;

                    coords.coord[0] = hw_coords.coord[COORD_X];
                    coords.coord[1] = hw_coords.coord[COORD_Y];
                    coords.coord[2] = hw_coords.coord[COORD_Z];
                    for ( i=0; i<3; i++ )
                    {
                        if ( coords.coord[i] < 0 )
                        {
                            sign[i] = 1;
                            coords.coord[i] = 0 - coords.coord[i];
                        }
                        else
                            sign[i] = 0;
                    }

                    i=0;
                    ssimu.resp_data[i++] = 0x59;
                    // X:  [xxxx xxxx][xxxx xxxx][xxxx 0000]    high bit to low bit
                    ssimu.resp_data[i++] = ((coords.coord[0] >> 12) & 0xff);
                    ssimu.resp_data[i++] = ((coords.coord[0] >> 4)  & 0xff);
                    // Y:  [0000 yyyy][yyyy yyyy][yyyy yyyy]
                    ssimu.resp_data[i++] = ((coords.coord[0] << 4)  & 0xf0) | ((coords.coord[1] >> 16) & 0x0f);
                    ssimu.resp_data[i++] = ((coords.coord[1] >> 8) & 0xff);
                    ssimu.resp_data[i++] = ((coords.coord[1] >> 0) & 0xff);
                    // Z:  [zzzz zzzz][zzzz zzzz][zzzz 0sss]
                    ssimu.resp_data[i++] = ((coords.coord[2] >> 12) & 0xff);
                    ssimu.resp_data[i++] = ((coords.coord[2] >> 4)  & 0xff);
                    ssimu.resp_data[i] =   ((coords.coord[2] << 4)  & 0xf0);

                    if ( sign[0] )
                        ssimu.resp_data[i] |= 0x04;
                    if ( sign[1] )
                        ssimu.resp_data[i] |= 0x02;
                    if ( sign[2] )
                        ssimu.resp_data[i] |= 0x01;

                    ssimu.resp_len = 9;
                }
                break;
            case SSIMU_CMD_RESET_COORD:
                ssimu.resp_data[0] = 0x50;  // acknowledge
                ssimu.resp_len = 1;

                hw_coords.coord[COORD_X] = CNC_DEFAULT_X;
                hw_coords.coord[COORD_Y] = CNC_DEFAULT_Y;
                hw_coords.coord[COORD_Z] = CNC_DEFAULT_Z;
                dispsim_add_point();
                break;
        }
    }
}



void fesimu_cmd_set_spindle_pwr(int val)
{
    ssimu.cmd_in_run = SSIMU_CMD_SPINDLE_PWR;
    ssimu.resp_time = 4 + 1;
    ssimu.poll_ctr = 0;
    if ( val & 0x40 )
        ssimu.spindle_on = true;
    if ( val & 0x04 )
        ssimu.spindle_on = false;
}

void fesimu_cmd_get_events(void)
{
    ssimu.cmd_in_run = SSIMU_CMD_GET_EVENTS;
    ssimu.resp_time = 2 + 4;
    ssimu.poll_ctr = 0;
}

void fesimu_cmd_set_evmask(int val)
{
    ssimu.cmd_in_run = SSIMU_CMD_SET_EVMASK;
    ssimu.resp_time = 4 + 1;
    ssimu.poll_ctr = 0;
    ssimu.evmask = val;
}

void fesimu_cmd_set_spindle_speed(int val)
{
    ssimu.cmd_in_run = SSIMU_CMD_SET_SPEED;
    ssimu.resp_time = 4 + 1;
    ssimu.poll_ctr = 0;
    ssimu.spindle_set_speed = val << 8;
}

void fesimu_cmd_get_rpm(void)
{
    ssimu.cmd_in_run = SSIMU_CMD_GET_RPM;
    ssimu.resp_time = 3 + 4;
    ssimu.poll_ctr = 0;
}

void fesimu_cmd_get_coord(void)
{
    ssimu.cmd_in_run = SSIMU_CMD_GET_COORD;
    ssimu.resp_time = 2 + 10;
    ssimu.poll_ctr = 0;
}

void fesimu_cmd_reset_coord(void)
{
    ssimu.cmd_in_run = SSIMU_CMD_RESET_COORD;
    ssimu.resp_time = 3 + 1;
    ssimu.poll_ctr = 0;
}


uint32 commfe_init()
{
    memset( &ssimu, 0, sizeof(ssimu));
    return 0;
}

uint32 commfe_sendCommand( uint8 *fecmd, uint32 cmd_len )
{
    if ( pClass->HW_wrp_fe_broken_link() )
        return 0;

    if ( (memcmp( fecmd, cmd_spindle, 2) == 0) && (cmd_len == 3) )
        fesimu_cmd_set_spindle_pwr( fecmd[2] );
    else if ( (memcmp( fecmd, cmd_getevents, 2) == 0) && (cmd_len == 2) )
        fesimu_cmd_get_events();
    else if ( (memcmp( fecmd, cmd_setevmask, 2) == 0) && (cmd_len == 3) )
        fesimu_cmd_set_evmask( fecmd[2] );
    else if ( (memcmp( fecmd, cmd_setspeed, 2) == 0) && (cmd_len == 3) )
        fesimu_cmd_set_spindle_speed( fecmd[2] );
    else if ( (memcmp( fecmd, cmd_getrpm, 2) == 0) && (cmd_len == 2) )
        fesimu_cmd_get_rpm();
    else if ( (memcmp( fecmd, cmd_getcoord, 2) == 0) && (cmd_len == 2) )
        fesimu_cmd_get_coord();
    else if ( (memcmp( fecmd, cmd_resetcoord, 2) == 0) && (cmd_len == 2) )
        fesimu_cmd_reset_coord();

    return 0;
}

uint32 commfe_checkResponse( uint8 resp_len )
{
    if ( ssimu.cmd_finished == false )
        return COMMFE_PENDING;

    ssimu.cmd_finished = false;
    ssimu.cmd_in_run = false;

    if ( resp_len != ssimu.resp_len )
        return COMMFE_PENDING;

    return COMMFE_OK;
}

uint32 commfe_getResponse( uint8 *fecmd, uint32 resp_len )
{
    if ( ssimu.resp_len == 0 )
        return 1;

    memcpy( fecmd, ssimu.resp_data, resp_len );
    resp_len = 0;
    return 0;
}

uint32 commfe_flushResponse( void )
{
    ssimu.resp_len = 0;
    return 0;
}

uint32 commfe_is_outfifo_empty( void )
{
    if ( ssimu.cmd_in_run && (ssimu.poll_ctr >= 2) )
        return 1;
    return 0;
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

    if ( (hw_coords.coord[COORD_X] > 130*400) || (hw_coords.coord[COORD_X]<0) ||
         (hw_coords.coord[COORD_Y] > 46*400) || (hw_coords.coord[COORD_Y]<0) )
        return;

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

    if ( (hw_coords.coord[COORD_Z]>=0) && (hw_coords.coord[COORD_Z]<=80*400) )
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

        ui->nm_seqctr->setValue( seq_ctr );

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

            ui->nm_rotor_pwr_x->setValue( accel.pwr[0] );
            ui->nm_rotor_pwr_y->setValue( accel.pwr[1] );
            ui->nm_rotor_pwr_z->setValue( accel.pwr[2] );
            ui->nm_rotor_pwr_a->setValue( accel.pwr[3] );
            ui->nm_ticks->setValue( accel.segment_ticks );

            accel.dirty = false;
        }

        ui->cb_fe_event_flag->setChecked( ssimu.flag );
        ui->nm_fe_spindle->setValue( ssimu.spindle_crt_speed );
    }

}
