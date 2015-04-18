#include <QScrollBar>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "mainw.h"
#include "ui_mainw.h"
#include "motion_core.h"
#include "cnc_defs.h"
#include "hw_stuff.h"
#include "comm_fe.h"
#include "comm_cmd.h"
#include "command_if.h"
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
// Master unit comm. simulation
/////////////////////////////////////////////////////

#define SIMU_MAX_RX_BUFF_SIZE   1024


struct SCommFifo
{
    uint8 buff[ SIMU_MAX_RX_BUFF_SIZE ];
    int r;
    int w;
    int c;

};


struct SSimuCommRx
{
    struct SCommFifo simu_feed;
    struct SCommFifo rx;
    uint8 resp[SIMU_MAX_RX_BUFF_SIZE];
    int resp_ct;
} comm_rx = {0, };


int fifo_push( struct SCommFifo *fifo, uint8 data )
{
    if ( fifo->c >= SIMU_MAX_RX_BUFF_SIZE )
        return -1;
    fifo->buff[ fifo->w++ ]= data;
    fifo->w %= SIMU_MAX_RX_BUFF_SIZE;
    fifo->c ++;
    return 0;
}

int fifo_push_bulk( struct SCommFifo *fifo, uint8 *data, int size )
{
    int size_1 = size;

    if ( fifo->c + size >= SIMU_MAX_RX_BUFF_SIZE )
        return -1;

    fifo->c += size;
    if ( size + fifo->w > SIMU_MAX_RX_BUFF_SIZE )
        size_1 = SIMU_MAX_RX_BUFF_SIZE - fifo->w;
    size -= size_1;
    
    memcpy( fifo->buff + fifo->w, data, size_1 );
    memcpy( fifo->buff, data + size_1, size );

    fifo->w += size_1 + size;
    fifo->w %= SIMU_MAX_RX_BUFF_SIZE;
    return 0;
}

uint8 fifo_pull( struct SCommFifo *fifo )
{
    uint8 data;
    if ( fifo->c == 0 )
        return -1;
    data = fifo->buff[ fifo->r++ ];
    fifo->r %= SIMU_MAX_RX_BUFF_SIZE;
    fifo->c --;
    return data;

}

void fifo_flush( struct SCommFifo *fifo )
{
    fifo->c = 0;
    fifo->r = 0;
    fifo->w = 0;
}

int fifo_free_space( struct SCommFifo *fifo )
{
    return (SIMU_MAX_RX_BUFF_SIZE - fifo->c);
}

int fifo_data_size( struct SCommFifo *fifo )
{
    return fifo->c;
}



static bool ovf = false;

uint32 comm_cmd_poll(void)
{
    if ( (fifo_free_space( &comm_rx.rx ) == 0) && (ovf == false) ) 
    {
        ovf = true;
        return COMFLAG_OVERFLOW;
    }
    if ( fifo_free_space( &comm_rx.rx ) )
    {
        ovf = false;
    }

    return COMFLAG_OK;
}

void comm_cmd_InFlush(void)
{
    fifo_flush( &comm_rx.rx );
    ovf = false;
}

uint32 comm_cmd_GetInLength(void)
{
    return fifo_data_size( &comm_rx.rx );
}

uint32 comm_cmd_GetOutFree(void)
{
    return SIMU_MAX_RX_BUFF_SIZE - comm_rx.resp_ct;
}

uint32 comm_rdChar(void)
{
    uint8 data;
    if ( fifo_data_size( &comm_rx.rx ) == 0 )
        return -1;
    data = fifo_pull( &comm_rx.rx );
    return data;
}

uint32 comm_wrChar( uint8 byte )
{
    if ( comm_rx.resp_ct >= SIMU_MAX_RX_BUFF_SIZE )
        return -1;
    comm_rx.resp[ comm_rx.resp_ct++ ] = byte;
    return 0;
}




/*  Parsing commands:
 *  '#' - marks comments
 *  command elements are in '[ ]'
 *
 *  [H] - header byte
 *  [C_xxx] - command indicator, where xxx is the abreviated command ID, see list below
 *  [P_xxx] - payload. Payload elements are separated by ','. notation for elemets is done by <...>
 *  [S] - checksum
 *                                     C_xxx        P_xxx
 *  command list - ( xxx ):
 *    - CMD_OBSA_RESET:                 "rest"
 *    - CMD_OBSA_SETUP_MAX_TRAVEL:      "stmt"      <x>,<y>,<z>,<a>                 ex:   [H][C_stmt][P_1200,3400,600,800][S]
 *    - CMD_OBSA_SETUP_MAX_SPEEDS:      "stms"      <max>,<rapid>
 *    - CMD_OBSA_SETUP_HOME_POZ:        "sthm"      <x>,<y>,<z>,<a>
 *    - CMD_OBSA_SETUP_PROBE_POZ:       "stpp"      <x>,<y>
 *    - CMD_OBSA_FIND_ORIGIN            "forg"
 *    - CMD_OBSA_GO_HOME                "home"
 *    - CMD_OBSA_FIND_Z_ZERO            "fzzr"
 *    - CMD_OBSA_STEP                   "step"      <coord list><dir>        coord list can be like "xya", dir is "+--"  ->  [H][C_step][P_xya,+--][S]
 *    - CMD_OBSA_START                  "STRT"
 *    - CMD_OB_PAUSE                    "PAUS"
 *    - CMD_OB_STOP                     "STOP"
 *    - CMD_OB_SCALE_FEED               "scfd"      <+/-factor>                     ex:   [H][C_scfd][P_-50][S]
 *    - CMD_OB_SCALE_SPINDLE            "scsp"      <+/-factor>
 *    - CMD_OB_GET_CRT_COORD            "gcrd"      <1/0>                    if P_xxx is missing then polling command will be used
 *    - CMD_OB_GET_CRT_CMD_ID           "gcmd"
 *    - CMD_OB_GET_STATUS               "gsts"
 *    - CMD_OB_GET_PROBE_TOUCH          "gthc"
 *    - CMD_IB_BULK                     "iblk"                               this should be followed by a command+param list
 *    - CMD_IB_SPINDLE                  "isps"      <id><rpm>
 *    - CMD_IB_WAIT                     "iwai"      <id><sec>
 *    - CMD_IB_GOTO                     "igoo"      <id><coord list+feed><p1><p2>...<feed>          coord list: "xyzaf", feed is always the last
 *    - CMD_IB_DRILL                    "idrl"      <id><x><y><ztop><zbtm><cyc><feed><clrtop>
 *
 *  Example for bulk:
 *      [H][C_iblk][C_isps][P_1,20000][C_igoo][P_2,xyf,1000,500,350][C_igoo][P_3,z,800][S]
 *
 *
 */


int local_tokenize_msg( char *str, char *token )
{
    int i = 0;
    int ptr = 0;
    // find the firs char
    while ( (str[0] == ' ') || (str[0] == '\t') )
    {
        ptr++;
        str++;
        if ( str[0] == 0 )
            return -1;      // no token found
    }

    // find the end of the token
    while ( str[i] != ']' )
    {
        if ( str[i] == 0 )  // invalid token
            return -1;      
        token[i] = str[i];
        i++;
        ptr++;
    }
    token[i] = 0;
    return ptr + 1;
}


int local_parse_max_travel( char *chunk, uint8 *outdata, int *out_ptr )
{
    int values[4];
    int i;
    int ptr;
    char *token;

    ptr = *out_ptr;

    token = strtok( chunk, " ," );
    for ( i=0; i<4; i++ )
    {
        if ( token == NULL )
            return -1;
        values[i] = atoi(token);
        token = strtok( NULL, " ," );
    }

    // X:  [xxxx xxxx][xxxx xxxx][xxxx 0000]    high bit to low bit
    outdata[ptr++] = ((values[0] >> 12) & 0xff);
    outdata[ptr++] = ((values[0] >> 4)  & 0xff);
    // Y:  [0000 yyyy][yyyy yyyy][yyyy yyyy]   
    outdata[ptr++] = ((values[0] << 4)  & 0xf0) | ((values[1] >> 16) & 0x0f);
    outdata[ptr++] = ((values[1] >> 8) & 0xff);
    outdata[ptr++] = ((values[1] >> 0) & 0xff);
    // Z:  [zzzz zzzz][zzzz zzzz][zzzz 0000]
    outdata[ptr++] = ((values[2] >> 12) & 0xff);
    outdata[ptr++] = ((values[2] >> 4)  & 0xff);
    // A:  [0000 aaaa][aaaa aaaa][aaaa aaaa]   
    outdata[ptr++] = ((values[2] << 4)  & 0xf0) | ((values[3] >> 16) & 0x0f);
    outdata[ptr++] = ((values[3] >> 8) & 0xff);
    outdata[ptr++] = ((values[3] >> 0) & 0xff);

    *out_ptr = ptr;
    return 0;
}

int local_parse_max_speed( char *chunk, uint8 *outdata, int *out_ptr )
{
    int values[2];
    int i;
    int ptr;
    char *token;

    ptr = *out_ptr;

    token = strtok( chunk, " ," );
    for ( i=0; i<2; i++ )
    {
        if ( token == NULL )
            return -1;
        values[i] = atoi(token);
        token = strtok( NULL, " ," );

        outdata[ptr++] = ((values[i] >> 8) & 0xff);
        outdata[ptr++] = ((values[i] >> 0)  & 0xff);
    }

    *out_ptr = ptr;
    return 0;
}

int local_parse_probe_poz( char *chunk, uint8 *outdata, int *out_ptr )
{
    int values[2];
    int i;
    int ptr;
    char *token;

    ptr = *out_ptr;

    token = strtok( chunk, " ," );
    for ( i=0; i<2; i++ )
    {
        if ( token == NULL )
            return -1;
        values[i] = atoi(token);
        token = strtok( NULL, " ," );
    }

    // X:  [xxxx xxxx][xxxx xxxx][xxxx 0000]    high bit to low bit
    outdata[ptr++] = ((values[0] >> 12) & 0xff);
    outdata[ptr++] = ((values[0] >> 4)  & 0xff);
    // Y:  [0000 yyyy][yyyy yyyy][yyyy yyyy]   
    outdata[ptr++] = ((values[0] << 4)  & 0xf0) | ((values[1] >> 16) & 0x0f);
    outdata[ptr++] = ((values[1] >> 8) & 0xff);
    outdata[ptr++] = ((values[1] >> 0) & 0xff);

    *out_ptr = ptr;
    return 0;
}


int local_parse_step( char *chunk, uint8 *outdata, int *out_ptr )
{
    int i;
    int ptr;
    char *token[2];
    uint8 field = 0;

    ptr = *out_ptr;

    token[0] = strtok( chunk, " ," );
    token[1] = strtok( NULL, " ," );
    if ( (token[0] == NULL) || (token[1] == NULL) || (strlen(token[0]) != strlen(token[1])) )
        return -1;

    for (i=0; i<strlen(token[0]); i++ )
    {
        switch ( token[0][i] )
        {
            case 'x':   field |= 0x10;  if ( token[1][i] == '+' )   field |= 0x01;  break;
            case 'y':   field |= 0x20;  if ( token[1][i] == '+' )   field |= 0x02;  break;
            case 'z':   field |= 0x40;  if ( token[1][i] == '+' )   field |= 0x04;  break;
            case 'a':   field |= 0x80;  if ( token[1][i] == '+' )   field |= 0x08;  break;
            default:    return -1;
        }
    }

    outdata[ptr++] = field;
    *out_ptr = ptr;
    return 0;
}

int local_parse_freerun( char *chunk, uint8 *outdata, int *out_ptr )
{
    // [P_x,+,1,300]
    int i;
    int j;
    int ptr;
    char *token[4];
    uint8 field = 0;
    bool no_limit = false;

    ptr = *out_ptr;

    token[0] = strtok( chunk, " ," );       // axis selector
    token[1] = strtok( NULL, " ," );        // direction
    token[2] = strtok( NULL, " ," );        // unlimitted
    token[3] = strtok( NULL, " ," );        // feed rate
    if ( (token[0] == NULL) || (token[1] == NULL) || (token[2] == NULL) || (token[3] == NULL) )
        return -1;
    switch ( token[0][0] )
    {
        case 'x': outdata[ptr] = 0x00; break;
        case 'y': outdata[ptr] = 0x40; break;
        case 'z': outdata[ptr] = 0x80; break;
        case 'a': outdata[ptr] = 0xC0; break;
        default:
            return -1;
    }
    outdata[ptr] |= ( token[2][0] == '1' ) ? 0x20 : 0x00;
    outdata[ptr] |= ( token[1][0] == '+' ) ? 0x10 : 0x00;
    outdata[ptr++] |= ( atoi(token[3]) >> 8 ) & 0x0f;
    outdata[ptr++] = atoi(token[3]) & 0xff;
    *out_ptr = ptr;
    return 0;
}

int local_parse_scale( char *chunk, uint8 *outdata, int *out_ptr )
{
    int ptr;
    char *token;
    int16 *val_ptr;

    ptr = *out_ptr;
    token = strtok( chunk, " ," );

    val_ptr = (int16*)(outdata + ptr);
    *val_ptr = atoi(token);
    ptr += 2;

    *out_ptr = ptr;
    return 0;
}

int local_parse_ctr_coord( char *chunk, uint8 *outdata, int *out_ptr )
{
    int ptr;
    char *token;
    int16 *val_ptr;

    ptr = *out_ptr;
    token = strtok( chunk, " ," );

    if ( token[0] == '1' )
        outdata[ptr++] = 0x01;
    else
        outdata[ptr++] = 0x00;

    *out_ptr = ptr;
    return 0;
}

int local_parse_ib_spindle( char *chunk, uint8 *outdata, int *out_ptr )
{
    int i;
    int ptr;
    char *token[2];

    ptr = *out_ptr;

    token[0] = strtok( chunk, " ," );
    token[1] = strtok( NULL, " ," );
    if ( (token[0] == NULL) || (token[1] == NULL) )
        return -1;

    outdata[ptr++] = atoi(token[0]);
    outdata[ptr++] = (atoi(token[1]) >> 8) & 0xff;
    outdata[ptr++] = atoi(token[1]) & 0xff;

    *out_ptr = ptr;
    return 0;
}

int local_parse_ib_wait( char *chunk, uint8 *outdata, int *out_ptr )
{
    int i;
    int ptr;
    char *token[2];

    ptr = *out_ptr;

    token[0] = strtok( chunk, " ," );
    token[1] = strtok( NULL, " ," );
    if ( (token[0] == NULL) || (token[1] == NULL) )
        return -1;

    outdata[ptr++] = atoi(token[0]);
    outdata[ptr++] = atoi(token[1]) & 0xff;

    *out_ptr = ptr;
    return 0;
}

int local_parse_ib_goto( char *chunk, uint8 *outdata, int *out_ptr )
{
    int i;
    int ptr;
    char *token;
    uint8 fields = 0;
    int val;
    int nr_arg = 0;
    int digit = 0;

    ptr = *out_ptr;

    // get cmd ID
    token = strtok( chunk, " ," );
    if ( token == NULL )
        return -1;
    outdata[ptr++] = atoi(token);       // cmdID

    // get fields
    token = strtok( NULL, " ," );
    if ( token == NULL )
        return -1;
    for (i=0; i<strlen(token); i++ )
    {
        switch ( token[i] )
        {
            case 'x':   fields |= 0x01; break;
            case 'y':   fields |= 0x02; break;
            case 'z':   fields |= 0x04; break;
            case 'a':   fields |= 0x08; break;
            case 'f':   fields |= 0x10; break;
            default:    return -1;
        }
        nr_arg++;
    }
    outdata[ptr++] = fields;

    // get arguments
    for (i=0; i<nr_arg; i++)
    {
        token = strtok( NULL, " ," );
        if ( token == NULL )
            return -1;
        val = atoi(token);

        if ( (fields & 0x10) && (i == (nr_arg -1) ))    // last argument, and it is the feed rate
            break;

        if ( (digit & 0x01) )
        {
            outdata[ptr++] |= ((val >> 16) & 0x0f);
            outdata[ptr++] = ((val >> 8) & 0xff);
            outdata[ptr++] = ((val >> 0) & 0xff);
        }
        else
        {
            outdata[ptr++] = ((val >> 12) & 0xff);
            outdata[ptr++] = ((val >> 4)  & 0xff);
            outdata[ptr]   = ((val << 4)  & 0xf0);
        }

        digit++;
        val = -1;
    }

    if ( (digit & 0x01) )
        ptr++;

    if ( val != -1 )        // means that the loop before broken because of feed rate parameter
    {
        outdata[ptr++] = ((val >> 8) & 0xff);
        outdata[ptr++] = ((val >> 0) & 0xff);
    }

    *out_ptr = ptr;
    return 0;
}


int local_parse_ib_drill( char *chunk, uint8 *outdata, int *out_ptr )
{
    int i;
    int ptr;
    char *token;
    uint8 fields = 0;
    int values[15];
    int nr_arg = 0;
    int digit = 0;

    ptr = *out_ptr;

    // get cmd ID
    token = strtok( chunk, " ," );
    if ( token == NULL )
        return -1;
    outdata[ptr++] = atoi(token);       // cmdID

    // get parameters
    token = strtok( NULL, " ," );
    for ( i=0; i<7; i++ )
    {
        if ( token == NULL )
            return -1;
        values[i] = atoi(token);
        token = strtok( NULL, " ," );
    }

    // x:  [xxxx xxxx][xxxx xxxx][xxxx 0000]    high bit to low bit
    outdata[ptr++] = ((values[0] >> 12) & 0xff);
    outdata[ptr++] = ((values[0] >> 4)  & 0xff);
    // y:  [0000 yyyy][yyyy yyyy][yyyy yyyy]   
    outdata[ptr++] = ((values[0] << 4)  & 0xf0) | ((values[1] >> 16) & 0x0f);
    outdata[ptr++] = ((values[1] >> 8) & 0xff);
    outdata[ptr++] = ((values[1] >> 0) & 0xff);
    // z:  [zzzz zzzz][zzzz zzzz][zzzz 0000]
    outdata[ptr++] = ((values[2] >> 12) & 0xff);
    outdata[ptr++] = ((values[2] >> 4)  & 0xff);
    // Z:  [0000 ZZZZ][ZZZZ ZZZZ][ZZZZ ZZZZ]   
    outdata[ptr++] = ((values[2] << 4)  & 0xf0) | ((values[3] >> 16) & 0x0f);
    outdata[ptr++] = ((values[3] >> 8) & 0xff);
    outdata[ptr++] = ((values[3] >> 0) & 0xff);
    // n:
    outdata[ptr++] = ((values[4] >> 0) & 0xff);
    // f:   [ffff ffff][ffff 0000]
    outdata[ptr++] = ((values[5] >> 4) & 0xff);
    // c:  [0000 cccc][cccc cccc]
    outdata[ptr++] = ((values[5] << 4) & 0xf0) | ((values[6] >> 8) & 0x0f);
    outdata[ptr++] = ((values[6] >> 0) & 0xff);

    *out_ptr = ptr;
    return 0;
}


int mainw::HW_wrp_insert_message( unsigned char *buffer, int size, bool from_master )
{
    QTextCharFormat format;
    QString msg;
    QTextCursor tc(doc_comm_log);
    bool move_to_bottom = false;
    QScrollBar *sb;
    int ptr = 0;
    char str[10];

    sb = ui->txt_comm_window->verticalScrollBar();
    if ( sb->value() == sb->maximum() )
        move_to_bottom = true;

    if ( from_master)
    {
        format.setForeground( Qt::red );
        msg.append( tr("\n") + tr("S> ") );
    }
    else
    {
        format.setForeground( Qt::darkGreen );
        msg.append( tr("\n") + tr("R< ") );
    }

    // insert the new message
    tc.movePosition( QTextCursor::End );
    tc.setCharFormat( format );

    for ( ptr = 0; ptr < size; ptr++ )
    {
        if ( ((ptr % 8) == 0) && (ptr != 0) && (ptr % 16) )
            sprintf( str, "| %02x ", buffer[ptr]);
        else
            sprintf( str, "%02x ", buffer[ptr]);

        if ( (ptr != 0) && ((ptr % 16) == 0) )
            msg.append( tr("\n") + tr("   ")  );
        msg.append( str );
    }

    tc.insertText( msg );

    // if scroll was at the end of log display then scroll to the end of the new line also
    if ( move_to_bottom )
        sb->triggerAction(QScrollBar::SliderToMaximum);

}


int mainw::HW_wrp_input_line(QString line)
{
    QByteArray b;
    char *indata;
    int in_ptr;
    uint8 outdata[256];
    int out_ptr = 0;
    uint8 cmd_type;
    char token[256];
    char *subtok;
    int cmd_poz = 0;
    int cmd_blk = 0;

    b = line.toLatin1();
    indata = b.data();

    in_ptr = local_tokenize_msg( indata, token );
    if ( in_ptr <=0 )
        return 0;

    while ( 1 )
    {
        if ( token[0] == '#' )
            return 0;
        if ( (token[0] != '[') || (strlen(token) < 2 ) )
            return -1;

        switch ( token[1] )
        {
            case 'H':
                outdata[out_ptr++] = 0xAA;
                break;
            case 'C':
                if ( strlen(token) != 7 )
                    return -1;

                if ( strncmp(token+3, "rest", 4 ) == 0 )
                    cmd_type = CMD_OB_RESET;
                else if ( strncmp(token+3, "stmt", 4 ) == 0 )
                    cmd_type = CMD_OBSA_SETUP_MAX_TRAVEL;
                else if ( strncmp(token+3, "stms", 4 ) == 0 )
                    cmd_type = CMD_OBSA_SETUP_MAX_SPEEDS;
                else if ( strncmp(token+3, "sthm", 4 ) == 0 )
                    cmd_type = CMD_OBSA_SETUP_HOME_POZ;
                else if ( strncmp(token+3, "stpp", 4 ) == 0 )
                    cmd_type = CMD_OBSA_SETUP_PROBE_POZ;
                else if ( strncmp(token+3, "forg", 4 ) == 0 )
                    cmd_type = CMD_OBSA_FIND_ORIGIN;
                else if ( strncmp(token+3, "home", 4 ) == 0 )
                    cmd_type = CMD_OBSA_GO_HOME;
                else if ( strncmp(token+3, "fzzr", 4 ) == 0 )
                    cmd_type = CMD_OBSA_FIND_Z_ZERO;
                else if ( strncmp(token+3, "step", 4 ) == 0 )
                    cmd_type = CMD_OBSA_STEP;
                else if ( strncmp(token+3, "frun", 4 ) == 0 )
                    cmd_type = CMD_OBSA_FREERUN;
                else if ( strncmp(token+3, "STRT", 4 ) == 0 )
                    cmd_type = CMD_OBSA_START;
                else if ( strncmp(token+3, "PAUS", 4 ) == 0 )
                    cmd_type = CMD_OB_PAUSE;
                else if ( strncmp(token+3, "STOP", 4 ) == 0 )
                    cmd_type = CMD_OB_STOP;
                else if ( strncmp(token+3, "scfd", 4 ) == 0 )
                    cmd_type = CMD_OB_SCALE_FEED;
                else if ( strncmp(token+3, "scsp", 4 ) == 0 )
                    cmd_type = CMD_OB_SCALE_SPINDLE;
                else if ( strncmp(token+3, "gcrd", 4 ) == 0 )
                    cmd_type = CMD_OB_GET_CRT_COORD;
                else if ( strncmp(token+3, "gcmd", 4 ) == 0 )
                    cmd_type = CMD_OB_GET_CRT_CMD_ID;
                else if ( strncmp(token+3, "gsts", 4 ) == 0 )
                    cmd_type = CMD_OB_GET_STATUS;
                else if ( strncmp(token+3, "gthc", 4 ) == 0 )
                    cmd_type = CMD_OB_GET_PROBE_TOUCH;
                else if ( strncmp(token+3, "iblk", 4 ) == 0 )
                    cmd_type = CMD_IB_BULK | 0x80;
                else if ( strncmp(token+3, "isps", 4 ) == 0 )
                    cmd_type = CMD_IB_SPINDLE;
                else if ( strncmp(token+3, "iwai", 4 ) == 0 )
                    cmd_type = CMD_IB_WAIT;
                else if ( strncmp(token+3, "igoo", 4 ) == 0 )
                    cmd_type = CMD_IB_GOTO;
                else if ( strncmp(token+3, "idrl", 4 ) == 0 )
                    cmd_type = CMD_IB_DRILL;
                else
                    return -1;

                cmd_poz = out_ptr;
                outdata[out_ptr++] = cmd_type;
                if ( cmd_type == (CMD_IB_BULK | 0x80) )
                {
                    cmd_blk = out_ptr;
                    out_ptr ++;
                }
                break;

            case 'P':
                {
                    int p_len = 0;

                    if ( token[2] != '_' )
                        return -1;
                    if ( strlen(token) < 4 )
                        return -1;
    
                    switch ( cmd_type )
                    {
                        case CMD_OBSA_SETUP_MAX_TRAVEL:                     // [H] [C_stmt][P_1000,2000,3000,4000] [S]   #test
                        case CMD_OBSA_SETUP_HOME_POZ:
                            if ( local_parse_max_travel( token+3, outdata + out_ptr + 1, &p_len ) )
                                return -1;
                            break;
                        case CMD_OBSA_SETUP_MAX_SPEEDS:                     // [H][C_stms][P_1500,900][S]
                            if ( local_parse_max_speed( token+3, outdata + out_ptr + 1, &p_len ) )
                                return -1;
                            break;
                        case CMD_OBSA_SETUP_PROBE_POZ:                      // [H][C_stpp][P_3000,4000][S]
                            if ( local_parse_probe_poz( token+3, outdata + out_ptr + 1, &p_len ) )
                                return -1;
                            break;
                        case CMD_OBSA_STEP:                                 // [H][C_step][P_xza,--+][S]
                            if ( local_parse_step( token+3, outdata + out_ptr + 1, &p_len ) )
                                return -1;
                            break;
                        case CMD_OBSA_FREERUN:                              // [H][C_frun][P_x,+,1,300][S]   or [H][C_frun][P_x,+,0,300][S]
                            if ( local_parse_freerun( token+3, outdata + out_ptr + 1, &p_len ) )
                                return -1;
                            break;
                        case CMD_OB_SCALE_FEED:                             // [H][C_scfd][P_-150][S]
                        case CMD_OB_SCALE_SPINDLE:                          // [H][C_scsp][P_120][S]
                            if ( local_parse_scale( token+3, outdata + out_ptr + 1, &p_len ) )
                                return -1;
                            break;
                        case CMD_OB_GET_CRT_COORD:                                                       // [H][C_gcrd][S]
                            if ( local_parse_ctr_coord( token+3, outdata + out_ptr + 1, &p_len ) )       // [H][C_gcrd][P_1][S]
                                return -1;
                            break;
                        case CMD_IB_SPINDLE:                                // [H][C_isps][P_13,20000][S]
                            if ( local_parse_ib_spindle( token+3, outdata + out_ptr + 1, &p_len ) )      
                                return -1;
                            break;
                        case CMD_IB_WAIT:                                   // [H][C_iwai][P_14,200][S]
                            if ( local_parse_ib_wait( token+3, outdata + out_ptr + 1, &p_len ) ) 
                                return -1;
                            break;
                        case CMD_IB_GOTO:                                   // [H][C_igoo][P_1,xyzf,8,9,10,11][S]
                            if ( local_parse_ib_goto( token+3, outdata + out_ptr + 1, &p_len ) )     
                                return -1;
                            break;
                        case CMD_IB_DRILL:                                  // [H][C_idrl][P_15,1000,2000,3000,3200,5,350,32][S]
                            if ( local_parse_ib_drill( token+3, outdata + out_ptr + 1, &p_len ) ) 
                                return -1;
                            break;
                    }

                    if ( cmd_poz )
                        outdata[cmd_poz] |= 0x80;
                    outdata[out_ptr] = p_len;
                    out_ptr += p_len + 1;
                }
                break;
            case 'S':
                {
                    uint8 cksum;
                    int i;

                    if ( cmd_blk )
                    {
                        outdata[cmd_blk]= out_ptr - cmd_blk - 1;
                    }

                    cksum = (uint8)(COMMCKSUMSTART);
                    for (i=0; i<out_ptr; i++)
                    {
                        cksum += outdata[i];
                    }
                    outdata[out_ptr++] = cksum;
                }
                break;
        }

        indata += in_ptr;
        in_ptr = local_tokenize_msg( indata, token );
        if ( in_ptr <=0 )
            break;
    }

    // outdata, out_ptr - are ready to be used
    if ( fifo_free_space( &comm_rx.simu_feed ) < out_ptr )
        return -2;
    fifo_push_bulk( &comm_rx.simu_feed, outdata, out_ptr );

    // insert the message
    HW_wrp_insert_message( outdata, out_ptr, true );
    return 0;
}


int mainw::HW_wrp_simu_datafeed()
{
    // simulate serial transfer
    if ( fifo_free_space( &comm_rx.rx ) && fifo_data_size( &comm_rx.simu_feed ) )
    {
        uint8 data;
        data = fifo_pull( &comm_rx.simu_feed );
        fifo_push( &comm_rx.rx, data );
    }

    // get response
    if ( comm_rx.resp_ct )
    {
        HW_wrp_insert_message( comm_rx.resp, comm_rx.resp_ct, false );
        comm_rx.resp_ct = 0;
    }

    return 0;
}



/////////////////////////////////////////////////////
// front end simulation
/////////////////////////////////////////////////////

const uint8 cmd_spindle[] =  { 0xA9, 0x14 };
const uint8 cmd_getevents[] = { 0xAA, 0xA9 };                // has no cksum
const uint8 cmd_setevmask[] = { 0xA9, 0x13 };
const uint8 cmd_setspeed[] = { 0xA9, 0x11 };
const uint8 cmd_getrpm[] = { 0xA9, 0x00 };
const uint8 cmd_getcoord[] = { 0xAA, 0x9A };                 // no checksum here also
const uint8 cmd_resetcoord[] = { 0xA9, 0x05 };

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

void HW_Wait_Reset(void)
{

}

void StepDBG_QT_innerLoop()
{
    StepTimerIntrHandler();
    pClass->HW_wrp_front_end_simu();
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

        {
            // front_end events
            QString str= tr("");
            if ( ssimu.events & FE_EV_A_AXIS ) str += tr("a "); else str += tr("- ");
            if ( ssimu.events & FE_EV_ENDPOINT ) str += tr("e "); else str += tr("- ");
            if ( ssimu.events & FE_EV_PROBE ) str += tr("p "); else str += tr("- ");
            if ( ssimu.events & FE_EV_SPINDLE_JAM ) str += tr("j "); else str += tr("- ");
            if ( ssimu.events & FE_EV_SPINDLE_OK ) str += tr("o "); else str += tr("- ");

            ui->ln_fe_eventlist->setText( str );
        }
    }

}
