#include <string.h>

#include "hw_stuff.h"
#include "cnc_sequencer.h"
#include "cnc_sequencer_internals.h"
#include "command_if.h"
#include "frontend.h"

/******************************************************************************
 *
 *             Sequence generator and main decision loop
 *     - this module controls the operation of the CNC driver -
 *
 *  Module is based on and controlling the submodules:
 *      - motion_core:  -step management with acceleration/deceleration and
 *                       sequence execution scheduler
 *      - frontend:     -physical coordinate readback, endpoint detector,
 *                       spindle control and monitoring
 *      - command_if:   -input command and communication interface with
 *                       the master. handles command decoding
 *
 *  The CNC sequencer is responsible of:
 *      - initting: motion_core, frontend, command_if
 *      - polling command_if for command input. Input fifo is a pull model.
 *      - managing command fifo (hold back and delete at completion)
 *      - executing commands
 *      - feeding motion_core's sequence fifo. (push model)
 *      - controlling and monitoring frontend
 *      - processing front panel events
 *
 *
 *  At start-up or reset:
 *      The cnc sequencer is initted with default max poz which is considered as max travel also.
 *
 *
 *
 *  Outband operation procedures:
 *      - See them described at each internal_outband_xxxx routine
 *
 *
 *  LED meanings:
 *       leds:        (R)   (G1)   (G2)
 *       operations:   O - constant on,   X - off,   B - blink slow,  b - blink fast     
 *  
 *       LED_SYSTEM
 *       (G2)  - X - system off
 *               B - system starts up (after reset)
 *               O - system OK
 *
 *       LED_PROCESSING
 *       (G1)  - X - standby, stopped
 *               B - program paused / waiting
 *               b - running a sequence
 * 
 *       LED_FAILURE     
 *       (R)   - X - no fault
 *               B - general fault (even for table and spindle stuck)
 *               b - intermitent fault - trying to repare it
 *               O - system crash ( BAD THING )
 *
 */


#define IB_SEQID_SPINDLE        0x80
#define IB_SEQID_GOBACK_XY      0x81        // don't change the value relation --v
#define IB_SEQID_GOBACK_Z       0x82        // don't change the value relation --^
#define IB_SEQID_REALIGN        0x83


struct SCNCSequencerInternals   cnc;


int internal_stop( bool leave_ob, bool stop_spindle );

static void internal_helper_set_power( int axis, uint32 feed )
{
    enum EPowerLevel pwr;
    if ( feed > SEQ_FEED_PWR_HIGH )
        pwr = mpwr_full;
    else if ( feed >= SEQ_FEED_PWR_MED )
        pwr = mpwr_high;
    else if ( feed >= SEQ_FEED_PWR_LOW )
        pwr = mpwr_med;
    else
        pwr = mpwr_low;

    motion_pwr_ctrl( axis, pwr );
}


void internal_fifo_flush(void)
{
    cnc.cmd_fifo.w = 0;
    cnc.cmd_fifo.r = 0;
    cnc.cmd_fifo.e = 0;
    cnc.cmd_fifo.wrtb = CNCSEQ_CMD_FIFO_SIZE;       // free space writeable:  fifosz - (readable + eraseable)
    cnc.cmd_fifo.rdb = 0;                           // readable data size:    fifosz - (writeable + eraseable)  eraseable means - allready read but in hold
}                                                   //     eraseable = fifosz - (readable + writeable)

#define SEQFIFO_ERASEABLE       ( CNCSEQ_CMD_FIFO_SIZE - ( cnc.cmd_fifo.rdb + cnc.cmd_fifo.wrtb ) )

// get a fillable pointer from the write pointer.
// internal_fifo_push() should be called to advance to the next
struct ScmdIfCommand *internal_fifo_get_fillable(void)
{
    if ( cnc.cmd_fifo.wrtb == 0 )
        return NULL;

    return &cnc.cmd_fifo.cmd[cnc.cmd_fifo.w];
}

struct ScmdIfCommand *internal_fifo_get_last_introduced(void)
{
    int w;
    if ( cnc.cmd_fifo.wrtb == CNCSEQ_CMD_FIFO_SIZE )
        return NULL;

    w = cnc.cmd_fifo.w;
    if ( w )
        w--;
    else
        w = CNCSEQ_CMD_FIFO_SIZE - 1;

    return &cnc.cmd_fifo.cmd[w];
}

// push a currectly edited command in the fifo
int internal_fifo_push( struct ScmdIfCommand *cmd )
{
    if ( cnc.cmd_fifo.wrtb == 0 )
        return -1;
    if ( cmd )
        cnc.cmd_fifo.cmd[cnc.cmd_fifo.w] = *cmd;
    cnc.cmd_fifo.w++;
    cnc.cmd_fifo.w %= CNCSEQ_CMD_FIFO_SIZE;
    cnc.cmd_fifo.wrtb--;
    cnc.cmd_fifo.rdb++;
    return 0;
}

// get the nr of elements available for pulling from the fifo
int internal_fifo_info_usable(void)
{
    return cnc.cmd_fifo.rdb;
}

// get the free space in the fifo
int internal_fifo_info_free(void)
{
    return cnc.cmd_fifo.wrtb;
}

// pull a readable element from the fifo
struct ScmdIfCommand *internal_fifo_pull( void )
{
    struct ScmdIfCommand *elem;

    if ( cnc.cmd_fifo.rdb == 0 )
        return NULL;

    elem = &cnc.cmd_fifo.cmd[ cnc.cmd_fifo.r++ ];
    cnc.cmd_fifo.r %= CNCSEQ_CMD_FIFO_SIZE;
    cnc.cmd_fifo.rdb--;
    return elem;
}

struct ScmdIfCommand *internal_fifo_peek_readable(void)
{
    if ( cnc.cmd_fifo.rdb == 0 )
        return NULL;
    return &cnc.cmd_fifo.cmd[ cnc.cmd_fifo.r ];
}

struct ScmdIfCommand *internal_fifo_peek_eraseable(void)
{
    if ( SEQFIFO_ERASEABLE == 0 )
        return NULL;
    return &cnc.cmd_fifo.cmd[ cnc.cmd_fifo.e ];
}

// erase an element
int internal_fifo_erase( void )
{
    if ( SEQFIFO_ERASEABLE == 0 )
        return -1;

    cnc.cmd_fifo.e++;
    cnc.cmd_fifo.e %= CNCSEQ_CMD_FIFO_SIZE;
    cnc.cmd_fifo.wrtb++;
    return 0;
}

void internal_fifo_move_read_to_eraser(void)
{
    cnc.cmd_fifo.r = cnc.cmd_fifo.e;
    cnc.cmd_fifo.rdb += SEQFIFO_ERASEABLE;
}




static int internal_procedure_coordinate_check_entry( void )
{
    if ( cnc.status.procedure.procID )
    {
        if ( cnc.status.procedure.procID == procid_getcoordinates )
            return 0;
        else
            return -1;
    }

    if ( front_end_request_coordinates() )
        return -1;

    cnc.status.procedure.procID = procid_getcoordinates;
    cnc.status.procedure.params.getcoord.coord_snapshot.coord[0] = CNC_COORD_UNINIT;
    cnc.status.procedure.params.getcoord.coord_fe.coord[0] = CNC_COORD_UNINIT;
    return 0;
}

static int internal_procedure_coordinate_check_poll( struct SEventStruct *evt )
{
    if ( cnc.status.procedure.procID != procid_getcoordinates )
        return 0;

    if ( (cnc.status.procedure.params.getcoord.coord_snapshot.coord[0] == CNC_COORD_UNINIT ) &&
         (front_end_is_coordinate_request_sent() ) )
    {
        motion_get_crt_coord( &cnc.status.procedure.params.getcoord.coord_snapshot );
    }
    // front end poll is done in the main sequencer poll loop - do not duplicate here
    if ( evt->fe_op_completed )
    {
        cnc.status.procedure.procID = procid_none;
        if ( evt->fe_op_failed )
        {
            cnc.status.flags.f.err_fatal = 1;
            cnc.status.flags.f.err_code = GENFAULT_FRONT_END;
            LED_Op( LED_FAILURE, LED_blink_slow );
            return -1;
        }
        else
            return 1;
    }
    return 0;
}

static int internal_procedure_coordinate_check_crt_coord( int max_deviation, bool save )
{
    int i;
    int diff;
    struct SStepCoordinates *fe_coord;

    fe_coord = front_end_get_coordinates();

    if ( save )
        cnc.status.procedure.params.getcoord.coord_fe = *fe_coord;

    for (i=0; i<CNC_MAX_COORDS; i++)
    {
        if ( cnc.status.procedure.params.getcoord.coord_snapshot.coord[i] > fe_coord->coord[i] )
            diff = cnc.status.procedure.params.getcoord.coord_snapshot.coord[i] - fe_coord->coord[i];
        else
            diff = fe_coord->coord[i] - cnc.status.procedure.params.getcoord.coord_snapshot.coord[i];
        if ( diff > max_deviation )
            return -1;
    }
    return 0;
}


static int internal_procedure_spindle_pwrdown_entry( bool full_pwrdown )
{
    if ( cnc.status.procedure.procID )
    {
        if ( cnc.status.procedure.procID == procid_spindle_pwrdown )
            return 0;
        else
            return -1;
    }

    if ( front_end_spindle_speed(0) )
        return -1;

    cnc.status.procedure.procID = procid_spindle_pwrdown;
    cnc.status.procedure.params.spindle_off = full_pwrdown ? 1 : 0;
    return 0;
}

static int internal_procedure_spindle_poll( struct SEventStruct *evt )
{
    if ( (evt->timer_tick_10ms) &&
         (cnc.status.procedure.procID == procid_spindle_pwrup) &&
         (cnc.status.procedure.params.spindle_off > 1 ) )
    {
        cnc.status.procedure.params.spindle_off--;
        if ( cnc.status.procedure.params.spindle_off == 1 )
        {
            cnc.status.procedure.params.spindle_off = 0;
            front_end_spindle_speed( cnc.status.misc.spindle_speed );   // 3sec is over, send the spindle speed
            cnc.status.flags.f.spindle_on = 1;                          // consider it on, even if it fails to be able to stop it
        }
    }

    if ( evt->fe_op_completed )
    {
        if ( evt->fe_op_failed )
        {
            if ( evt->fe_spindle_jam == 0 )
            {
                cnc.status.flags.f.err_fatal = 1;
                cnc.status.flags.f.err_code = GENFAULT_FRONT_END;
                LED_Op( LED_FAILURE, LED_blink_slow );
            }
            else
                cnc.status.flags.f.err_spjam = 1;

            cnc.status.procedure.procID = procid_none;
            return -1;
        }
        else
        {
            switch ( cnc.status.procedure.procID )
            {
                case procid_spindle_pwrdown:
                    cnc.status.flags.f.spindle_on = 0;
                    if ( cnc.status.procedure.params.spindle_off )      // if power needs to be turned off
                    {
                        cnc.status.procedure.params.spindle_off--;
                        front_end_spindle_power( false );
                        cnc.status.flags.f.spindle_pwr = 0;             // assume that it will succeed, otherwise gen-fault, needs reset
                        return 0;
                    }
                    else
                    {
                        cnc.status.procedure.procID = procid_none;      // else return success
                        return 1;
                    }
                    break;
                case procid_spindle_pwrup:
                    if ( cnc.status.procedure.params.spindle_off )      // spindle power up action is sent
                    {
                        cnc.status.flags.f.spindle_pwr = 1;
                        cnc.status.procedure.params.spindle_off = 300;  // now we have to wait 3sec. for the power regulator to start up till spindle speed can be set
                    }
                    else                                                // else the completion was for spindle speed command
                    {
                        cnc.status.procedure.procID = procid_none;      // return success
                        return 1;
                    }
                    break;
                default:
                    break;
            }
        }
    }
    return 0;
}


static int internal_procedure_spindle_control_entry( uint32 rpm )
{
    if ( cnc.status.procedure.procID )
    {
        return -1;
    }

    cnc.status.misc.spindle_speed = rpm;
    if ( rpm == 0 )
    {
        internal_procedure_spindle_pwrdown_entry(false);
        return 0;
    }

    if ( cnc.status.flags.f.spindle_pwr == 0 )              // if spindle was powered down
    {
        front_end_spindle_power( true );                    // power it up
        cnc.status.procedure.params.spindle_off = 1;        // instruct to power up the spindle unit
    }
    else
    {
        cnc.status.procedure.params.spindle_off = 0;
        front_end_spindle_speed( rpm );                     // else set the rpm directly
        cnc.status.flags.f.spindle_on = 1;                  // consider it on, even if it fails to be able to stop it
    }

    cnc.status.procedure.procID = procid_spindle_pwrup;
    return 0;
}




static void internal_send_simple_ack( struct ScmdIfCommand *cmd )
{
    struct ScmdIfResponse resp;
    resp.cmd_type = cmd->cmd_type;
    resp.resp_type = RESP_ACK;
    resp.resp.getCoord.has_data = 0;
    cmdif_confirm_reception(&resp);
}


static void internal_ob_helper_gohome_setsequence( int seq_start )
{
    struct SMotionSequence seq;
    motion_sequence_stop();     // paranoya

    seq.cmdID = 0;
    seq.seqID = seq_start;
    seq.seqType = SEQ_TYPE_GOTO;
    seq.params.go_to.feed = cnc.setup.feed_rapid;

    if ( seq_start == 1 )       // start from the beginning
    {
        motion_get_crt_coord( &seq.params.go_to.coord );
        seq.params.go_to.coord.coord[COORD_Z] = cnc.setup.home_poz.coord[COORD_Z];
        motion_sequence_insert( &seq );
    }
    seq.seqID = 2;
    seq.params.go_to.coord = cnc.setup.home_poz;
    motion_sequence_insert( &seq );
    motion_sequence_start();

    LED_Op( LED_PROCESSING, LED_blink_fast );
}

static void internal_ob_helper_forg_backoff_setup(uint32 ax_mask)
{
    int i;
    struct SMotionSequence seq;
    motion_sequence_stop();     // paranoya

    if ( ax_mask == 0 )
        return;

    // set up destination coordinate
    seq.cmdID = 0;
    seq.seqID = 1;
    seq.seqType = SEQ_TYPE_GOTO;
    seq.params.go_to.feed = 400;

    motion_get_crt_coord( &seq.params.go_to.coord );
    for (i=0; i<CNC_MAX_COORDS; i++)
    {
        if ( ax_mask & (1<<i) )
            seq.params.go_to.coord.coord[CNC_MAX_COORDS - i - 1] -= 450;        // coordinates are in inverse order at front-end
    }

    motion_sequence_insert( &seq );
    motion_sequence_start();
    LED_Op( LED_PROCESSING, LED_blink_fast );

    // set up front end for endpoint touch detection
    if ( front_end_check_op_busy() )
    {
        front_end_terminate_touch_detection();
        front_end_sync_event_loop();
    }
    front_end_request_touch_detection( ax_mask );
}

static void internal_ob_helper_forg_search_setup(uint32 ax_mask)
{
    int i;
    motion_sequence_stop();

    if ( front_end_check_op_busy() )
    {
        front_end_terminate_touch_detection();
        front_end_sync_event_loop();
    }
    front_end_request_touch_detection( ax_mask );

    for (i=0; i<CNC_MAX_COORDS; i++)
    {
        if ( ax_mask & (1<<(CNC_MAX_COORDS - i - 1)) )
        {
            internal_helper_set_power( i, 200 );
            motion_freerun( i, true, 200, true );
        }
    }
}

static void internal_ob_helper_findZ_goto_setup( int sequence )
{
    struct SMotionSequence seq;
    motion_sequence_stop();     // paranoya

    seq.cmdID = 0;
    seq.seqID = 1;
    seq.seqType = SEQ_TYPE_GOTO;

    if ( sequence == 0 )       // go to the probe pozition
    {
        motion_get_crt_coord( &seq.params.go_to.coord );
        seq.params.go_to.feed = cnc.setup.feed_rapid;
        seq.params.go_to.coord.coord[COORD_X] = cnc.setup.z_probe.coord[COORD_X];
        seq.params.go_to.coord.coord[COORD_Y] = cnc.setup.z_probe.coord[COORD_Y];
    }
    else
    {
        motion_get_crt_coord( &seq.params.go_to.coord );
        seq.params.go_to.feed = 300;
        seq.params.go_to.coord = cnc.setup.z_probe;
        seq.params.go_to.coord.coord[COORD_Z] = 0;
        // set up touch detection
        if ( front_end_check_op_busy() )
        {
            front_end_terminate_touch_detection();
            front_end_sync_event_loop();
        }
        front_end_request_touch_detection( FE_TOUCH_PROBE );
    }

    motion_sequence_insert( &seq );
    motion_sequence_start();
    LED_Op( LED_PROCESSING, LED_blink_fast );
}

static int internal_ob_helper_check_skipped_step( struct SEventStruct *evt, uint32 outband )
{
    int res;

    res = internal_procedure_coordinate_check_poll(evt);
    if ( res )
    {
        uint32 *mfail;
        if ( outband == CMD_OBSA_GO_HOME )
            mfail = &cnc.status.outband.params.gohome.m_fail;
        else
            mfail = &cnc.status.outband.params.findZ.m_fail;

        if ( res == 1 )                                 // if coordinate readout is available from front-end
        {
            if ( internal_procedure_coordinate_check_crt_coord(SEQ_MAX_COORDDEV_OUTBAND, false) )
            {
                // missed step detected
                if ( cnc.status.misc.stats_tstuck < 0xffff )
                    cnc.status.misc.stats_tstuck++;
                cnc.status.flags.f.err_step = 1;

                (*mfail) += 100;     // failure ctr is decreased with 100 at each 10ms. so failure can be accumulated if problem isn't solved
                if ( (*mfail) > (SEQ_MAX_MOVEMENT_RETRIALS * 100) )
                {
                    cnc.status.flags.f.err_code = GENFAULT_TABLE_STUCK;
                    cnc.status.flags.f.err_fatal = 1;
                    LED_Op( LED_FAILURE, LED_blink_slow );
                    return -1;
                }
                // stop everything - but maintain procedure status
                // this will updagte the coordinates also
                internal_stop( true, false );

                if ( outband == CMD_OBSA_GO_HOME )
                {
                    res = motion_sequence_crt_seqID();              // get the failed sequence id
                    if ( (res != 1) && (res != 2) )
                        return -1;
                    internal_ob_helper_gohome_setsequence( res );   // regenerate movement sequences
                }
                else
                {
                    internal_ob_helper_findZ_goto_setup( 0 );
                }
            }
            else if ( (*mfail) )
            {
                (*mfail)--;
            }
        }
        else                                            // if coordinate readout failed
            return -1;
    }

    // trigger coordinate check
    if ( evt->timer_tick_100us )
    {
        uint32 *ccheck;
        if ( outband == CMD_OBSA_GO_HOME )
            ccheck = &cnc.status.outband.params.gohome.ccheck;
        else
            ccheck = &cnc.status.outband.params.findZ.ccheck;

        (*ccheck)--;
        if ( (*ccheck) == 0 )
        {
            (*ccheck) = SEQ_PERIOD_COORD_CHK_OUTBAND;
            internal_procedure_coordinate_check_entry();
        }
    }
    return 0;
}

static void internal_ob_helper_send_coordinates( uint32 msg_type )
{
    struct ScmdIfResponse resp;
    struct ScmdIfCommand *ibcmd;

    ibcmd = internal_fifo_get_last_introduced();

    resp.cmd_type = CMD_OB_GET_CRT_COORD;
    resp.resp_type = msg_type;
    resp.resp.getCoord.has_data = 1;
    resp.resp.getCoord.cmdIDex = motion_sequence_crt_cmdID();
    resp.resp.getCoord.cmdIDq = (ibcmd != NULL) ? ibcmd->cmdID : 0x00;
    motion_get_crt_coord( &resp.resp.getCoord.coord );

    cmdif_confirm_reception(&resp);
}

static void internal_ob_helper_setstate_started(void)
{
    cnc.status.flags.f.run_outband = 1;
    cnc.status.flags.f.run_ob_failed = 0;
    cnc.status.flags.f.run_ob_suceeded = 0;
}

static void internal_ob_helper_setstate_succeed(void)
{
    cnc.status.flags.f.run_outband = 0;
    cnc.status.flags.f.run_ob_suceeded = 1;
    if ( cnc.status.flags.f.run_paused )
        LED_Op( LED_PROCESSING, LED_blink_slow );
}

static void internal_ob_helper_setstate_failed(void)
{
    cnc.status.flags.f.run_outband = 0;
    cnc.status.flags.f.run_ob_failed = 1;
    if ( cnc.status.flags.f.run_paused )
        LED_Op( LED_PROCESSING, LED_blink_slow );
}


static void ihp_fillvector( struct SStepCoordinates *coord, double *p )
{
    p[0] = coord->coord[0];
    p[1] = coord->coord[1];
    p[2] = coord->coord[2];
}

static void iph_conv_back( double *a, struct SStepCoordinates *coord )
{
    int i;
    for (i=0; i<3; i++)
    {
        if ( a[i] < 0 )
            coord->coord[i] = 0;
        else if ( (TStepCoord)(a[i]) > cnc.setup.max_travel.coord[i] )
            coord->coord[i] = cnc.setup.max_travel.coord[i];
        else
            coord->coord[i] = (TStepCoord)(a[i]);
    }
    coord->coord[3] = 0;
}

static void iph_substract( double *a, double *b, double *res )
{
    res[0] = a[0] - b[0];
    res[1] = a[1] - b[1];
    res[2] = a[2] - b[2];
}

static void iph_multiply_dot( double *a, double *b, double *res )
{
    *res = a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

static void iph_lenght_squared( double *a, double *res )
{
    *res = a[0] * a[0] + a[1] * a[1] + a[2] * a[2];
}

static void iph_multiply( double *a, double scalar, double *res )
{
    res[0] = a[0] * scalar;
    res[1] = a[1] * scalar;
    res[2] = a[2] * scalar;
}


static void internal_helper_perpendicular( struct SStepCoordinates *crt_coord, struct SStepCoordinates *pi1, struct SStepCoordinates *pi2, struct SStepCoordinates *perp_coord )
{
/*  - from stackoverflow http://stackoverflow.com/questions/9368436/3d-perpendicular-point-on-line-from-3d-point

    Parametric equations:
    x = x1 + t(x2-x1), y = y1 + t(y2-y1), z = z1 + t(z2-z1)
    The vector u is defined by the coefficients of t. <x2-x1, y2-y1, z2-z1>.

    The vector PQ is defined by Q=crt_coord minus a point P=perp_coord on the line. Any point on the line can be chosen,
    so it would be simplest to just use the line t = 0, which simplifies to x1, y1, and z1. <xc-x1, yc-y1, zc-z1>

    For the point of intersection the formula to find the actual point is a bit different.
    To do that, you need to find the component of u in the direction of PQ:

    ((PQ . u) / ||u||^2) * u

    This gets us the w1 component, but we want w2, which is the component between Q and the line:

    PQ = w1 + w2   ->    w2 = PQ - w1
    From there, we take w2 and add it to the point Q to get the point on the line nearest Q. In code this would be:

    Vector3 p1 = new Vector3(x1, y1, z1);
    Vector3 p2 = new Vector3(x2, y2, z2);
    Vector3 q = new Vector3(xc, yc, zc);

    Vector3 u = p2 - p1;
    Vector3 pq = q - p1;
    Vector3 w2 = pq - Vector3.Multiply(u, Vector3.Dot(pq, u) / u.LengthSquared);

    Vector3 point = q - w2;    ---->  Where point.X is xp, point.Y is yp, and point.Z is zp

    We will work with double precision since this operation is not time critical
*/
    double p1[3];
    double p2[3];
    double q[3];
    double u[3];
    double pq[3];
    double dot;
    double usq;

    ihp_fillvector( pi1, p1 );          // Vector3 p1 = new Vector3(x1, y1, z1);
    ihp_fillvector( pi2, p2 );          // Vector3 p2 = new Vector3(x2, y2, z2);
    ihp_fillvector( crt_coord, q );     // Vector3 q = new Vector3(xc, yc, zc);

    iph_substract( p2, p1, u );         // Vector3 u = p2 - p1;
    iph_substract( q, p1, pq );         // Vector3 pq = q - p1;

    // Vector3 w2 = pq - Vector3.Multiply(u, Vector3.Dot(pq, u) / u.LengthSquared);
    // p1, p2 are reusable                                                           ((PQ . u) / ||u||^2) * u
    iph_multiply_dot( pq, u, &dot );            // Vector3.Dot(pq, u)
    iph_lenght_squared( u, &usq );              // u.LengthSquared
    if ( usq == 0 )
    {
        *perp_coord = *pi2;                     // for the case when the line to be approached is with 0 length -> use the p2 as goto coordinate
        return;
    }
    iph_multiply( u, dot / usq, p1 );           // Vector3.Multiply(u, Vector3.Dot(pq, u) / u.LengthSquared)
    iph_substract( pq, p1, p2 );                // Vector3 w2 = pq - multiply from abowe;

    iph_substract( q, p2, p1 );                 // Vector3 point = q - w2

    iph_conv_back( p1, perp_coord );            // output the result
}


static inline void internal_ib_helper_clear_finished_cmd( bool leave_last )
{
    struct ScmdIfCommand *cmd;
    uint32 cmdID_crt;       // command ID of the currently executed, or finished
    bool was_the_same = false;
    bool do_erase;

    cmdID_crt = motion_sequence_crt_cmdID();
    if ( cmdID_crt == 0 )                               // internal sequence is in run (resuming operation)
    {                                                   // no command from the fifo should be touched
        return;
    }
    else if (leave_last == false)                       // sequence from a command in run
    {
        cnc.status.inband.restartable = false;          // - clean the restartable flag - meaning that we are finished with the restarting (if applicable)
        cnc.status.flags.f.run_recovering = 0;
        LED_Op( LED_FAILURE, LED_off );
    }

    if ( motion_sequence_check_run() )
        LED_Op( LED_PROCESSING, LED_blink_fast );
    else
        LED_Op( LED_PROCESSING, LED_off );

    do
    {
        do_erase = false;
        cmd = internal_fifo_peek_eraseable();
        if ( cmd == NULL )
            break;

        if ( cmd->cmdID != cmdID_crt )                  // if erasable cmd is not the currently executing
        {
            if (was_the_same == false)                  // and is not after the current finished one
                do_erase = true;                        //      delete it
            else                                        // if it was after the current finished one
                break;                                  //      we are done
        }
        else                                            //  if we reached the current command
        {
            if ( (motion_sequence_check_run() == false) &&  // and nothing is executed (current cmd is terminated)
                 (leave_last == false) )
                do_erase = true;                        //      delete it
            else                                        // if the current command is still running
                break;                                  //      we are done
            was_the_same = true;
        }

        if ( do_erase )                                 // erase the command from cmd fifo
        {
            if ( cmd->cmd_type == CMD_IB_GOTO )         // but if movement command - save it's destination coordinate as start coordinate of the new command
                cnc.status.inband.resume.last_erased_coord = cmd->cmd.ib_goto.coord;
            internal_fifo_erase();
        }

    }
    while ( 1 );
}


static inline void internal_ib_helper_fetch_and_push_cmd( void )
{
    struct SMotionSequence *seq;
    struct ScmdIfCommand *cmd;
    bool to_start = false;

    if ( cnc.status.flags.f.run_paused )
        return;

    cmd = internal_fifo_peek_readable();
    if ( cmd )
    {
        switch ( cmd->cmd_type )
        {
            // commands which can be pushed right away
            case CMD_IB_GOTO:
            case CMD_IB_WAIT:
            case CMD_IB_SPINDLE:
                seq = motion_sequence_get_fill_pointer();
                if ( seq )
                {
                    seq->cmdID = cmd->cmdID;
                    seq->seqID = 1;                         // seqID for inbands means - sequences straight from program
                    if ( cmd->cmd_type == CMD_IB_GOTO)
                    {
                        seq->seqType = SEQ_TYPE_GOTO;
                        seq->params.go_to.feed = cmd->cmd.ib_goto.feed;
                        seq->params.go_to.coord = cmd->cmd.ib_goto.coord;
                    }
                    else if ( cmd->cmd_type == CMD_IB_WAIT)
                    {
                        seq->seqType = SEQ_TYPE_HOLD;
                        seq->params.hold = cmd->cmd.ib_wait;
                    }
                    else
                    {
                        seq->seqType = SEQ_TYPE_SPINDLE;
                        seq->params.spindle = cmd->cmd.ib_spindle_speed;
                    }
                    motion_sequence_insert(NULL);   // push the built sequence in the motion core
                    internal_fifo_pull();           // pull out the currently processed command

                    if ( motion_sequence_check_run() )
                        LED_Op( LED_PROCESSING, LED_blink_fast );
                }
                else
                    to_start = true;                // start when sequence fifo is full
                break;
            // commands which can generate more sequences
            case CMD_IB_DRILL:
                // TODO
                break;
        }
    }
    else
        to_start = true;                            // start when command fifo is empty

    if ( to_start && (cnc.status.inband.mc_started == false))
    {
        cnc.status.inband.mc_started = true;
        cnc.status.inband.check_coord = SEQ_PERIOD_COORD_CHK_OUTBAND;
        motion_sequence_start();
        if ( motion_sequence_check_run() )
            LED_Op( LED_PROCESSING, LED_blink_fast );
    }
}


static void internal_ib_helper_stop_non_restartable( void )
{
    // stop mc, stop spindle, flush sequence fifo, flush the command fifo, terminate started mode
    internal_stop(false, true);
    internal_fifo_flush();

    cnc.status.cmd.last_cmdID = 0;
    motion_get_crt_coord( &cnc.status.cmd.last_coord );

    cnc.status.flags.f.run_program = 0;
    cnc.status.flags.f.run_paused = 0;
    cnc.status.flags.f.run_recovering = 0;
    cnc.status.inband.restartable = false;
}

static void internal_ib_helper_stop_restartable( bool stop_spindle )
{
    // stop motion core and flush sequence fifo, command fifo remains,
    // - if the interrupted command was a goto: need to save the start and end points of it
    // - if it was a drill cmd. save the last sequence's start and end point, and memorize which sequence it was, and what needs to be regenerated
    // - for others, just re-execute the interrupted command
    motion_sequence_stop();                                                 // hold still any movements
    motion_get_crt_coord(&cnc.status.inband.resume.stopped_soft_coord);     // get the currently known step coordinates (for reference)
    cnc.status.inband.mc_started = false;
    cnc.status.inband.check_coord = 0;
    cnc.status.inband.restartable = true;

    cnc.status.flags.f.run_program = 1;
    cnc.status.flags.f.run_paused = 1;

    internal_stop(false, stop_spindle);             // stop everything including the spindle, update the crt. coordinates from front-end

    motion_get_crt_coord( &cnc.status.inband.resume.stopped_hard_coord );   // motion core coordinates are updated at internal_stop() from the front_end if
                                                                            // difference is > +/-1 step

    internal_ib_helper_clear_finished_cmd( true );  // clean up any finished remaining commands in the inband command fifo - leaving the current one
                                                    // this will update the end coordinate of the deleted cmd (start coordinate of the current one)

    internal_fifo_move_read_to_eraser();            // move read pointer to the eraser - reactivating the unfinished commands

}

static void internal_ib_helper_resume( void )
{
    // start the run again.
    // - if spindle was stopped, but speed is set up --> insert spindle_on sequence
    // -> if go_home, find_z, freerun, or any movement outband was used --> insert sequences to the leaved point first on XY, then on Z  (possible only after pause command)
    // -> if coordinate mismatch detected then --> insert sequence which is 3D perpendicular to the leaved point from the current coordinate  (possible on table stuck or spindle stuck event)
    // - push the interrupted or the next command (internal_ib_helper_fetch_and_push_cmd)

    struct SMotionSequence *seq;
    struct ScmdIfCommand *cmd;
    struct SStepCoordinates crt_coord;

    if ( cnc.status.inband.restartable == false )   // if no restartable stop was given - do nothing
        return;
                                                    // we will clear this flag only when restarting is finished and sequencer executes actual commands from fifo
    cnc.status.flags.f.run_paused = 0;              // clear the pause flag to prevent start entering here

    // check for spindle start
    if ( (cnc.status.flags.f.spindle_on == 0) && (cnc.status.misc.spindle_speed != 0) )
    {
        // spindle should be started
        seq = motion_sequence_get_fill_pointer();
        seq->cmdID = 0;                             // no command ID with this
        seq->seqID = IB_SEQID_SPINDLE;              // internal sequence ID for spindle start

        seq->seqType = SEQ_TYPE_SPINDLE;
        seq->params.spindle = cnc.status.misc.spindle_speed;
        motion_sequence_insert(NULL);   // push the built sequence in the motion core
    }

    // check if movement outbands were executed
    motion_get_crt_coord( &crt_coord );
    if ( mutil_coordinates_differ( &crt_coord, &cnc.status.inband.resume.stopped_hard_coord ) )     // coordinates differ from last physical - need to go in XY then Z scheme
    {
        int i;
        for ( i=0; i<2; i++ )
        {
            seq = motion_sequence_get_fill_pointer();
            seq->cmdID = 0;                             // no command ID with this
            seq->seqID = IB_SEQID_GOBACK_XY + i;            // internal sequence ID for XY go-back

            seq->seqType = SEQ_TYPE_GOTO;
            seq->params.go_to.coord = cnc.status.inband.resume.stopped_hard_coord;
            if ( i == 0 )
            {
                seq->params.go_to.feed = cnc.setup.feed_rapid;
                seq->params.go_to.coord.coord[ COORD_Z ] = crt_coord.coord[ COORD_Z ];
            }
            else
                seq->params.go_to.feed = cnc.setup.feed_rapid * 2 / 3;

            motion_sequence_insert(NULL);   // push the built sequence in the motion core
        }
    }

    // check for realignment
    cmd = internal_fifo_peek_readable();            // get the currently interrupted command
    if ( cmd )
    {
        if ( (cmd->cmd_type == CMD_IB_GOTO) &&                                                                                          // it was a movement command
             (mutil_coordinates_differ( &cnc.status.inband.resume.stopped_hard_coord, &cnc.status.inband.resume.stopped_soft_coord )) ) // and internal coordinates differred from the coordinates that needed to be on the line
        {
            // hard coordinates are the one updated from front-end - the actual position
            // soft coordinates are what needed to be in that moment
            // - execute a perpendicular sequence from the hard coordinates to the line defined by the last_erased_coord and it's final coordinates
            struct SStepCoordinates perp_coord;

            internal_helper_perpendicular( &cnc.status.inband.resume.stopped_hard_coord,
                                           &cnc.status.inband.resume.last_erased_coord, &cmd->cmd.ib_goto.coord,
                                           &perp_coord );

            seq = motion_sequence_get_fill_pointer();
            seq->cmdID = 0;                             // no command ID with this
            seq->seqID = IB_SEQID_REALIGN;
            seq->seqType = SEQ_TYPE_GOTO;
            seq->params.go_to.coord = perp_coord;
            seq->params.go_to.feed = cmd->cmd.ib_goto.feed;

            motion_sequence_insert(NULL);   // push the built sequence in the motion core

        }
    }

    // push the first command from command fifo
    internal_ib_helper_fetch_and_push_cmd();

}

static inline void internal_ib_helper_process_callback( struct SEventStruct *evt )
{
    if ( cnc.status.inband.cb_operation == SEQ_TYPE_HOLD )      // hold timer
    {
        if ( evt->timer_tick_10ms )
        {
            cnc.status.inband.cb_op.wait--;
            if ( cnc.status.inband.cb_op.wait == 0 )
            {
                cnc.status.inband.cb_operation = 0;
                motion_sequence_confirm_inband_execution();
            }
        }
    }
    else if ( cnc.status.inband.cb_operation == SEQ_TYPE_SPINDLE )  // spindle control
    {
        if ( evt->timer_tick_10ms || evt->fe_op_completed )
        {
            int res;

            if ( cnc.status.inband.cb_op.spindle.timeout )      // in case if timeout is needed - process it first
            {
                cnc.status.inband.cb_op.spindle.timeout--;
                if ( cnc.status.inband.cb_op.spindle.timeout == 0 )
                {
                    internal_procedure_spindle_control_entry( cnc.status.inband.cb_op.spindle.speed );
                }
                return;
            }

            res = internal_procedure_spindle_poll(evt);
            if ( res == 0 )                                     // spindle procedure not finished
                return;
            if ( res == 1 )                                     // procedure finished with success
            {
                cnc.status.inband.cb_operation = 0;
                if ( cnc.status.inband.cb_op.spindle.not_callback )
                    internal_ib_helper_resume();                // case when internal error - and retrying ( spindle stuck or table stuck )
                else
                {
                    motion_sequence_confirm_inband_execution();
                }
                return;
            }
            // error handling
            if ( cnc.status.misc.stats_spstuck < 0xffff )
                cnc.status.misc.stats_spstuck++;
            cnc.status.inband.cb_op.spindle.retry --;

            if ( (cnc.status.inband.cb_op.spindle.retry == 0) ||    // retried X times, no success
                 (cnc.status.flags.f.err_fatal) )
            {
                if ( cnc.status.flags.f.err_fatal == 0 )
                {
                    cnc.status.flags.f.err_code = GENFAULT_SPINDLE_STUCK;
                }
                cnc.status.flags.f.err_fatal = 1;

                LED_Op( LED_FAILURE, LED_blink_slow );
                internal_ib_helper_stop_non_restartable();
            }
            else
            {
                cnc.status.inband.cb_op.spindle.timeout = 50;   // 0.5 sec. timeout bw. retrials
            }
        }
    }
}

static inline void internal_ib_helper_check_operation( struct SEventStruct *evt )
{
    // check spindle
    if ( cnc.status.flags.f.spindle_on && HW_FrontEnd_Event() )     // only spindle jam can be - this is the only active
    {
        cnc.status.inband.cb_operation = SEQ_TYPE_SPINDLE;
        cnc.status.inband.cb_op.spindle.speed = cnc.status.misc.spindle_speed;
        cnc.status.inband.cb_op.spindle.retry = SEQ_MAX_MOVEMENT_RETRIALS;
        cnc.status.inband.cb_op.spindle.timeout = 100;                      // 1 second pause before trying
        cnc.status.inband.cb_op.spindle.not_callback = true;                // this will call the start over
        internal_ib_helper_stop_restartable( true );
        LED_Op( LED_FAILURE, LED_blink_fast );
        cnc.status.flags.f.run_recovering = 1;
        cnc.status.flags.f.err_spjam = 1;
        if ( cnc.status.misc.stats_spstuck < 0xffff )
            cnc.status.misc.stats_spstuck++;
        return;
    }

    // check for coordinate
    if ( (evt->timer_tick_100us || evt->fe_op_completed) &&
          motion_sequence_check_run() )
    {
        int res;
        res = internal_procedure_coordinate_check_poll(evt);
        if ( res )                                                  // coordinate update completion
        {
            if (res == 1)
            {
                if ( internal_procedure_coordinate_check_crt_coord( SEQ_MAX_COORDDEV_INBAND, true ) )
                {
                    internal_ib_helper_stop_restartable( false );
                    if ( cnc.status.misc.stats_tstuck < 0xffff )
                        cnc.status.misc.stats_tstuck++;
                    cnc.status.flags.f.err_step = 1;

                    cnc.status.inband.coord_fail += 100;     // failure ctr is decreased with 100 at each 10ms. so failure can be accumulated if problem isn't solved
                    if ( cnc.status.inband.coord_fail > (SEQ_MAX_MOVEMENT_RETRIALS * 100) )
                    {
                        // if restart failed X times - halt everything
                        cnc.status.flags.f.err_code = GENFAULT_TABLE_STUCK;
                        cnc.status.flags.f.err_fatal = 1;
                        LED_Op( LED_FAILURE, LED_blink_slow );
                        internal_ib_helper_stop_non_restartable();
                    }
                    else
                    {
                        // if failed < X times - restart it beginning with the spindle (since it is stopped at the failure detection)
                        cnc.status.inband.cb_operation = SEQ_TYPE_SPINDLE;
                        cnc.status.inband.cb_op.spindle.speed = cnc.status.misc.spindle_speed;
                        cnc.status.inband.cb_op.spindle.retry = SEQ_MAX_MOVEMENT_RETRIALS;
                        cnc.status.inband.cb_op.spindle.timeout = 50;
                        cnc.status.inband.cb_op.spindle.not_callback = true;        // this will call the start over when spindle is OK
                        cnc.status.flags.f.run_recovering = 1;
                        LED_Op( LED_FAILURE, LED_blink_fast );
                    }
                }
                else if ( cnc.status.inband.coord_fail )
                    cnc.status.inband.coord_fail --;
            }
            else
            {
                // communication failure with the front-end
                internal_ib_helper_stop_non_restartable();
            }
        }
        else if ( cnc.status.inband.check_coord )                   // call for coordinate update if is the time
        {
            cnc.status.inband.check_coord--;
            if ( cnc.status.inband.check_coord == 0 )
            {
                internal_procedure_coordinate_check_entry();
                cnc.status.inband.check_coord = SEQ_PERIOD_COORD_CHK_OUTBAND;
            }
        }
    }
}



int internal_stop( bool leave_ob, bool stop_spindle )
{
    // stop motion core
    motion_sequence_stop();
    LED_Op( LED_PROCESSING, LED_off );

    if ( (cnc.status.flags.f.err_fatal == 0) || (cnc.status.flags.f.err_code != GENFAULT_FRONT_END) )
    {
        // wait front-end to finish it's job
        if ( front_end_check_op_busy() )
        {
            front_end_terminate_touch_detection();
            front_end_sync_event_loop();
        }

        // spindle off - if no need for status preservation
        if ( cnc.status.flags.f.spindle_on && stop_spindle )
        {
            front_end_spindle_speed(0);
            front_end_sync_event_loop();
            cnc.status.flags.f.spindle_on = 0;
        }
        else
        {
            HW_Wait_Reset();
        }

        // get the current coordinates and update the soft coordinates if necessary
        front_end_request_coordinates();
        if (front_end_sync_event_loop())
        {
            cnc.status.flags.f.err_fatal = 1;
            cnc.status.flags.f.err_code  = GENFAULT_FRONT_END;
            LED_Op( LED_FAILURE, LED_blink_slow );
        }
        else
        {
            motion_get_crt_coord( &cnc.status.procedure.params.getcoord.coord_snapshot );
            if (internal_procedure_coordinate_check_crt_coord( 1, true ))
                motion_set_crt_coord( &cnc.status.procedure.params.getcoord.coord_fe );
        }
    }

    // reset status
    if ( leave_ob == false )
    {
        cnc.status.flags.f.run_outband = 0;

        cnc.status.outband.command.cmd_type = 0;
        cnc.status.outband.state = 0;
        cnc.status.procedure.procID = procid_none;
    }

    // set axis power to auto
    motion_pwr_ctrl( COORD_X, mpwr_auto );
    motion_pwr_ctrl( COORD_Y, mpwr_auto );
    motion_pwr_ctrl( COORD_Z, mpwr_auto );
    motion_pwr_ctrl( COORD_A, mpwr_off );       // A channel is not used in current implementation

    return 0;
}


static void internal_cmd_pause(void)
{
    if ( cnc.status.flags.f.run_outband )
    {
        internal_stop(false, true);
    }
    else
    if ( cnc.status.flags.f.run_program )    // from stopped mode, no outband is in run
    {
        internal_ib_helper_stop_restartable( true );
        LED_Op( LED_PROCESSING, LED_blink_slow );
        cnc.status.flags.f.run_recovering = 0;
    }
}

static inline int internal_outband_reset(void)
{
    struct ScmdIfResponse resp;

    // send response before resetting everything
    resp.cmd_type = CMD_OB_RESET;
    resp.resp_type = RESP_ACK;
    cmdif_confirm_reception( &resp );

    HW_Wait_Reset();        // brute waiting loop for all comm. to make timeout or tx fifo to get empty

    sequencer_init(true);

    return RESP_ACK;
}

static inline int internal_outband_get_stats(void)
{
    struct ScmdIfResponse resp;

    resp.cmd_type = CMD_OB_GET_STATS;
    resp.resp_type = RESP_ACK;

    resp.resp.stats.hb_crt = cnc.status.misc.hb_crt;
    resp.resp.stats.hb_max = cnc.status.misc.hb_max;
    resp.resp.stats.hb_min = cnc.status.misc.hb_min;
    cnc.status.misc.hb_crt = 0;
    cnc.status.misc.hb_max = 0;
    cnc.status.misc.hb_min = 0xffff;

    resp.resp.stats.cnc_tstuck = cnc.status.misc.stats_tstuck;
    resp.resp.stats.cnc_spstuck = cnc.status.misc.stats_spstuck;
    cnc.status.misc.stats_tstuck = 0;
    cnc.status.misc.stats_spstuck = 0;

    cmdif_get_stats( &resp.resp.stats.comm_ovf, &resp.resp.stats.comm_ckrej, &resp.resp.stats.comm_conrej, &resp.resp.stats.comm_tout );
    front_end_get_stats( &resp.resp.stats.fe_tout, &resp.resp.stats.fe_rej, &resp.resp.stats.fe_retry );
    resp.resp.stats.mc_starv = motion_stats_starved();

    cmdif_confirm_reception(&resp);
    return RESP_ACK;
}

static inline int internal_outband_max_travel( struct ScmdIfCommand *cmd )
{
    int i;
    if ( cnc.status.flags.f.run_outband || cnc.status.flags.f.run_program )
        return RESP_PEN;

    motion_set_max_travel( &cmd->cmd.max_travel );
    motion_set_crt_coord( &cmd->cmd.max_travel );
    cnc.setup.max_travel = cmd->cmd.max_travel;

    for (i=0; i<CNC_MAX_COORDS; i++)
    {
        if ( cmd->cmd.home_poz.coord[i] > cnc.setup.max_travel.coord[i] )
            cmd->cmd.home_poz.coord[i] = cnc.setup.max_travel.coord[i];
    }

    internal_send_simple_ack( cmd );
    return RESP_ACK;
}

static inline int internal_outband_max_speeds( struct ScmdIfCommand *cmd )
{

    if ( cnc.status.flags.f.run_outband || cnc.status.flags.f.run_program )
        return RESP_PEN;

    if ( cmd->cmd.max_speeds.rapid > cmd->cmd.max_speeds.absolute )
        return RESP_INV;
    if ( (cmd->cmd.max_speeds.rapid < CNC_MIN_FEED) ||
         (cmd->cmd.max_speeds.absolute < CNC_MIN_FEED) )
        return RESP_INV;

    cnc.setup.feed_max = cmd->cmd.max_speeds.absolute;
    cnc.setup.feed_rapid = cmd->cmd.max_speeds.rapid;

    internal_send_simple_ack( cmd );
    return RESP_ACK;
}

static inline int internal_outband_home_poz( struct ScmdIfCommand *cmd )
{
    int i;

    if ( cnc.status.flags.f.run_outband )
        return RESP_PEN;

    for (i=0; i<CNC_MAX_COORDS; i++)
    {
        if ( cmd->cmd.home_poz.coord[i] > cnc.setup.max_travel.coord[i] )
            return RESP_INV;
    }

    cnc.setup.home_poz = cmd->cmd.home_poz;

    internal_send_simple_ack( cmd );
    return RESP_ACK;
}

static inline int internal_outband_probe_poz( struct ScmdIfCommand *cmd )
{
    int i;

    if ( cnc.status.flags.f.run_outband )
        return RESP_PEN;

    for (i=0; i<2; i++)
    {
        if ( cmd->cmd.probe_poz.coord[i] > cnc.setup.max_travel.coord[i] )
            return RESP_INV;
    }

    cnc.setup.z_probe = cmd->cmd.probe_poz;
    cnc.setup.z_probe.coord[COORD_Z] = 0;
    cnc.setup.z_probe.coord[COORD_A] = 0;

    internal_send_simple_ack( cmd );
    return RESP_ACK;
}

static inline int internal_outband_find_origin( void )
{
    // search the endpoints
    // If no front end - set crt coord to max travel
    // If front end:
    //  - back off 450 steps on each axis until no endpoint pulse is received (clear the coarse sensor)
    //  - freerun each axis till endpoint pulse is detected - repeat this until all axis has it's signal
    //  - set crt coord to max travel and reset front-end coordinates to max
    // Note: no table lock-up is detected here, user should prevent operational failures
    if ( cnc.status.flags.f.run_outband || cnc.status.flags.f.run_program )
        return RESP_PEN;

    motion_set_crt_coord( &cnc.setup.max_travel );
    if ( cnc.setup.fe_present == false )
    {
        cnc.status.cmd.last_coord = cnc.setup.max_travel;
        LED_Op( LED_SYSTEM, LED_on );
    }
    else
    {
        // set up the status of the procedure
        internal_ob_helper_setstate_started();

        cnc.status.outband.command.cmd_type = CMD_OBSA_FIND_ORIGIN;
        cnc.status.outband.state = obstat_forg_spindledn;
        cnc.status.outband.params.findorg.axis_mask = 0x00;

        // power down the spindle for safety reason
        if ( cnc.status.flags.f.spindle_pwr )
            internal_procedure_spindle_pwrdown_entry(true);
        else
        {
            cnc.status.outband.state++;                         // if spindle is powered down - simply skip to the next state
            internal_ob_helper_forg_backoff_setup( 0x0e );
        }
    }

    cnc.status.outband.command.cmd_type = CMD_OBSA_FIND_ORIGIN;
    internal_send_simple_ack( &cnc.status.outband.command );
    return RESP_ACK;
}


static inline int internal_outband_gohome( bool response )
{
    // bring the milling head in home/tool change position
    // stop the spindle if needed
    // fill up the followin sequences:
    //  - z top
    //  - x,y home poz
    //  - run sequence - ch
    if ( cnc.status.flags.f.run_outband || 
         cnc.status.flags.f.run_recovering ||
         (cnc.status.flags.f.run_program && (cnc.status.flags.f.run_paused == 0)) )
        return RESP_PEN;

    // set up the status of the procedure
    internal_ob_helper_setstate_started();

    cnc.status.outband.command.cmd_type = CMD_OBSA_GO_HOME;
    cnc.status.outband.params.gohome.m_fail = 0;
    cnc.status.outband.params.gohome.ccheck = SEQ_PERIOD_COORD_CHK_OUTBAND;
    cnc.status.outband.state = obstat_home_preparation;

    // power down the spindle for safety reason
    if ( cnc.status.flags.f.spindle_pwr )
        internal_procedure_spindle_pwrdown_entry(true);
    else
    {
        cnc.status.outband.state++;
        internal_ob_helper_gohome_setsequence( 1 );
    }

    if ( response )
    {
        // send acknowledge
        cnc.status.outband.command.cmd_type = CMD_OBSA_GO_HOME;
        internal_send_simple_ack( &cnc.status.outband.command );
        return RESP_ACK;
    }
}


static inline int internal_outband_findZzero( void )
{
    if ( cnc.status.flags.f.run_outband || 
         cnc.status.flags.f.run_recovering ||
         (cnc.status.flags.f.run_program && (cnc.status.flags.f.run_paused == 0)) )
        return RESP_PEN;

    if ( (cnc.setup.fe_present == false) || (cnc.setup.z_probe.coord[COORD_X] == -1) )
        return RESP_INV;
    else
    {
        // set up the status of the procedure
        internal_ob_helper_setstate_started();

        cnc.status.outband.command.cmd_type = CMD_OBSA_FIND_Z_ZERO;
        cnc.status.outband.state = obstat_findz_spindledn;
        cnc.status.outband.probe.valid = 0;
        cnc.status.outband.params.findZ.m_fail = 0;
        cnc.status.outband.params.findZ.ccheck = SEQ_PERIOD_COORD_CHK_OUTBAND;

        // power down the spindle for safety reason
        if ( cnc.status.flags.f.spindle_pwr )
            internal_procedure_spindle_pwrdown_entry(true);
        else
        {
            cnc.status.outband.state++;                         // if spindle is powered down - simply skip to the next state
            internal_ob_helper_findZ_goto_setup( 0 );           // set up tranzition to Z probe location
        }
    }

    internal_send_simple_ack( &cnc.status.outband.command );
    return RESP_ACK;
}

static inline int internal_outband_step( struct ScmdIfCommand *cmd )
{
    // step is a standalone simple outband with immediate action
    // no feedback is provided, stop command is recommended to detect miss step
    int i;
    if ( cnc.status.flags.f.run_outband || 
         cnc.status.flags.f.run_recovering ||
         (cnc.status.flags.f.run_program && (cnc.status.flags.f.run_paused == 0)) )
        return RESP_PEN;

    for ( i=0; i<CNC_MAX_COORDS; i++ )
    {
        if ( cmd->cmd.step.axis_mask & (1<<i) )
        {
            motion_pwr_ctrl( i, mpwr_med );
            motion_step(i, cmd->cmd.step.dir_mask & (1<<i));
            motion_pwr_ctrl( i, mpwr_auto );

        }
    }
    internal_send_simple_ack( cmd );
    return RESP_ACK;
}


static inline int internal_outband_freerun( struct ScmdIfCommand *cmd )
{
    // freerun procedure:
    //  - if possible, axis freerun is started, timeout counter reset
    //  - power is set up according to feed speed
    //  - for runs with max travel: the measured coordinates are read back periodically from front-end
    //      if max deviation is >= SEQ_MAX_COORD_DEV_FRUN the procedure is stopped with failure and coordinates updated from readback
    //  - uppon timeout -> feed 0 is given internally
    //  -> if feed 0 is given - procedure is terminated with success if motion core stopped
    //  -> if stop command - motion is stopped immediatelly
    //  At finishing of procedure if more than +/-1 steps is detected, coordinates are updated

    // do some checks
    if ( (cnc.status.flags.f.run_program && (cnc.status.flags.f.run_paused == 0)) ||
         cnc.status.flags.f.run_recovering ||
         ( cnc.status.flags.f.run_outband && (cnc.status.outband.command.cmd_type != CMD_OBSA_FREERUN) ) )      // same outband is permitted, others not
        return RESP_PEN;
    if ( cnc.status.flags.f.run_outband &&
         (cnc.status.outband.command.cmd.frun.axis != cmd->cmd.frun.axis) )                                     // only the same axis is permitted
        return RESP_PEN;
    if ( cmd->cmd.frun.feed > cnc.setup.feed_max )                                                              // feed must be < then max alowed
        return RESP_INV;

    // set up freerun
    internal_helper_set_power( cmd->cmd.frun.axis, cmd->cmd.frun.feed );
    if ( motion_freerun( cmd->cmd.frun.axis, cmd->cmd.frun.dir, cmd->cmd.frun.feed, cmd->cmd.frun.no_limit ) )
    {
        motion_pwr_ctrl( cmd->cmd.frun.axis, mpwr_auto );
        return RESP_PEN;
    }

    // set up the status of the procedure
    internal_ob_helper_setstate_started();

    cnc.status.outband.command = *cmd;
    cnc.status.outband.params.frun.coord_check = SEQ_PERIOD_COORD_CHK_FRUN;
    cnc.status.outband.params.frun.safety_to = SEQ_PERIOD_FRUN_SAFESTOP;
    cnc.status.outband.state = cmd->cmd.frun.feed ? obstat_frun_running : obstat_frun_stopping;

    // send acknowledge
    internal_send_simple_ack( cmd );
    return RESP_ACK;
}


static inline int internal_outband_spindle( struct ScmdIfCommand *cmd )
{
    // spindle control:
    // - powers up the spindle unit if needed
    // - sets the spindle speed
    // do some checks
    if ( cnc.status.flags.f.run_program || cnc.status.flags.f.run_outband )
        return RESP_PEN;
    if ( (cmd->cmd.ib_spindle_speed > cnc.setup.spindle_max) ||
        ((cmd->cmd.ib_spindle_speed < cnc.setup.spindle_min) && (cmd->cmd.ib_spindle_speed != 0)) )
        return RESP_INV;

    // set up the status of the procedure
    if ( internal_procedure_spindle_control_entry( cmd->cmd.ib_spindle_speed ) )
        return RESP_PEN;

    internal_ob_helper_setstate_started();

    cnc.status.outband.command = *cmd;
    cnc.status.outband.state = 0;

    // send acknowledge
    internal_send_simple_ack( cmd );
    return RESP_ACK;
}


static inline int internal_outband_start( void )
{
    if ( cnc.status.flags.f.run_outband )
        return RESP_PEN;

    if ( cnc.status.flags.f.run_program == 0 )      // from stopped mode, no outband is in run
    {
        cnc.status.flags.f.run_program = 1;
        cnc.status.inband.cb_operation = 0;
        cnc.status.inband.mc_started = false;
        cnc.status.inband.cmd_on_hold = false;
        cnc.status.inband.restartable = false;
        cnc.status.inband.check_coord = 0;
        cnc.status.inband.coord_fail = 0;
        motion_get_crt_coord( &cnc.status.inband.resume.last_erased_coord );    // this is the start coordinate of the first goto/drill/arc command
    }
    else if ( cnc.status.flags.f.run_paused && (cnc.status.flags.f.run_recovering == 0) )  // from paused mode
    {
        internal_ib_helper_resume();
    }

    // send acknowledge
    cnc.status.outband.command.cmd_type = CMD_OBSA_START;
    internal_send_simple_ack( &cnc.status.outband.command );
    return RESP_ACK;
}


static inline int internal_outband_pause( void )
{
    struct ScmdIfResponse resp;

    internal_cmd_pause();

    // send acknowledge
    resp.cmd_type = CMD_OB_PAUSE;
    resp.resp_type = RESP_ACK;
    resp.resp.cmdID = motion_sequence_crt_cmdID();
    cmdif_confirm_reception(&resp);
    return RESP_ACK;
}


static inline int internal_outband_stop( void )
{
    struct ScmdIfResponse resp;
    struct ScmdIfCommand *ibElem;

    resp.cmd_type = CMD_OB_STOP;
    resp.resp_type = RESP_ACK;

    ibElem = internal_fifo_get_last_introduced();
    if ( ibElem == NULL )
        resp.resp.stop.cmdIDq = 0;
    else
        resp.resp.stop.cmdIDq = ibElem->cmdID;

    internal_ib_helper_stop_non_restartable();
    motion_feed_scale( 0 );
    cnc.status.misc.spindle_speed = 0;

    if ( cnc.status.flags.f.err_fatal &&
        ((cnc.status.flags.f.err_code == GENFAULT_SPINDLE_STUCK) ||
         (cnc.status.flags.f.err_code == GENFAULT_TABLE_STUCK))    )
    {
        cnc.status.flags.f.err_code = 0;
        cnc.status.flags.f.err_fatal = 0;
    }

    if ( cnc.status.flags.f.err_fatal == 0 )
        LED_Op( LED_FAILURE, LED_off );

    resp.resp.stop.cmdIDex = motion_sequence_crt_cmdID();
    cmdif_confirm_reception(&resp);
    return RESP_ACK;
}


static inline int internal_outband_scale_feed( struct ScmdIfCommand *cmd )
{
    if ( (cmd->cmd.scale_feed > 200) || (cmd->cmd.scale_feed < -200) )
        return RESP_INV;

    motion_feed_scale( cmd->cmd.scale_feed );

    internal_send_simple_ack( cmd );
    return RESP_ACK;
}


static inline int internal_outband_scale_spindle( struct ScmdIfCommand *cmd )
{
    if ( (cmd->cmd.scale_spindle > 200) || (cmd->cmd.scale_spindle < -200) )
        return RESP_INV;

    if ( cnc.status.flags.f.spindle_on )
    {
        if ( cnc.status.flags.f.run_outband )
            return RESP_PEN;

        internal_ob_helper_setstate_started();

        // TODO - implementation
    }
    else
    {
        cnc.setup.spindle_scale = cmd->cmd.scale_spindle;
    }

    internal_send_simple_ack( cmd );
    return RESP_ACK;
}


static inline int internal_outband_get_coord( struct ScmdIfCommand *cmd )
{
    if ( cmd->cmd.get_coord.is_setup )
    {
        if ( cmd->cmd.get_coord.dmp_enable )
            cnc.status.misc.coord_dump_ctr = 10;
        else
            cnc.status.misc.coord_dump_ctr = 0;

        internal_send_simple_ack( cmd );
        return RESP_ACK;
    }

    internal_ob_helper_send_coordinates( RESP_ACK );
    return RESP_ACK;
}


static inline int internal_outband_get_crt_cmdID( void )
{
    struct ScmdIfResponse resp;

    resp.cmd_type = CMD_OB_GET_CRT_CMD_ID;
    resp.resp_type = RESP_ACK;
    resp.resp.cmdID = motion_sequence_crt_cmdID();
    cmdif_confirm_reception(&resp);
    return RESP_ACK;
}


static inline int internal_outband_get_status( void )
{
    struct ScmdIfResponse resp;
    struct ScmdIfCommand *ibcmd;
    bool inband_empty = false;

    ibcmd = internal_fifo_get_last_introduced();

    resp.cmd_type = CMD_OB_GET_STATUS;
    resp.resp_type = RESP_ACK;
    resp.resp.status.cmdIDex = motion_sequence_crt_cmdID();
    resp.resp.status.cmdIDq = (ibcmd != NULL) ? ibcmd->cmdID : 0x00;
    resp.resp.status.freeSpace = internal_fifo_info_free();

    if ( cnc.status.flags.f.run_program &&
         (motion_sequence_check_run() == false) &&
         (internal_fifo_info_usable() == 0) &&
         cnc.status.inband.mc_started )
        inband_empty = true;

    // prepare status byte
    if ( cnc.status.flags.f.run_outband )
        resp.resp.status.status_byte = 0x03;        //  cc = 11
    else if ( cnc.status.flags.f.run_ob_failed )
        resp.resp.status.status_byte = 0x01;        //  cc = 01
    else
        resp.resp.status.status_byte = 0x00;        //  cc = 00

    resp.resp.status.status_byte |= cnc.status.flags.f.run_program ? (1 << 2) : 0x00;       // r
    resp.resp.status.status_byte |= cnc.status.flags.f.run_paused ? (1 << 3) : 0x00;        // p
    resp.resp.status.status_byte |= cnc.status.flags.f.stat_bpress ? (1 << 4) : 0x00;       // B
    resp.resp.status.status_byte |= cnc.status.flags.f.stat_restarted ? (1 << 5) : 0x00;    // I
    resp.resp.status.status_byte |= inband_empty ? (1 << 6) : 0x00;                         // e

    // prepare failure byte
    resp.resp.status.fail_byte  = cnc.status.flags.f.err_code & 0x0f;
    resp.resp.status.fail_byte |= cnc.status.flags.f.err_starv ? (1 << 4) : 0x00;           // s
    resp.resp.status.fail_byte |= cnc.status.flags.f.err_step ? (1 << 5) : 0x00;            // F
    resp.resp.status.fail_byte |= cnc.status.flags.f.err_spjam ? (1 << 6) : 0x00;           // S
    resp.resp.status.fail_byte |= cnc.status.flags.f.err_fatal ? (1 << 7) : 0x00;           // G

    // reset flags
    cnc.status.flags.f.stat_bpress = 0;
    cnc.status.flags.f.err_starv = 0;
    cnc.status.flags.f.err_step = 0;
    cnc.status.flags.f.err_spjam = 0;

    cmdif_confirm_reception(&resp);
    return RESP_ACK;
}


static inline int internal_outband_get_probe( void )
{
    struct ScmdIfResponse resp;

    if ( cnc.status.outband.probe.valid == false )
        return RESP_INV;

    resp.cmd_type = CMD_OB_GET_PROBE_TOUCH;
    resp.resp_type = RESP_ACK;
    resp.resp.touch = cnc.status.outband.probe.poz;
    cmdif_confirm_reception(&resp);
    return RESP_ACK;
}


static inline void internal_poll_ob_go_home( struct SEventStruct *evt )
{
    if ( evt->timer_tick_100us || evt->fe_op_completed )
    {
        int res;
        switch ( cnc.status.outband.state )
        {
            case obstat_home_preparation:
                // wait for spindle power down
                res = internal_procedure_spindle_poll(evt);
                if ( res == 0 )                                     // spindle procedure not finished
                    return;
                if ( res == 1 )                                     // procedure finished with success
                {
                    internal_ob_helper_gohome_setsequence( 1 );     //      set up sequences for home poz
                    cnc.status.outband.state++;
                    break;
                }
                goto _error_exit;
            case obstat_home_motion:
                // wait for finishing of movement

                // check coordinates
                res = internal_ob_helper_check_skipped_step( evt, CMD_OBSA_GO_HOME );
                if ( res )
                    goto _error_exit;

                // see if procedure is finished
                if ( motion_sequence_check_run()== false )
                {
                    // finish the command by stopping and updating everything
                    internal_stop( false, false );
                    internal_ob_helper_setstate_succeed();
                }

                break;
        }
    }
    return;
_error_exit:
    internal_stop( false, false );                // procedure failed
    internal_ob_helper_setstate_failed();
}

static inline void internal_poll_ob_freerun( struct SEventStruct *evt )
{
    int res;
    res = internal_procedure_coordinate_check_poll( evt );
    if ( res )     // procedure completed
    {
        if ( (res == -1) || internal_procedure_coordinate_check_crt_coord(SEQ_MAX_COORDDEV_FRUN, false) )
        {
            // coordinate mismatch detected - steps skipped
            // stop command execution (this will update the coordinates also)
            internal_stop(false, false);
            internal_ob_helper_setstate_failed();
            if ( res == 1 )
                cnc.status.flags.f.err_step = 1;
            return;
        }
    }

    if ( evt->timer_tick_100us )
    {
        // check when motion core finished the run
        if ( motion_sequence_check_run() == false )
        {
            // finish the command by stopping and updating everything
            internal_stop(false, false);
            internal_ob_helper_setstate_succeed();
            return;
        }

        switch ( cnc.status.outband.state )
        {
            case obstat_frun_running:            // running a freerun operation
                // check safety timer
                cnc.status.outband.params.frun.safety_to--;
                if ( cnc.status.outband.params.frun.safety_to == 0 )
                {
                    // safety timeout - stop all the runs gently
                    int i;
                    for (i=0; i<CNC_MAX_COORDS; i++ )
                    {
                        motion_freerun( i, 0, 0, cnc.status.outband.command.cmd.frun.no_limit );
                    }
                    cnc.status.outband.state = obstat_frun_stopping;
                }

                // check coordinate readout
                cnc.status.outband.params.frun.coord_check--;
                if ( cnc.status.outband.params.frun.coord_check == 0 )
                {
                    cnc.status.outband.params.frun.coord_check = SEQ_PERIOD_COORD_CHK_FRUN;
                    if ( cnc.status.outband.command.cmd.frun.no_limit == false )
                    {
                        internal_procedure_coordinate_check_entry();
                    }
                }
                break;
            case obstat_frun_stopping:
                // currently nothing to do here
                break;
        }
    }
}

static inline void internal_poll_ob_find_org( struct SEventStruct *evt )
{
    // check separatelly for endpoint touch
    if ( HW_FrontEnd_Event() && ( cnc.status.outband.state == obstat_forg_search) )
    {
        uint32 tmask = 0;
        int res = 0;

        motion_sequence_stop();                 // stop all the runs
        if ( front_end_check_op_busy() )
            res = front_end_sync_event_loop();  // wait till front-end sends the touch mask
        if ( res )
            goto _fatal_error_exit;
        front_end_get_touched_list( &tmask );
        cnc.status.outband.params.findorg.axis_mask &= ~tmask;
        if ( cnc.status.outband.params.findorg.axis_mask )              // if there are unfinished axis - search for them
        {
            internal_ob_helper_forg_search_setup( cnc.status.outband.params.findorg.axis_mask );
        }
        else                                                            // else
        {
            cnc.status.outband.state++;
            front_end_coordinate_reset_to_max();
        }
    }
    else
    if ( evt->timer_tick_100us || evt->fe_op_completed )
    {
        int res;
        switch ( cnc.status.outband.state )
        {
            case obstat_forg_spindledn:
                // wait for spindle power down
                res = internal_procedure_spindle_poll(evt);
                if ( res == 0 )                                     // spindle procedure not finished
                    return;
                if ( res == 1 )                                     // procedure finished with success
                {
                    internal_ob_helper_forg_backoff_setup( 0x0e );
                    cnc.status.outband.state++;
                    break;
                }
                goto _error_exit;
            case obstat_forg_back_off:
                // wait the finishing of execution or signal detection from endpoint sensor
                if ( evt->fe_op_completed )
                {
                    if ( evt->fe_op_failed )
                        goto _fatal_error_exit;
                    else                                // endpoint sensor signalled
                    {
                        uint32 tmask = 0;
                        front_end_get_touched_list( &tmask );
                        front_end_request_touch_detection( 0x0e & ~tmask );     // reuest detection for the remaining channels
                        cnc.status.outband.params.findorg.axis_mask |= tmask;
                    }
                }
                // check if run is completed
                if ( motion_sequence_check_run() == false )
                {
                    if ( cnc.status.outband.params.findorg.axis_mask )      // detected signal - need to run the axis again
                    {
                        internal_ob_helper_forg_backoff_setup( cnc.status.outband.params.findorg.axis_mask );
                        cnc.status.outband.params.findorg.axis_mask = 0;
                    }
                    else
                    {
                        cnc.status.outband.params.findorg.axis_mask = 0x0e;     // search for XYZ axis
                        internal_ob_helper_forg_search_setup( cnc.status.outband.params.findorg.axis_mask );
                        cnc.status.outband.state++;
                    }
                }
                break;
            case obstat_forg_finalize:
                if ( evt->fe_op_completed )
                {
                    if ( evt->fe_op_failed )
                        goto _fatal_error_exit;
                    else                                // front-end coordinates reset
                    {
                        motion_set_crt_coord( &cnc.setup.max_travel );
                        internal_stop(false, false);
                        cnc.status.flags.f.stat_restarted = 0;
                        LED_Op( LED_SYSTEM, LED_on );
                        motion_get_crt_coord( &cnc.status.cmd.last_coord );
                        internal_ob_helper_setstate_succeed();
                        return;
                    }
                }
                break;
            default:
                break;
        }
    }
    return;
_fatal_error_exit:
    cnc.status.flags.f.err_fatal = 1;
    cnc.status.flags.f.err_code = GENFAULT_FRONT_END;
    LED_Op( LED_FAILURE, LED_blink_slow );
_error_exit:
    internal_stop( false, false );                            // procedure failed
    internal_ob_helper_setstate_failed();
}

static inline void internal_poll_ob_findZ(  struct SEventStruct *evt )
{
    // check separatelly for probe touch
    if ( HW_FrontEnd_Event() && (cnc.status.outband.state == obstat_findz_search) )
    {
        internal_stop( true, false );                           // stop the run and update coordinates if mismatch
        motion_get_crt_coord( &cnc.status.outband.probe.poz );  // save the pozition
        cnc.status.outband.probe.valid = true;
        // set up for home run
        cnc.status.outband.command.cmd_type = CMD_OBSA_GO_HOME;
        cnc.status.outband.params.gohome.m_fail = 0;
        cnc.status.outband.params.gohome.ccheck = SEQ_PERIOD_COORD_CHK_OUTBAND;
        cnc.status.outband.state = obstat_home_motion;
        internal_ob_helper_gohome_setsequence( 1 );
    }
    else
    if ( evt->timer_tick_100us || evt->fe_op_completed )
    {
        int res;
        switch ( cnc.status.outband.state )
        {
            case obstat_findz_spindledn:
                // wait for spindle power down
                res = internal_procedure_spindle_poll(evt);
                if ( res == 0 )                                     // spindle procedure not finished
                    return;
                if ( res == 1 )                                     // procedure finished with success
                {
                    internal_ob_helper_findZ_goto_setup( 0 );
                    cnc.status.outband.state++;
                    break;
                }
                goto _error_exit;
            case obstat_findz_goXY:
                // wait to reach XY

                // check coordinates
                res = internal_ob_helper_check_skipped_step( evt, CMD_OBSA_FIND_Z_ZERO );
                if ( res )
                    goto _error_exit;

                if ( motion_sequence_check_run() == false )     // reached the position
                {
                    internal_ob_helper_findZ_goto_setup( 1 );
                    cnc.status.outband.state++;
                }
                break;
            default:
                break;
        }
    }
    return;
_error_exit:
    internal_stop( false, false );                             // procedure failed
    internal_ob_helper_setstate_failed();
}


static inline void internal_poll_ob_spindle( struct SEventStruct *evt )
{
    if ( evt->timer_tick_10ms || evt->fe_op_completed )
    {
        int res;

        res = internal_procedure_spindle_poll(evt);
        if ( res == 0 )                                     // spindle procedure not finished
            return;
        if ( res == 1 )                                     // procedure finished with success
        {
            internal_ob_helper_setstate_succeed();
            return;
        }
        goto _error_exit;
    }
    return;
_error_exit:
    internal_stop( false, true );                           // procedure failed
    internal_ob_helper_setstate_failed();
}


static inline void internal_poll_inband( struct SEventStruct *evt )
{
    // entered if run program is set
    if ( cnc.status.inband.cmd_on_hold == false )
        internal_ib_helper_fetch_and_push_cmd();

    if ( cnc.status.inband.cb_operation )
        internal_ib_helper_process_callback( evt );
    else if ( cnc.status.flags.f.run_paused == 0 )
        internal_ib_helper_check_operation( evt );          // check only if not in pause (can execute moving outbands)

    if ( evt->cnc_motion_seq_finished && (cnc.status.flags.f.run_paused == 0) )
        internal_ib_helper_clear_finished_cmd( false );
}


static int internal_processcmd_inband( struct ScmdIfCommand *cmd, bool bulk )
{
    int res = -3;
    int i;
    struct ScmdIfCommand *pcmd;
    struct ScmdIfResponse resp;

    // get the fill pointer fom fifo
    if ( bulk )
        pcmd = cmd;
    else
    {
        pcmd = internal_fifo_get_fillable();
        if ( pcmd == NULL )
        {
            resp.cmd_type = CMD_IB_BULK;
            resp.resp_type = RESP_REJ;
            resp.resp.cmdID = cnc.status.cmd.last_cmdID;
            cmdif_confirm_reception( &resp );
            return -1;
        }
        *pcmd = *cmd;
    }


    // check for parameter validity
    switch ( pcmd->cmd_type )
    {
        case CMD_IB_WAIT:
            if ( pcmd->cmd.ib_wait > 0 )
                res = 0;
            break;
        case CMD_IB_SPINDLE:
            if ( pcmd->cmd.ib_spindle_speed <= 60000 )
                res = 0;
            break;
        case CMD_IB_DRILL:
            if ( (pcmd->cmd.ib_drill.coord.coord[COORD_X] > cnc.setup.max_travel.coord[COORD_X]) ||
                 (pcmd->cmd.ib_drill.coord.coord[COORD_Y] > cnc.setup.max_travel.coord[COORD_Y]) ||
                 ((pcmd->cmd.ib_drill.coord.coord[COORD_Z] + (int32)pcmd->cmd.ib_drill.clearance) > cnc.setup.max_travel.coord[COORD_Z]) ||
                 ((pcmd->cmd.ib_drill.coord.coord[COORD_Z] - pcmd->cmd.ib_drill.coord.coord[COORD_A]) < 0) ||
                 (pcmd->cmd.ib_drill.feed > cnc.setup.feed_max) ||
                 (pcmd->cmd.ib_drill.cycles == 0 ) )
                break;
            cnc.status.cmd.last_coord.coord[COORD_X] = pcmd->cmd.ib_drill.coord.coord[COORD_X];
            cnc.status.cmd.last_coord.coord[COORD_Y] = pcmd->cmd.ib_drill.coord.coord[COORD_Y];
            cnc.status.cmd.last_coord.coord[COORD_Z] = pcmd->cmd.ib_drill.coord.coord[COORD_Z] + pcmd->cmd.ib_drill.clearance;
            res = 0;
            break;
        case CMD_IB_GOTO:
            // complete missing fields
            for (i=0; i<CNC_MAX_COORDS; i++)
            {
                if ( (pcmd->cmd.ib_goto.valid_fields & (1<<i)) == 0 )
                {
                    pcmd->cmd.ib_goto.coord.coord[i] = cnc.status.cmd.last_coord.coord[i];
                    pcmd->cmd.ib_goto.valid_fields |= (1<<i);
                }
            }
            if ( (pcmd->cmd.ib_goto.valid_fields & 0x10) == 0 )
            {
                pcmd->cmd.ib_goto.feed = cnc.status.cmd.last_feed;
                pcmd->cmd.ib_goto.valid_fields |= 0x10;
            }
            // check parameter validity
            if ( pcmd->cmd.ib_goto.feed > cnc.setup.feed_max )
                break;

            res = 0;
            for (i=0; i<CNC_MAX_COORDS; i++)
            {
                if ( pcmd->cmd.ib_drill.coord.coord[i] > cnc.setup.max_travel.coord[i] )
                {
                    res = -3;
                    break;
                }
            }
            if ( res == -3 )
                break;

            // save coordinate set
            cnc.status.cmd.last_feed = pcmd->cmd.ib_goto.feed;
            cnc.status.cmd.last_coord = pcmd->cmd.ib_goto.coord;
            break;
    }

    // push and respond
    if ( res == -3 )                // invalid parmeters
    {
        resp.cmd_type = CMD_IB_BULK;
        resp.resp_type = RESP_INV;
        resp.resp.cmdID = cnc.status.cmd.last_cmdID;
    }
    else
    {
        // push to command fifo
        internal_fifo_push(NULL);
        cnc.status.cmd.last_cmdID = pcmd->cmdID;

        resp.cmd_type = pcmd->cmd_type;
        resp.resp_type = RESP_ACK;
        resp.resp.inband.cmdID = pcmd->cmdID;
        resp.resp.inband.Qfree = internal_fifo_info_free();
    }

    if ( bulk == false )
        cmdif_confirm_reception( &resp );   // if single command - respond here

    return res;
}


static inline void internal_processcmd_outband( struct ScmdIfCommand *cmd )
{
    int res = RESP_INV;

    if ( cnc.status.flags.f.err_fatal &&
         ((cmd->cmd_type != CMD_OB_RESET) &&
          (cmd->cmd_type != CMD_OB_GET_STATS) &&
          (cmd->cmd_type != CMD_OB_STOP) &&
          (cmd->cmd_type != CMD_OB_GET_STATUS)) )
    {
        // nothing - res remains invalid
    }
    else
    {
        switch ( cmd->cmd_type )
        {
            case CMD_OB_RESET:                  res = internal_outband_reset(); break;
            case CMD_OB_GET_STATS:              res = internal_outband_get_stats(); break;
            case CMD_OBSA_SETUP_MAX_TRAVEL:     res = internal_outband_max_travel( cmd ); break;
            case CMD_OBSA_SETUP_MAX_SPEEDS:     res = internal_outband_max_speeds( cmd ); break;
            case CMD_OBSA_SETUP_HOME_POZ:       res = internal_outband_home_poz( cmd ); break;
            case CMD_OBSA_SETUP_PROBE_POZ:      res = internal_outband_probe_poz( cmd ); break;
            case CMD_OBSA_FIND_ORIGIN:          res = internal_outband_find_origin(); break;
            case CMD_OBSA_GO_HOME:              res = internal_outband_gohome( true ); break;
            case CMD_OBSA_FIND_Z_ZERO:          res = internal_outband_findZzero(); break;
            case CMD_OBSA_STEP:                 res = internal_outband_step( cmd ); break;
            case CMD_OBSA_FREERUN:              res = internal_outband_freerun( cmd ); break;
            case CMD_OBSA_SPINDLE:              res = internal_outband_spindle( cmd ); break;
            case CMD_OBSA_START:                res = internal_outband_start(); break;
            case CMD_OB_PAUSE:                  res = internal_outband_pause(); break;
            case CMD_OB_STOP:                   res = internal_outband_stop(); break;
            case CMD_OB_SCALE_FEED:             res = internal_outband_scale_feed( cmd ); break;
            case CMD_OB_SCALE_SPINDLE:          res = internal_outband_scale_spindle( cmd ); break;
            case CMD_OB_GET_CRT_COORD:          res = internal_outband_get_coord( cmd ); break;
            case CMD_OB_GET_CRT_CMD_ID:         res = internal_outband_get_crt_cmdID(); break;
            case CMD_OB_GET_STATUS:             res = internal_outband_get_status(); break;
            case CMD_OB_GET_PROBE_TOUCH:        res = internal_outband_get_probe(); break;
        }
    }
    if ( res != RESP_ACK )
    {
        struct ScmdIfResponse resp;
        resp.resp_type = res;
        resp.cmd_type = cmd->cmd_type;
        cmdif_confirm_reception( &resp );
    }
}


static inline void local_sequencer_process_command( struct SEventStruct *evt )
{
    if ( evt->comm_command_ready )
    {
        struct ScmdIfCommand cmd;
        struct ScmdIfResponse resp;
        int res;

        res = cmdif_get_command( &cmd );
        if ( res == 0 )             // check for normal commands
        {
            if ( cmd.cmd_inband )
                internal_processcmd_inband( &cmd, false );
            else
                internal_processcmd_outband( &cmd );
        }
        else if ( res == -2 )        // if bulk command
        {
            struct ScmdIfCommand *pcmd;
            do
            {
                res = -3;
                pcmd = internal_fifo_get_fillable();
                if ( pcmd )                             // if there is room in inband fifo
                {
                    res = cmdif_get_bulk( pcmd );       // get the bulk command
                    if ( res == 0 )
                        res = internal_processcmd_inband( pcmd, true );     // verify and push it
                    if ( (res == -3) || (res == -2) )                       // if param failed - notify master
                    {
                        resp.cmd_type = CMD_IB_BULK;
                        resp.resp.cmdID = cnc.status.cmd.last_cmdID;
                        resp.resp_type = RESP_INV;
                    }
                }
                else                                    // if no room - notify master
                {
                    resp.cmd_type = CMD_IB_BULK;
                    resp.resp.cmdID = cnc.status.cmd.last_cmdID;
                    resp.resp_type = RESP_REJ;
                }

            } while ( res == 0 );

            if ( res == -1 )    // meaning that bulk is processed with success
            {
                resp.cmd_type = CMD_IB_BULK;
                resp.resp_type = RESP_ACK;
                resp.resp.inband.cmdID = cnc.status.cmd.last_cmdID;
                resp.resp.inband.Qfree = internal_fifo_info_free();
            }
            cmdif_confirm_reception( &resp );
        }
        else
        {
            // command generic failure at parsing
            resp.resp_type = RESP_INV;
            resp.cmd_type = CMD_OB_RESET; //dummy
            cmdif_confirm_reception( &resp );
        }
    }

    // process buttons
    if ( evt->button_pressed_emerg || evt->button_pressed_resume || evt->internal_outband_gohome || evt->cnc_motion_seq_fatal )
    {
        if ( evt->button_pressed_emerg )
        {
            if ( cnc.status.flags.f.run_paused == 0 )
                internal_cmd_pause();
            else
            {
                internal_ib_helper_stop_non_restartable();
                motion_feed_scale( 0 );
                cnc.status.misc.spindle_speed = 0;

                if ( cnc.status.flags.f.err_fatal &&
                    ((cnc.status.flags.f.err_code == GENFAULT_SPINDLE_STUCK) ||
                     (cnc.status.flags.f.err_code == GENFAULT_TABLE_STUCK))    )
                {
                    cnc.status.flags.f.err_code = 0;
                    cnc.status.flags.f.err_fatal = 0;
                }

                if ( cnc.status.flags.f.err_fatal == 0 )
                    LED_Op( LED_FAILURE, LED_off );
            }
        }
        else if ( evt->button_pressed_resume )
        {
            if ( cnc.status.flags.f.run_paused &&
                 cnc.status.inband.restartable &&
                (cnc.status.flags.f.run_recovering == 0) )
            {
                internal_ib_helper_resume();
            }
        }
        else if ( evt->internal_outband_gohome )
        {
            if ( (cnc.status.flags.f.run_paused && (cnc.status.flags.f.run_recovering == 0)) ||
                 (cnc.status.flags.f.run_program == 0) )
            {
                internal_outband_gohome( false );
            }
        }

        if ( evt->cnc_motion_seq_fatal )        // fatal internal error from motion core
        {
            internal_ib_helper_stop_non_restartable();
            LED_Op( LED_FAILURE, LED_blink_slow );
            cnc.status.flags.f.err_fatal = 1;
            cnc.status.flags.f.err_code = GENFAULT_INTERNAL;
        }

    }
}


static inline void local_poll_outbands( struct SEventStruct *evt )
{
    if ( cnc.status.flags.f.run_outband == 0 )
        return;

    switch ( cnc.status.outband.command.cmd_type )
    {
        case CMD_OBSA_GO_HOME: internal_poll_ob_go_home( evt ); break;
        case CMD_OBSA_FREERUN: internal_poll_ob_freerun( evt ); break;
        case CMD_OBSA_FIND_ORIGIN: internal_poll_ob_find_org( evt ); break;
        case CMD_OBSA_FIND_Z_ZERO: internal_poll_ob_findZ( evt ); break;
        case CMD_OBSA_SPINDLE: internal_poll_ob_spindle( evt ); break;
    }

}


static inline void local_poll_sequencer( struct SEventStruct *evt )
{
    cnc.status.misc.hb_ctr++;

    // check for coordinate dump
    if ( evt->timer_tick_10ms )
    {
        if ( cnc.status.misc.hb_ctr > 0xffff )
            cnc.status.misc.hb_ctr = 0xffff;

        if ( cnc.status.misc.hb_ctr > cnc.status.misc.hb_max )
            cnc.status.misc.hb_max = cnc.status.misc.hb_ctr;
        if ( cnc.status.misc.hb_ctr < cnc.status.misc.hb_min )
            cnc.status.misc.hb_min = cnc.status.misc.hb_ctr;
        cnc.status.misc.hb_crt = cnc.status.misc.hb_ctr;
        cnc.status.misc.hb_ctr = 0;

        if ( cnc.status.misc.coord_dump_ctr )
        {
            cnc.status.misc.coord_dump_ctr--;
            if ( cnc.status.misc.coord_dump_ctr == 0 )
            {
                cnc.status.misc.coord_dump_ctr = 10;
                internal_ob_helper_send_coordinates( RESP_DMP );
            }
        }
    }

    if ( cnc.status.flags.f.run_program )
    {
        internal_poll_inband( evt );
    }
}


void seq_callback_process_inband( uint32 seqType, uint32 value )
{
    cnc.status.inband.cb_operation = seqType;
    cnc.status.inband.cb_op.spindle.not_callback = false;

    if ( seqType == SEQ_TYPE_HOLD )
    {
        cnc.status.inband.cb_op.wait = value * 100;     // 10ms units
    }
    else if ( seqType == SEQ_TYPE_SPINDLE )
    {
        cnc.status.inband.cb_op.spindle.speed = value;
        cnc.status.inband.cb_op.spindle.retry = SEQ_MAX_MOVEMENT_RETRIALS;
        cnc.status.inband.cb_op.spindle.timeout = 0;
        internal_procedure_spindle_control_entry( value );
    }
}

/* *************************************************
 *
 *  Interface routines
 *
 * *************************************************/


void sequencer_init( bool restart )
{
    struct ScmdIfResponse resp;

    memset( &cnc, 0, sizeof(cnc) );

    motion_init();
    if ( front_end_init() == 0 )
        cnc.setup.fe_present = true;
    cmdif_init();

    internal_fifo_flush();

    // setup defaults
    cnc.setup.max_travel.coord[ COORD_X ] = CNC_DEFAULT_X;
    cnc.setup.max_travel.coord[ COORD_Y ] = CNC_DEFAULT_Y;
    cnc.setup.max_travel.coord[ COORD_Z ] = CNC_DEFAULT_Z;
    cnc.setup.max_travel.coord[ COORD_A ] = CNC_DEFAULT_A;

    memset( &cnc.setup.z_probe, 0xff, sizeof(struct SStepCoordinates) );    // set all to -1 indicating uninitialized structure

    cnc.setup.home_poz      = cnc.setup.max_travel;
    cnc.setup.feed_rapid    = 1000;
    cnc.setup.feed_max      = 1500;
    cnc.setup.spindle_max   = 30000;
    cnc.setup.spindle_min   = 6000;

    // set up initial status
    cnc.status.cmd.last_coord = cnc.setup.max_travel;
    cnc.status.cmd.last_feed = 150;
    cnc.status.flags.f.stat_restarted = 1;

    cnc.status.misc.hb_min = 0xffff;

    // init motion core
    motion_set_crt_coord( &cnc.setup.max_travel );          // consider starting with maximum coordinates
    motion_set_max_travel( &cnc.setup.max_travel );

    motion_pwr_ctrl( COORD_X, mpwr_auto );
    motion_pwr_ctrl( COORD_Y, mpwr_auto );
    motion_pwr_ctrl( COORD_Z, mpwr_auto );
    motion_pwr_ctrl( COORD_A, mpwr_off );       // A channel is not used in current implementation

    // register inband callback
    motion_sequence_register_callback( seq_callback_process_inband );

    LED_Op( LED_SYSTEM, LED_blink_slow );
    LED_Op( LED_PROCESSING, LED_off );
    LED_Op( LED_FAILURE, LED_off );

    // notify master
    resp.resp_type = RESP_RST;
    cmdif_confirm_reception(&resp);
}


void sequencer_poll( struct SEventStruct *evt )
{
    // event generator polls
    front_end_poll(evt);
    cmdif_poll(evt);
    motion_poll(evt);
    // event consumer polls
    local_sequencer_process_command(evt);
    local_poll_outbands( evt );
    local_poll_sequencer( evt );
}


