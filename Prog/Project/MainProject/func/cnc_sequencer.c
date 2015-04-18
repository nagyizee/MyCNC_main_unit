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
 */


struct SCNCSequencerInternals   cnc;


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
    cnc.cmd_fifo.wrtb = CNCSEQ_CMD_FIFO_SIZE;
    cnc.cmd_fifo.rdb = 0;
}

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
int internal_fifo_usable(void)
{
    return cnc.cmd_fifo.rdb;
}

// get the free space in the fifo
int internal_fifo_free(void)
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
    if ( cnc.cmd_fifo.e == cnc.cmd_fifo.r )
        return NULL;
    return &cnc.cmd_fifo.cmd[ cnc.cmd_fifo.e ];
}

// erase an element
int internal_fifo_erase( void )
{
    if ( (cnc.cmd_fifo.e == cnc.cmd_fifo.w) ||
         (cnc.cmd_fifo.e == cnc.cmd_fifo.r) )
        return -1;

    cnc.cmd_fifo.e++;
    cnc.cmd_fifo.e %= CNCSEQ_CMD_FIFO_SIZE;
    cnc.cmd_fifo.wrtb++;
    return 0;
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
    if ( cnc.status.procedure.procID == procid_none )
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

static int internal_procedure_spindle_pwrdown_poll( struct SEventStruct *evt )
{
    if ( evt->fe_op_completed )
    {
        if ( evt->fe_op_failed )
        {
            cnc.status.procedure.procID = procid_none;
            cnc.status.flags.f.err_fatal = 1;
            cnc.status.flags.f.err_code = GENFAULT_FRONT_END;
            return -1;
        }
        else
        {
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
        }
    }
    return 0;
}






static void internal_send_simple_ack( struct ScmdIfCommand *cmd )
{
    struct ScmdIfResponse resp;
    resp.cmd_type = cmd->cmd_type;
    resp.resp_type = RESP_ACK;
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
}

static void internal_ob_helper_setstate_failed(void)
{
    cnc.status.flags.f.run_outband = 0;
    cnc.status.flags.f.run_ob_failed = 1;
}


int internal_stop( bool leave_ob )
{
    // stop motion core
    motion_sequence_stop();

    if ( (cnc.status.flags.f.err_fatal == 0) || (cnc.status.flags.f.err_code != GENFAULT_FRONT_END) )
    {
        // wait front-end to finish it's job
        if ( front_end_check_op_busy() )
        {
            front_end_terminate_touch_detection();
            front_end_sync_event_loop();
        }

        // spindle off - if no need for status preservation
        if ( cnc.status.flags.f.spindle_on && (leave_ob == false) )
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
        }
        else
        {
            struct SStepCoordinates *real_coord;
            motion_get_crt_coord( &cnc.status.procedure.params.getcoord.coord_snapshot );
            if (internal_procedure_coordinate_check_crt_coord( 1, true ))
                motion_set_crt_coord( &cnc.status.procedure.params.getcoord.coord_fe );
        }   
    }

    // reset status
    motion_get_crt_coord( &cnc.status.cmd.last_coord );
    cnc.status.cmd.last_cmdID = 0;

    if ( leave_ob == false )
    {
        cnc.status.flags.f.run_outband = 0;
        cnc.status.flags.f.run_program = 0;

        cnc.status.outband.command.cmd_type = 0;
        cnc.status.outband.state = 0;
        cnc.status.procedure.procID = procid_none;
    }

    // set axis power to auto
    motion_pwr_ctrl( COORD_X, mpwr_auto );
    motion_pwr_ctrl( COORD_Y, mpwr_auto );
    motion_pwr_ctrl( COORD_Z, mpwr_auto );
    motion_pwr_ctrl( COORD_A, mpwr_off );       // A channel is not used in current implementation

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

    if ( cnc.status.flags.f.run_outband || cnc.status.flags.f.run_program )
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

    if ( cnc.status.flags.f.run_outband || cnc.status.flags.f.run_program )
        return RESP_PEN;

    for (i=0; i<2; i++)
    {
        if ( cmd->cmd.probe_poz.coord[i] > cnc.setup.z_probe.coord[i] )
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


static inline int internal_outband_gohome( void )
{
    // bring the milling head in home/tool change position
    // stop the spindle if needed
    // fill up the followin sequences:
    //  - z top
    //  - x,y home poz
    //  - run sequence - ch
    if ( cnc.status.flags.f.run_outband || cnc.status.flags.f.run_program )
        return RESP_PEN;

    // set up the status of the procedure
    internal_ob_helper_setstate_started();

    cnc.status.outband.command.cmd_type = CMD_OBSA_GO_HOME;
    cnc.status.outband.params.gohome.m_fail = 0;
    cnc.status.outband.state = obstat_home_preparation;
    cnc.status.outband.params.gohome.ccheck = SEQ_PERIOD_COORD_CHK_OUTBAND;

    // power down the spindle for safety reason
    if ( cnc.status.flags.f.spindle_pwr )
        internal_procedure_spindle_pwrdown_entry(true);
    else
    {
        cnc.status.outband.state++;
        internal_ob_helper_gohome_setsequence( 1 );
    }

    // send acknowledge
    cnc.status.outband.command.cmd_type = CMD_OBSA_GO_HOME;
    internal_send_simple_ack( &cnc.status.outband.command );
    return RESP_ACK;
}


static inline int internal_outband_step( struct ScmdIfCommand *cmd )
{
    // step is a standalone simple outband with immediate action
    // no feedback is provided, stop command is recommended to detect miss step
    int i;
    if ( cnc.status.flags.f.run_outband || cnc.status.flags.f.run_program )
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
    if ( cnc.status.flags.f.run_program ||
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

    internal_stop(false);

    resp.resp.stop.cmdIDex = motion_sequence_crt_cmdID();
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
                res = internal_procedure_spindle_pwrdown_poll(evt);
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
                res = internal_procedure_coordinate_check_poll(evt);
                if ( res )
                {
                    if ( res == 1 )                                 // if coordinate readout is available from front-end
                    {
                        if ( internal_procedure_coordinate_check_crt_coord(SEQ_MAX_COORDDEV_OUTBAND, false) )
                        {
                            // missed step detected
                            cnc.status.flags.f.err_step = 1;
                            cnc.status.outband.params.gohome.m_fail += 100;     // failure ctr is decreased with 100 at each 10ms. so failure can be accumulated if problem isn't solved
                            if ( cnc.status.outband.params.gohome.m_fail > (SEQ_MAX_MOVEMENT_RETRIALS * 100) )
                            {
                                cnc.status.flags.f.err_code = GENFAULT_TABLE_STUCK;
                                cnc.status.flags.f.err_fatal = 1;
                                goto _error_exit;
                            }
                            // stop everything - but maintain procedure status
                            // this will updagte the coordinates also
                            internal_stop( true );
                            res = motion_sequence_crt_seqID();              // get the failed sequence id
                            if ( (res != 1) && (res != 2) )
                                goto _error_exit;
                            internal_ob_helper_gohome_setsequence( res );   // regenerate movement sequences
                            motion_sequence_start();                        // restart sequences
                        }
                        else if ( cnc.status.outband.params.gohome.m_fail )
                        {
                            cnc.status.outband.params.gohome.m_fail--;
                        }
                    }
                    else                                            // if coordinate readout failed
                        goto _error_exit;
                }

                // see if procedure is finished
                if ( motion_sequence_check_run()== false )
                {
                    // finish the command by stopping and updating everything
                    internal_stop( false );
                    internal_ob_helper_setstate_succeed();
                }

                // trigger coordinate check
                if ( evt->timer_tick_100us )
                {
                    cnc.status.outband.params.gohome.ccheck--;
                    if ( cnc.status.outband.params.gohome.ccheck == 0 )
                    {
                        cnc.status.outband.params.gohome.ccheck = SEQ_PERIOD_COORD_CHK_OUTBAND;
                        internal_procedure_coordinate_check_entry();
                    }
                }

                break;
        }
    }
    return;
_error_exit:
    internal_stop( false );                            // procedure failed
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
            internal_stop(false);
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
            internal_stop(false);
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

static inline internal_poll_ob_find_org( struct SEventStruct *evt )
{
    if ( evt->timer_tick_100us || evt->fe_op_completed )
    {
        int res;
        switch ( cnc.status.outband.state )
        {
            case obstat_forg_spindledn:
                // wait for spindle power down
                res = internal_procedure_spindle_pwrdown_poll(evt);
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
            case obstat_forg_search:
                if ( HW_FrontEnd_Event() )
                {
                    uint32 tmask = 0;

                    motion_sequence_stop();                 // stop all the runs
                    res = 0;
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
                break;
            case obstat_forg_finalize:
                if ( evt->fe_op_completed )
                {
                    if ( evt->fe_op_failed )
                        goto _fatal_error_exit;
                    else                                // front-end coordinates reset
                    {
                        motion_set_crt_coord( &cnc.setup.max_travel );
                        internal_stop(false);
                        internal_ob_helper_setstate_succeed();
                        return;
                    }
                }
                break;
        }
    }
    return;
_fatal_error_exit:
    cnc.status.flags.f.err_fatal = 1;
    cnc.status.flags.f.err_code = GENFAULT_FRONT_END;
_error_exit:
    internal_stop( false );                            // procedure failed
    internal_ob_helper_setstate_failed();
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
        resp.resp.inband.Qfree = internal_fifo_usable();
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
            case CMD_OBSA_SETUP_MAX_TRAVEL:     res = internal_outband_max_travel( cmd ); break;
            case CMD_OBSA_SETUP_MAX_SPEEDS:     res = internal_outband_max_speeds( cmd ); break;
            case CMD_OBSA_SETUP_HOME_POZ:       res = internal_outband_home_poz( cmd ); break;
            case CMD_OBSA_SETUP_PROBE_POZ:      res = internal_outband_probe_poz( cmd ); break;
            case CMD_OBSA_FIND_ORIGIN:          res = internal_outband_find_origin(); break;
            case CMD_OBSA_GO_HOME:              res = internal_outband_gohome(); break;
            case CMD_OBSA_FIND_Z_ZERO:
            case CMD_OBSA_STEP:                 res = internal_outband_step( cmd ); break;
            case CMD_OBSA_FREERUN:              res = internal_outband_freerun( cmd ); break;
            case CMD_OBSA_START:
            case CMD_OB_PAUSE:
            case CMD_OB_STOP:                   res = internal_outband_stop(); break;
            case CMD_OB_SCALE_FEED:
            case CMD_OB_SCALE_SPINDLE:
            case CMD_OB_GET_CRT_COORD:
            case CMD_OB_GET_CRT_CMD_ID:
            case CMD_OB_GET_STATUS:
            case CMD_OB_GET_PROBE_TOUCH:    break;
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
                resp.resp.inband.Qfree = internal_fifo_free();
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
    }

}


void seq_callback_process_inband( uint32 seqType, uint32 value )
{



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

    // setup defaults
    cnc.setup.max_travel.coord[ COORD_X ] = CNC_DEFAULT_X;
    cnc.setup.max_travel.coord[ COORD_Y ] = CNC_DEFAULT_Y;
    cnc.setup.max_travel.coord[ COORD_Z ] = CNC_DEFAULT_Z;
    cnc.setup.max_travel.coord[ COORD_A ] = CNC_DEFAULT_A;

    memset( &cnc.setup.z_probe, 0xff, sizeof(struct SStepCoordinates) );    // set all to -1 indicating uninitialized structure

    cnc.setup.home_poz      = cnc.setup.max_travel;
    cnc.setup.feed_rapid    = 1000;
    cnc.setup.feed_max      = 1500;

    // set up initial status
    cnc.status.cmd.last_coord = cnc.setup.max_travel;
    cnc.status.cmd.last_feed = 150;
    cnc.status.flags.f.stat_restarted = 1;

    // init motion core 
    motion_set_crt_coord( &cnc.setup.max_travel );          // consider starting with maximum coordinates
    motion_set_max_travel( &cnc.setup.max_travel ); 

    motion_pwr_ctrl( COORD_X, mpwr_auto );
    motion_pwr_ctrl( COORD_Y, mpwr_auto );
    motion_pwr_ctrl( COORD_Z, mpwr_auto );
    motion_pwr_ctrl( COORD_A, mpwr_off );       // A channel is not used in current implementation

    // register inband callback
    motion_sequence_register_callback( seq_callback_process_inband );

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
}

