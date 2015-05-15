#include <string.h>
#include "hw_stuff.h"
#include "events_ui.h"
#include "frontend.h"
#include "frontend_internals.h"
#include "comm_fe.h"


struct SFrontEndStruct fe;

const char cmd_spindle[] =  { 0xA9, 0x14, 0x00 };
const char cmd_getevents[] = { 0xAA, 0xA9 };            // has no cksum
const char cmd_setevmask[] = { 0xA9, 0x13, 0x00 };
const char cmd_setspeed[] = { 0xA9, 0x11, 0x00 };
const char cmd_getrpm[] = { 0xA9, 0x00 };
const char cmd_getcoord[] = { 0xAA, 0x9A };             // no checksum here also
const char cmd_resetcoord[] = { 0xA9, 0x05 };
const char cmd_getsignals[] = { 0xA9, 0x06 };
const char cmd_setrawrpm[] = { 0xa9, 0x12, 0x00 };

extern void stepper_get_coord( struct SStepCoordinates *coord );



/* *************************************************
 *
 *  internal routines
 *
 * *************************************************/

static int internal_cmd_send( void )
{
    commfe_flushResponse();
    fe.opstat.timeout_ctr = FE_MSG_TIMEOUT;
    if ( commfe_sendCommand(fe.opstat.out_msg, fe.opstat.out_msg_len) )
        return -1;
    return 0;
}


static int internal_cmd_response_poll( bool tick100us )
{
    uint32 res;
    bool req_retry = false;

    res = commfe_checkResponse( fe.opstat.out_resp_len );
    if ( res == COMMFE_PENDING )
    { 
        if (tick100us)
        {
            fe.opstat.timeout_ctr--;
            if ( fe.opstat.timeout_ctr == 0 )
                req_retry = true;
        }
    }
    else if (res == COMMFE_NAK)
    {
        req_retry = true;
    }
    else
    {
        // ack received - message OK
        return 1;
    }

    if ( req_retry )
    {
        if ( fe.opstat.retries )
        {
            // retry the message
            internal_cmd_send();
            fe.opstat.retries--;
        }
        else
        {
            // out of retrials - giving up
            return -1;
        }
    }
    return 0;   // still pending
}


int internal_event_loop(void)
{
    struct SEventStruct evt;
    int res;

    fe.opstat.retries = FE_MSG_RETRY;
    do
    {
        StepDBG_QT_innerLoop();

        evt = Event_Poll();

        res = internal_cmd_response_poll( evt.timer_tick_100us ? true : false );

        Event_Clear(evt);
    } while ( res == 0 );

    if (res == 1)
    {
        res = 0;
    }

    return res;
}


static int internal_command_spindle_pwr( bool enable )
{
    memcpy( fe.opstat.out_msg, cmd_spindle, sizeof(cmd_spindle) );
    fe.opstat.out_msg_len = sizeof(cmd_spindle);
    fe.opstat.out_resp_len = 1;
    if ( enable )
        fe.opstat.out_msg[2] |= 0x40;
    else 
        fe.opstat.out_msg[2] |= 0x04;

    return internal_cmd_send();
}


static int internal_command_get_events( void )
{
    memcpy( fe.opstat.out_msg, cmd_getevents, sizeof(cmd_getevents) );
    fe.opstat.out_msg_len = sizeof(cmd_getevents);
    fe.opstat.out_resp_len = 3;
    return internal_cmd_send();
}

static int internal_command_set_flags( uint8 flags )
{
    memcpy( fe.opstat.out_msg, cmd_setevmask, sizeof(cmd_setevmask) );
    fe.opstat.out_msg_len = sizeof(cmd_setevmask);
    fe.opstat.out_resp_len = 1;
    fe.opstat.out_msg[2] = flags;
    return internal_cmd_send();
}

static int internal_command_set_spindle_speed( uint8 speed )
{
    memcpy( fe.opstat.out_msg, cmd_setspeed, sizeof(cmd_setspeed) );
    fe.opstat.out_msg_len = sizeof(cmd_setspeed);
    fe.opstat.out_resp_len = 1;
    fe.opstat.out_msg[2] = speed;
    return internal_cmd_send();
}

static int internal_command_get_rpm( void )
{
    memcpy( fe.opstat.out_msg, cmd_getrpm, sizeof(cmd_getrpm) );
    fe.opstat.out_msg_len = sizeof(cmd_getrpm);
    fe.opstat.out_resp_len = 3;
    return internal_cmd_send();
}

static int internal_command_get_coordinates( void )
{
    memcpy( fe.opstat.out_msg, cmd_getcoord, sizeof(cmd_getcoord) );
    fe.opstat.out_msg_len = sizeof(cmd_getcoord);
    fe.opstat.out_resp_len = 9;
    return internal_cmd_send();
}

static int internal_command_reset_coord( void )
{
    memcpy( fe.opstat.out_msg, cmd_resetcoord, sizeof(cmd_resetcoord) );
    fe.opstat.out_msg_len = sizeof(cmd_resetcoord);
    fe.opstat.out_resp_len = 1;
    return internal_cmd_send();
}

static int internal_command_get_signals( void )
{
    memcpy( fe.opstat.out_msg, cmd_getsignals, sizeof(cmd_getsignals) );
    fe.opstat.out_msg_len = sizeof(cmd_getsignals);
    fe.opstat.out_resp_len = 7;
    return internal_cmd_send();
}

static int internal_command_set_raw_rpm( uint8 val )
{
    memcpy( fe.opstat.out_msg, cmd_setrawrpm, sizeof(cmd_setrawrpm) );
    fe.opstat.out_msg_len = sizeof(cmd_setrawrpm);
    fe.opstat.out_resp_len = 1;
    fe.opstat.out_msg[2] = val;
    return internal_cmd_send();
} 


/*--------------------------
 *  Poll loop routines
 *-------------------------*/

static inline void local_poll_spindle_pwr( struct SEventStruct *evt )
{
    int res;

    res = internal_cmd_response_poll( evt->timer_tick_100us ? true : false );
    if ( res == 0 )         // pending
        return;
        
    fe.op = feop_none;
    evt->fe_op_completed = 1;   // mark for completed

    if ( res == -1 )
    {
        evt->fe_op_failed = 1;  // mark for failed
        return;
    }
    // if not failed - save the new state
    fe.status.spindle_on = fe.opstat.p.sp_pwr;
}


static inline void local_poll_sindle_rpm( struct SEventStruct *evt )
{   
    int res;

    if ( fe.opstat.phase == opstat_spsp_wait_flag )         // special case - we are waiting for a flag, not for a message
    {
        // if waiting for event flag
        if ( HW_FrontEnd_Event() )          // flag received
        {
            // set evmask for detecting spindle jam only
            if ( internal_command_set_flags( FE_EV_SPINDLE_JAM ) )
                goto _error_exit;
            fe.opstat.retries = FE_MSG_RETRY;
            fe.opstat.phase++;
        }
        else if ( evt->timer_tick_100us )
        {
            if ( fe.opstat.timeout_ctr )
                fe.opstat.timeout_ctr--;
            else
                goto _error_exit;
        }

    }
    else
    {
        // waiting for message response
        res = internal_cmd_response_poll( evt->timer_tick_100us ? true : false );
        if ( res == 0 )         // pending
            return;
        if ( res == -1 )        // failure
            goto _error_exit;

        // ok - set up the next action
        switch ( fe.opstat.phase )
        {
            case opstat_spsp_reset_flags:       // flags are reset
                // set evmask for detecting spindle jam and spindle ok
                if ( internal_command_set_flags( FE_EV_SPINDLE_JAM | FE_EV_SPINDLE_OK ) )
                    goto _error_exit;
                fe.opstat.retries = FE_MSG_RETRY;
                fe.opstat.phase++;
                break;
            case opstat_spsp_set_evmask1:       // event mask is set up for spindle jam and ok
                // set spindle speed command
                if ( internal_command_set_spindle_speed( fe.opstat.p.sp_rpm >> 8 ) )
                    goto _error_exit;
                fe.opstat.retries = FE_MSG_RETRY;
                fe.opstat.phase++;
                break;
            case opstat_spsp_set_speed:         // speed is set up
                // set up wait for event flag for ok or jammed
                fe.opstat.timeout_ctr = 30000;  // 3sec timeout for spin-up timeout
                fe.opstat.phase++;
                break;
            case opstat_spsp_set_evmask2:       // event mask set for jam only
                // send request for event list
                if ( internal_command_get_events() )
                    goto _error_exit;
                fe.opstat.retries = FE_MSG_RETRY;
                fe.opstat.phase++;
                break;
            case opstat_spsp_check_events:      // events received
                // check if spindle is jammed or is OK
                commfe_getResponse( fe.opstat.out_msg, 3 );
                if (fe.opstat.out_msg[1] & FE_EV_SPINDLE_JAM) 
                {
                    evt->fe_spindle_jam = 1;
                    goto _error_exit;
                }
                if ((fe.opstat.out_msg[1] & FE_EV_SPINDLE_OK) == 0)
                    goto _error_exit;
                // finished with success
                fe.op = feop_none;
                fe.status.spindle_rpm_set = fe.opstat.p.sp_rpm;
                evt->fe_op_completed = 1;   // mark for completed
                break;
        }
    }

    return;
_error_exit:
    fe.op = feop_none;
    evt->fe_op_completed = 1;   // mark for completed
    evt->fe_op_failed = 1;      // mark for failed
    return;
}


static inline void local_poll_sindle_read_rpm( struct SEventStruct *evt )
{
    int res;

    res = internal_cmd_response_poll( evt->timer_tick_100us ? true : false );
    if ( res == 0 )         // pending
        return;

    fe.op = feop_none;
    evt->fe_op_completed = 1;   // mark for completed

    if ( res == -1 )
    {
        evt->fe_op_failed = 1;  // mark for failed
        return;
    }

    // if not failed - get the rpm
    {
        uint16 rpm;
        commfe_getResponse( fe.opstat.out_msg, 3 );
        rpm = (fe.opstat.out_msg[1] | (fe.opstat.out_msg[2] << 8));
        fe.status.spindle_rpm_get = rpm;
    }
}


static inline void local_poll_coordinates_get( struct SEventStruct *evt )
{
    int res;

    res = internal_cmd_response_poll( evt->timer_tick_100us ? true : false );
    if ( res == 0 )         // pending
        return;

    fe.op = feop_none;
    evt->fe_op_completed = 1;   // mark for completed

    if ( res == -1 )
    {
        evt->fe_op_failed = 1;  // mark for failed
        return;
    }

    // if not failed - get the rpm
    {
        // [ACK+9][xxxx xxxx][xxxx xxxx][xxxx yyyy][yyyy yyyy][yyyy yyyy][zzzz zzzz][zzzz zzzz][zzzz 0sss][cksum]
        int x = 0;
        int y = 0;
        int z = 0;

        commfe_getResponse( fe.opstat.out_msg, 9 );
        //   [0000 xxxx][xxxx xxxx][xxxx xxxx]
        x = (fe.opstat.out_msg[1] << 12) | (fe.opstat.out_msg[2] << 4) | (fe.opstat.out_msg[3] >> 4);
        y = ( (fe.opstat.out_msg[3] & 0x0f) << 16) | (fe.opstat.out_msg[4]<<8) | (fe.opstat.out_msg[5]);
        z = (fe.opstat.out_msg[6] << 12) | (fe.opstat.out_msg[7] << 4) | (fe.opstat.out_msg[8] >> 4);

        if ( fe.opstat.out_msg[8] & 0x04 )
            x = -x;
        if ( fe.opstat.out_msg[8] & 0x02 )
            y = -y;
        if ( fe.opstat.out_msg[8] & 0x01 )
            z = -z;
        
        fe.status.coord.coord[ COORD_X ] = x;
        fe.status.coord.coord[ COORD_Y ] = y;
        fe.status.coord.coord[ COORD_Z ] = z;
        fe.status.coord.coord[ COORD_A ] = 0;
        fe.status.coord_updated = true;
    }
}


static inline void local_poll_simple_command_completion( struct SEventStruct *evt )
{
    int res;

    res = internal_cmd_response_poll( evt->timer_tick_100us ? true : false );
    if ( res == 0 )         // pending
        return;

    fe.op = feop_none;
    evt->fe_op_completed = 1;   // mark for completed

    if ( res == -1 )
    {
        evt->fe_op_failed = 1;  // mark for failed
        return;
    }
}


static inline void local_poll_touch( struct SEventStruct *evt )
{
    int res;

    if ( fe.opstat.phase == opstat_tch_wait_flag )         // special case - we are waiting for a flag, not for a message
    {
        // if waiting for event flag
        if ( HW_FrontEnd_Event() || (fe.opstat.p.touch.set_mask == 0) )          // flag received (or operation cancelled)
        {
            // set evmask for detecting spindle jam only
            if ( internal_command_set_flags( FE_EV_SPINDLE_JAM ) )
                goto _error_exit;
            fe.opstat.retries = FE_MSG_RETRY;
            fe.opstat.phase++;
        }
    }
    else
    {
        // waiting for message response
        res = internal_cmd_response_poll( evt->timer_tick_100us ? true : false );
        if ( res == 0 )         // pending
            return;
        if ( res == -1 )        // failure
            goto _error_exit;

        // ok - set up the next action
        switch ( fe.opstat.phase )
        {
            case opstat_tch_set_evmask:       // evmask for touch detection is set up
                // reset the current events 
                if ( internal_command_get_events() )
                    goto _error_exit;
                fe.opstat.retries = FE_MSG_RETRY;
                fe.opstat.phase++;
                break;
            case opstat_tch_reset_events:   // reset event completed
                // set up wait for event flag for ok or jammed
                fe.opstat.timeout_ctr = 0;  // no time-out since we don't know how long it takes
                fe.opstat.phase++;
                break;
            case opstat_tch_set_org_evmask: // original event mask set up
                // send request for event list
                if ( internal_command_get_events() )
                    goto _error_exit;
                fe.opstat.retries = FE_MSG_RETRY;
                fe.opstat.phase++;
                break;
            case opstat_tch_get_events:     // events received
                commfe_getResponse( fe.opstat.out_msg, 3 );
                {
                    uint32 ev_flags = fe.opstat.out_msg[1];
                    uint32 ev_sig   = fe.opstat.out_msg[2];

                    if ( (ev_flags & FE_EV_PROBE) == 0 )        // if no probe event, clear probe signal
                        ev_sig &= ~FE_TOUCH_PROBE;
                    if ( (ev_flags & FE_EV_A_AXIS) == 0 )       // if no a axis null, clear a axis null signal
                        ev_sig &= ~FE_TOUCH_AXIS_A;
                    if ( (ev_flags & FE_EV_ENDPOINT) == 0 )     // if no endpoint event, clear the endpoint signals
                        ev_sig &= ~( FE_TOUCH_AXIS_X | FE_TOUCH_AXIS_Y | FE_TOUCH_AXIS_Z );
                    
                    ev_sig &= fe.opstat.p.touch.set_mask;       // clear the unrequested signals
                    fe.status.touch_mask = ev_sig;
                }
                // finished with success
                fe.op = feop_none;
                evt->fe_op_completed = 1;   // mark for completed
                break;
        }
    }

    return;
_error_exit:
    fe.op = feop_none;
    evt->fe_op_completed = 1;   // mark for completed
    evt->fe_op_failed = 1;      // mark for failed
    return;
}


/* *************************************************
 *
 *  main routines
 *
 * *************************************************/

    int front_end_init( void )
    {
        memset( &fe, 0, sizeof(fe) );
        fe.status.spindle_rpm_get = FE_RPM_INVALID;

        commfe_init();

        // set event flags for spindle jam detection
        if ( internal_command_set_flags( FE_EV_SPINDLE_JAM ) )
            goto _error_exit;
        if ( internal_event_loop() )
            goto _error_exit;

        // power down the spindle assembly
        if ( internal_command_spindle_pwr(false) )
            goto _error_exit;
        if ( internal_event_loop() )
            goto _error_exit;

        // set spindle speed 0
        if ( internal_command_set_spindle_speed(0) )
            goto _error_exit;
        if ( internal_event_loop() )
            goto _error_exit;

        // reset coordinates
        if ( internal_command_reset_coord() )
            goto _error_exit;
        if ( internal_event_loop() )
            goto _error_exit;

        // reset events
        if ( internal_command_get_events() )
            goto _error_exit;
        if ( internal_event_loop() )
            goto _error_exit;

        fe.in_use = true;
        return 0;

    _error_exit:
        return -1;
    }


    void front_end_poll( struct SEventStruct *evt )
    {
        if ( fe.op )
        {
            if ( fe.no_change )
            {
                fe.no_change = false;
                evt->fe_op_completed = 1;
                fe.op = feop_none;
            }
            else switch ( fe.op )
            {
                case feop_spindle_pwr:
                    local_poll_spindle_pwr(evt);
                    break;
                case feop_spindle_set_rpm:
                    local_poll_sindle_rpm(evt);
                    break;
                case feop_spindle_get_rpm:
                    local_poll_sindle_read_rpm(evt);
                    break;
                case feop_coord_get:
                    local_poll_coordinates_get(evt);
                    break;
                case feop_coord_reset:
                    local_poll_simple_command_completion(evt);
                    break;
                case feop_touch_detect:
                    local_poll_touch(evt);
                    break;
                default:
                    break;
            }
        }
    }


    bool front_end_check_op_busy( void )
    {
        if ( fe.op )
            return true;
        return false;
    }


/* *************************************************
 *
 *  Interface routines
 *
 * *************************************************/


    int front_end_spindle_power( bool enable )
    {
        if ( fe.op )
            return -1;

        if ( (fe.status.spindle_on != enable) && (fe.in_use ) )
        {
            if ( internal_command_spindle_pwr( enable ) )
                return -1;
            fe.op = feop_spindle_pwr;
            fe.opstat.retries = FE_MSG_RETRY;
            fe.opstat.p.sp_pwr = enable;
        }
        else
        {
            fe.op = feop_spindle_pwr;
            fe.status.spindle_on = enable;
            fe.no_change = true;
        }
        return 0;
    }


    int front_end_spindle_speed( uint32 speed )
    {
        if ( fe.op )
            return -1;

        if ( fe.in_use )
        {
            if ( internal_command_get_events() )    // send dummy call for event clean-up
                return -1;
            fe.op = feop_spindle_set_rpm;
            fe.opstat.phase = opstat_spsp_reset_flags;
            fe.opstat.retries = FE_MSG_RETRY;
            fe.opstat.p.sp_rpm = speed;
        }
        else
        {
            fe.op = feop_spindle_set_rpm;
            fe.status.spindle_rpm_set = speed;
            fe.no_change = true;
        }
        return 0;
    }


    int front_end_request_rpm( void )
    {
        if ( fe.op )
            return -1;

        if ( fe.in_use )
        {
            if ( internal_command_get_rpm() )
                return -1;
            fe.op = feop_spindle_get_rpm;
            fe.opstat.retries = FE_MSG_RETRY;
            fe.status.spindle_rpm_get = FE_RPM_INVALID;     // indicating invalid rpm
        }
        else
        {
            fe.op = feop_spindle_set_rpm;
            fe.status.spindle_rpm_get = fe.status.spindle_rpm_set;
            fe.no_change = true;
        }
        return 0;
    }


    int front_end_get_rpm( uint32 *rpm )
    {
        if ( fe.status.spindle_rpm_get != FE_RPM_INVALID )
        {
            *rpm = fe.status.spindle_rpm_get;
            fe.status.spindle_rpm_get = FE_RPM_INVALID;
            return 0;
        }
        *rpm = 0;
        return -1;
    }
    

    int front_end_request_coordinates( void )
    {
        if ( fe.op )
            return -1;

        if ( fe.in_use )
        {
            if ( internal_command_get_coordinates() )
                return -1;
            fe.op = feop_coord_get;
            fe.opstat.retries = FE_MSG_RETRY;
            fe.status.coord_updated = false;     // indicating invalid coordinate
        }
        else
        {
            fe.op = feop_spindle_set_rpm;
            stepper_get_coord( &fe.status.coord ); 
            fe.status.coord_updated = true;
            fe.no_change = true;
        }
        return 0;
    }


    struct SStepCoordinates *front_end_get_coordinates( void )
    {
        if ( fe.status.coord_updated )
        {
            fe.status.coord_updated = false;
            return &fe.status.coord;
        }
        return NULL;
    }


    bool front_end_is_coordinate_request_sent( void )
    {
        if ( fe.in_use )
        {
            if ( (fe.op == feop_coord_get) && commfe_is_outfifo_empty() )
                return true;
            return false;
        }
        return true;
    }


    int front_end_coordinate_reset_to_max( void )
    {
        if ( fe.op )
            return -1;

        if ( fe.in_use )
        {
            if ( internal_command_reset_coord() )
                return -1;
            fe.op = feop_coord_reset;
            fe.opstat.retries = FE_MSG_RETRY;
        }
        else
        {
            fe.op = feop_coord_reset;
            fe.no_change = true;
        }
        return 0;
    }


    int front_end_request_touch_detection( uint32 touch_mask )
    {
        if ( fe.op )
            return -1;

        if ( fe.in_use )
        {
            uint32 mask = 0;
            if ( touch_mask & FE_TOUCH_PROBE )
                mask = FE_EV_PROBE;
            if ( touch_mask & (FE_TOUCH_AXIS_Z | FE_TOUCH_AXIS_Y | FE_TOUCH_AXIS_X) )
                mask |= FE_EV_ENDPOINT;
            if ( touch_mask & FE_TOUCH_AXIS_A )
                mask |= FE_EV_A_AXIS;

            if ( internal_command_set_flags( mask ) )
                return -1;
            fe.op = feop_touch_detect;
            fe.opstat.phase = opstat_tch_set_evmask;
            fe.opstat.retries = FE_MSG_RETRY;
            fe.opstat.p.touch.set_mask = touch_mask;
            fe.status.touch_mask = 0;
        }
        else
        {
            fe.op = feop_touch_detect;
            fe.status.touch_mask = touch_mask;
            fe.no_change = true;
        }
        return 0;
    }


    int front_end_get_touched_list( uint32 *touch_mask )
    {
        if ( fe.op )
            return -1;
        *touch_mask = fe.status.touch_mask;
        fe.status.touch_mask = 0;
        return 0;
    }


    int front_end_terminate_touch_detection( void )
    {
        int res = 0;
        if (fe.op == feop_touch_detect) 
        {
            if (fe.opstat.phase == opstat_tch_wait_flag )
            {
                // set evmask for detecting spindle jam only
                if ( internal_command_set_flags( FE_EV_SPINDLE_JAM ) )      // set back the original jam detection
                    res = -1;
                fe.opstat.retries = FE_MSG_RETRY;
                fe.opstat.phase++;
                fe.opstat.p.touch.set_mask = 0;     // clear mask - will not output any flag
            }
            return res;
        }
        return -1;
    }


    int front_end_dbg_get_event_list( uint8 *evt )
    {
        if ( (fe.op) || (fe.in_use == 0) )
            return -1;

        // send request for events
        if ( internal_command_get_events() )
            return -1;
        // wait for response
        if ( internal_event_loop() )
            return -1;
        // read the data
        commfe_getResponse( fe.opstat.out_msg, 3 );
        evt[0] = fe.opstat.out_msg[1];
        evt[1] = fe.opstat.out_msg[2];
        return 0;
    }


    int front_end_dbg_get_signals( uint8 *sig )
    {
        if ( (fe.op) || (fe.in_use == 0) )
            return -1;

        // send request for hardware signals
        if ( internal_command_get_signals() )
            return -1;
        // wait for response
        if ( internal_event_loop() )
            return -1;
        // read the data
        commfe_getResponse( fe.opstat.out_msg, 7 );

        memcpy( sig, fe.opstat.out_msg+1, 6 );
        return 0;
    }


    int front_end_dbg_set_raw_spindle( uint32 raw_val )
    {
        if ( (fe.op) || (fe.in_use == 0) )
            return -1;

        // send raw rpm value
        if ( internal_command_set_raw_rpm( raw_val ) )
            return -1;
        // wait for response
        if ( internal_event_loop() )
            return -1;

        return 0;
    }


    int front_end_sync_event_loop(void)
    {
        struct SEventStruct evt;

        do
        {
            StepDBG_QT_innerLoop();

            evt = Event_Poll();

            front_end_poll( &evt );

            Event_Clear(evt);

        } while ( evt.fe_op_completed == 0 );

        if ( evt.fe_op_failed )
            return -1;
        return 0;
    }

