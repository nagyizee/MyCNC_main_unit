#include <string.h>
#include "hw_stuff.h"
#include "events_ui.h"
#include "frontend.h"
#include "frontend_internals.h"
#include "comm_fe.h"


struct SFrontEndStruct fe;


/* *************************************************
 *
 *  internal routines
 *
 * *************************************************/

static int internal_command_spindle_pwr( bool enable )
{
    char fecmd[5] = { 0xA9, 0x14, 0x00, 0xEE }; // Spindle speed 0: 0xa9 0x11 0x00 0x00

    if ( enable )
        fecmd[2] |= 0x40;
    else 
        fecmd[2] |= 0x04;

    commfe_flushResponse();
    if ( commfe_sendCommand(fecmd, 3) )
        return -1;
    return 0;
}





/*--------------------------
 *  Poll loop routines
 *-------------------------*/

static inline void local_poll_spindle_pwr( struct SEventStruct *evt )
{
    uint32 res;
    bool req_retry = false;

    res = commfe_checkResponse(1);
    if ( (res == COMMFE_PENDING) && (evt->timer_tick_100us) )
    { 
        fe.opstat.timeout_ctr--;
        if ( fe.opstat.timeout_ctr == 0 )
            req_retry = true;
    }
    else if (res == COMMFE_NAK)
    {
        req_retry = true;
    }
    else
    {
        // ack received - operation finished
        fe.op = feop_none;
        evt->fe_op_completed = 1;
        fe.status.spindle_on = fe.opstat.p.sp_pwr;
        return;
    }

    if ( req_retry )
    {
        if ( fe.opstat.retries )
        {
            // retry the message
            internal_command_spindle_pwr( fe.opstat.p.sp_pwr );
            fe.opstat.timeout_ctr = FE_MSG_TIMEOUT;
            fe.opstat.retries--;
        }
        else
        {
            // out of retrials - giving up
            fe.op = feop_none;
            evt->fe_op_completed = 1;
            evt->fe_op_failed = 1;
        }
    }
}




/* *************************************************
 *
 *  main routines
 *
 * *************************************************/

    int front_end_init( void )
    {
        memset( &fe, 0, sizeof(fe) );
        return 0;
    }


    void front_end_poll( struct SEventStruct *evt )
    {
        if ( fe.op )
        {
            if ( fe.no_change )
            {
                evt->fe_op_completed = 1;
                fe.op = feop_none;
            }
            else switch ( fe.op )
            {
                case feop_spindle_pwr:
                    local_poll_spindle_pwr(evt);
                    break;

            }
        }
    }


    bool front_end_chek_op_busy( void )
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

        fe.op = feop_spindle_pwr;
        if ( (fe.status.spindle_on != enable) && (fe.in_use ) )
        {
            internal_command_spindle_pwr( enable );
            fe.opstat.phase = opst_spinpwr_wait_ack;
            fe.opstat.retries = 5;
            fe.opstat.timeout_ctr = FE_MSG_TIMEOUT;
            fe.opstat.reserved = 0;     // !!!!- SEE if compilator traduces this in 1 single store operation
            fe.opstat.p.sp_pwr = enable;
        }
        else
        {
            fe.status.spindle_on = enable;
            fe.no_change = true;
        }
        return 0;
    }


    int front_end_spindle_speed( uint32 speed )
    {

        return 0;
    }


    int front_end_check_spindle_ok( void )
    {

        return 0;
    }


    int front_end_get_rpm( uint32 *rpm )
    {

        return 0;
    }


    int front_end_request_coordinates( void )
    {
        
        return 0;
    }


    struct SStepCoordinates *front_end_get_coordinates( void )
    {

        return NULL;
    }


    bool front_end_is_coordinate_request_sent( void )
    {

        return false;
    }


    int front_end_coordinate_reset_to_max( void )
    {

        return 0;
    }


    int front_end_request_touch_detection( uint32 touch_mask )
    {

        return 0;
    }


    int front_end_get_touched_list( uint32 *touch_mask )
    {

        return 0;
    }


    int front_end_dbg_get_event_list( uint8 *evt )
    {

        return 0;
    }


    int front_end_dbg_get_signals( uint8 *sig )
    {

        return 0;
    }


    int front_end_dbg_set_raw_spindle( uint32 raw_val )
    {

        return 0;
    }

