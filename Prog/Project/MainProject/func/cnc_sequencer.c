#include <string.h>

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
 *
 */


struct SCNCSequencerInternals   cnc;








void seq_callback_process_inband( uint32 seqType, uint32 value )
{



}


void local_sequencer_process_command( void )
{
    struct ScmdIfCommand cmd;
    struct ScmdIfResponse resp;
    int res;

    res = cmdif_get_command( &cmd );
    if ( res == -2 )        // if bulk command
    {
        do
        {
            res = cmdif_get_bulk( &cmd );

        } while ( res == 0 );

    }

    resp.cmd_type = cmd.cmd_type;
    resp.resp_type = RESP_ACK;
    cmdif_confirm_reception( &resp );

}

/* *************************************************
 *
 *  Interface routines
 *
 * *************************************************/


void sequencer_init()
{
    memset( &cnc, 0, sizeof(cnc) );

    motion_init();
    if ( front_end_init() == 0 )
        cnc.setup.fe_present = true;
    cmdif_init();

    // init motion core 
    struct SStepCoordinates origin;
    origin.coord[ COORD_X ] = CNC_DEFAULT_X;
    origin.coord[ COORD_Y ] = CNC_DEFAULT_Y;
    origin.coord[ COORD_Z ] = CNC_DEFAULT_Z;
    origin.coord[ COORD_A ] = CNC_DEFAULT_A;
    motion_set_crt_coord( &origin );

    motion_pwr_ctrl( COORD_X, mpwr_auto );
    motion_pwr_ctrl( COORD_Y, mpwr_auto );
    motion_pwr_ctrl( COORD_Z, mpwr_auto );
    motion_pwr_ctrl( COORD_A, mpwr_off );       // A channel is not used in current implementation

    // register inband callback
    motion_sequence_register_callback( seq_callback_process_inband );

    
}


void sequencer_poll( struct SEventStruct *evt )
{
    front_end_poll(evt);
    cmdif_poll(evt);
    if ( evt->comm_command_ready )
    {
        local_sequencer_process_command();
    }
    

    motion_poll(evt);
}

