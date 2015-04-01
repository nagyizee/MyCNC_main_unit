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
 *  At start-up or reset:
 *      The cnc sequencer is initted with default max poz which is considered as max travel also.
 *
 *
 *
 *
 *
 *
 */


struct SCNCSequencerInternals   cnc;



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

    elem = &cnc.cmd_fifo[ cnc.cmd_fifo.r++ ];
    cnc.cmd_fifo.r %= CNCSEQ_CMD_FIFO_SIZE;
    cnc.cmd_fifo.rdb--;
    return elem;
}

struct ScmdIfCommand *internal_fifo_peek_readable(void)
{
    if ( cnc.cmd_fifo.rdb == 0 )
        return NULL;
    return &cnc.cmd_fifo[ cnc.cmd_fifo.r ];
}

struct ScmdIfCommand *internal_fifo_peek_eraseable(void)
{
    if ( cnc.cmd_fifo.e == cnc.cmd_fifo.r )
        return NULL;
    return &cnc.cmd_fifo[ cnc.cmd_fifo.e ];
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
    return elem;
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
                 ((pcmd->cmd.ib_drill.coord.coord[COORD_Z] + pcmd->cmd.ib_drill.clearance) > cnc.setup.max_travel.coord[COORD_Z]) ||
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


}


void local_sequencer_process_command( void )
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
        resp.cmd_type = CMD_OBSA_RESET; //dummy 
        cmdif_confirm_reception( &resp );
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


void sequencer_init()
{
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

    cnc.setup.feed_rapid    = 1000;
    cnc.setup.feed_max      = 1500;

    cnc.status.cmd.last_coord = cnc.setup.max_travel;
    cnc.status.cmd.last_feed = 150;


    // init motion core 
    motion_set_crt_coord( &setup.max_travel );          // consider starting with maximum coordinates

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
    motion_poll(evt);
    if ( evt->comm_command_ready )
    {
        local_sequencer_process_command();
    }


}

