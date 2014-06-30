#include <stdio.h>
#include <math.h>
#include "motion_core.h"
#include "motion_core_internals.h"
#include "events_ui.h"


struct SMotionCoreInternals core;



/* *************************************************
 *
 *  Internal routines
 *
 * *************************************************/


/*--------------------------
 *  Fifo routines
 *-------------------------*/

static int internal_seq_fifo_insert( struct SMotionSequence *seq )
{
    if ( core.seq_fifo.c >= MAX_SEQ_FIFO )
        return -1;

    core.seq_fifo.seq[ core.seq_fifo.w++ ] = *seq;
    if ( core.seq_fifo.w == MAX_SEQ_FIFO )
        core.seq_fifo.w = 0;
    core.seq_fifo.c++;
    return 0;
}


static int internal_seq_fifo_get( struct SMotionSequence *seq )
{
    if ( core.seq_fifo.c == 0 )
        return -1;

    *seq = core.seq_fifo.seq[ core.seq_fifo.r++ ];

    if ( core.seq_fifo.r == MAX_SEQ_FIFO )
        core.seq_fifo.r = 0;
    core.seq_fifo.c--;
    return 0;
}


static int internal_seq_fifo_peek( struct SMotionSequence **seq, bool next )
{
    int idx;
    if ( (core.seq_fifo.c == 0) || ( next && (core.seq_fifo.c == 1)) )
        return -1;

    idx = core.seq_fifo.r;
    if ( next )
    {
        idx++;
        if ( idx == MAX_SEQ_FIFO )
            idx = 0;
    }

    *seq = &core.seq_fifo.seq[ idx ];
    return 0;
}


static void internal_seq_fifo_flush( void )
{
    core.seq_fifo.c = 0;
    core.seq_fifo.w = 0;
    core.seq_fifo.r = 0;
}


static inline int internal_seq_fifo_fullness( void )
{
    return core.seq_fifo.c;
}



/*--------------------------
 *  Sequencer routines
 *-------------------------*/

static inline uint32 internal_run_helper_get_distance( TStepCoord val1, TStepCoord val2, uint32 *dist )
{
    if (val2 > val1)
    {
        *dist = val2 - val1;
        return 1;
    }
    else
    {
        *dist = val1 - val2;
        return 0;
    }
}


#define FP32      32
#define FP16      16

#define DASSERT(x)      do {   if ( !(x) ) \
                                sprintf( stderr, "Assert: %s\n" # x );\
                        } while(0);


static void internal_run_goto( struct SMotionSequence *seq )
{
    uint32 dists[CNC_MAX_COORDS];
    int i;

    struct SMotionSequence *next_seq = NULL;


    // peek the next sequence for checking the speeds for speed difference
    if ( internal_seq_fifo_peek( &next_seq, false) == 0 )
    {
        if ( next_seq->seqType != SEQ_TYPE_GOTO )
            next_seq = NULL;
    }

    // get the distances and direction on each axis
    core.status.op.go_to.dirmask = 0;
    for ( i=0; i<CNC_MAX_COORDS; i++ )
    {
        core.status.op.go_to.dirmask |= ( internal_run_helper_get_distance( core.crt_coord.coord[i], seq->params.go_to.coord.coord[i], &dists[i] ) << i );
    }

    // calculate overall lenght and times
    {
        uint64 sum = 0LL;
        double L = 0;
        double T = 0;
        uint64 Ti = 0;

        // L = sqrt( x*x + y*y + z*z + a*a )
        for ( i=0; i<CNC_MAX_COORDS; i++ )
        {
            sum += ((uint64)dists[i]) * ((uint64)dists[i]);
        }

        // T = L / sps_speed -> how many seconds we should step
        // T = ( L * STEP_SEC ) / sps_speed -> how many step units we should step. This should be < 1
        // t[i] = T / dist

        L = sqrt( sum );                    // total lenght in steps (in fp32)
        T = L / seq->params.go_to.feed;     // Total time in seconds

        // constraints: 2h with 20kstep -> 32bit, in fp3 -> 64bit
        // TODO: - In the sequence generator take care for low feed speeds with long distances - break them down in multi-sequence
        DASSERT( T < (8*60*400*20000) );

        Ti = (uint64)( T * ( (1LL<<FP32) * STEP_SEC ) );         // Total time in sysclocks in fp32

        // calculate step speed increment on each axis
        for ( i=0; i<CNC_MAX_COORDS; i++ )
        {
            if ( dists[i] )
                core.status.op.go_to.StepCkInc[i] = Ti / (dists[i]+1);        // nr. of sysclock / step in fp.32
            else
                core.status.op.go_to.StepCkInc[i] = Ti;

            DASSERT( core.status.op.go_to.StepCkInc[i] >= (1LL<<FP32) );
            core.status.op.go_to.StepCkCntr[i] = core.status.op.go_to.StepCkInc[i];
        }
    }

    core.status.is_running = true;
    core.status.crt_seq = *seq;
    core.status.op.go_to.Tctr = 0;

    return;
}




static void internal_seq_run( struct SMotionSequence *seq, bool in_run )
{
    switch ( seq->seqType )
    {
        case SEQ_TYPE_GOTO:
            internal_run_goto( seq );
            break;
        case SEQ_TYPE_HOLD:
            break;
        case SEQ_TYPE_SPINDLE:
            break;
    }

}



static inline void internal_seqpoll_goto( void )
{
    if ( stepper_getQ() >= MAX_ISR_STEPS )
        return;

    uint32 endmask = 0;
    uint32 stepmask = 0;
    uint64 Tctr_fp32;
    int i;

    // increment sysclock counter
    core.status.op.go_to.Tctr++;
    Tctr_fp32 = ((uint64)core.status.op.go_to.Tctr << FP32);

    // for each axis
    for (i=0; i<CNC_MAX_COORDS; i++)
    {
        // check if endpoint is not reached
        if ( core.crt_coord.coord[i] != core.status.crt_seq.params.go_to.coord.coord[i] )   // end point not reached by this axis
        {
            // if we can step on this axis
            if ( Tctr_fp32 >= core.status.op.go_to.StepCkCntr[i] )          // proposed step time reached
            {
                // mark the step clock for this sysclock tick and step the soft coordinate also
                stepmask |= (1<<i);
                core.crt_coord.coord[i] += (core.status.op.go_to.dirmask & (1 << i)) ? 1 : -1;
                // set up the next stepping point
                core.status.op.go_to.StepCkCntr[i]  += core.status.op.go_to.StepCkInc[i];
            }
        }
        else
        {
            endmask |= ( 1 << i );
        }
    }
    if ( endmask != CNC_MAX_COORDMASK )  // if there still are working coordinates then proceed with step clock generation
    {
        // insert step clock
        stepper_insert_clock( stepmask, core.status.op.go_to.dirmask );
    }
    else
    {
        // finish current sequence
        // TODO - start up the next one if available
        core.status.is_running = false;


    }
}





static void internal_sequencer_poll( struct SEventStruct *evt )
{

    if ( core.status.is_running == false )
    {
        struct SMotionSequence seq;
        if ( internal_seq_fifo_get( &seq ) == 0 )
        {
            internal_seq_run( &seq, false );
        }
    }

    if ( core.status.is_running )
    {
        switch ( core.status.crt_seq.seqType )
        {
            case SEQ_TYPE_GOTO:
                internal_seqpoll_goto();
                break;
            case SEQ_TYPE_HOLD:
                break;
            case SEQ_TYPE_SPINDLE:
                break;
        }
    }


}


/* *************************************************
 *
 *  Core routines
 *
 * *************************************************/

void motion_init( void )
{
    memset( &core, 0, sizeof(core) );

}


void motion_poll( struct SEventStruct *evt )
{
    if ( core.status.is_started )
        internal_sequencer_poll( evt );


}



/* *************************************************
 *
 *  Interface routines
 *
 * *************************************************/


void motion_pwr_ctrl( uint32 axis, uint32 power )
{


}

bool motion_pwr_is_set( uint32 axis )
{

}



void motion_set_crt_coord( struct SStepCoordinates *coord )
{
    core.crt_coord = *coord;
    stepper_set_coord( coord );
}


void motion_get_crt_coord( struct SStepCoordinates *coord )
{
    *coord = core.crt_coord;
}


void motion_set_max_travel( struct SStepCoordinates *coord )
{
    core.max_travel = *coord;
}


int motion_step( uint32 axis, uint32 dir )
{

    if ( core.status.is_running )
        return -1;

    while ( stepper_getQ() );

    if ( dir )
    {
        if ( (core.crt_coord.coord[axis] < core.max_travel.coord[axis]) || (core.max_travel.coord[axis] == 0) )
        {
            core.crt_coord.coord[axis]++;
            stepper_insert_clock( (1<<axis), (1<<axis) );
        }
    }
    else
    {
        if ( (core.crt_coord.coord[axis] > 0) || (core.max_travel.coord[axis] == 0) )
        {
            if (core.crt_coord.coord[axis] > 0)           // if non-zero coordinate - just decrement it with one step
                core.crt_coord.coord[axis]--;
            else
            {
                struct SStepCoordinates coord;      // for zero coordinate value - increase the hw. axis to avoid uint wraparround
                stepper_get_coord( &coord );
                coord.coord[axis] = 1;
                stepper_set_coord( &coord );
            }

            stepper_insert_clock( (1<<axis), 0 );
        }
    }

    return 0;
}


int motion_sequence_insert( struct SMotionSequence *seq )
{
    return internal_seq_fifo_insert( seq );
}


void motion_sequence_start( void )
{
    struct SEventStruct evt = { 0, };
    if ( core.status.is_started )
        return;

    core.status.is_started = true;
    internal_sequencer_poll( &evt );
}


void motion_sequence_stop( void )
{

}

void motion_sequence_flush( void )
{

}


uint32 motion_sequence_crt_cmdID()
{
    if ( core.status.is_running )
        return core.status.crt_seq.cmdID;
    else
    {
        struct SMotionSequence *seq;
        if ( internal_seq_fifo_peek( &seq, false ) )
            return CMD_ID_INVALID;
        return seq->cmdID;
    }
}


uint32 motion_sequence_crt_seqID()
{
    if ( core.status.is_running )
        return core.status.crt_seq.seqID;
    else
    {
        struct SMotionSequence *seq;
        if ( internal_seq_fifo_peek( &seq, false ) )
            return CMD_ID_INVALID;
        return seq->seqID;
    }
}






TSpindleSpeed mconv_mmpm_2_sps( uint32 feed_mmpm )
{
    return ((TSpindleSpeed)( (feed_mmpm * STEP_MM) / 60 ));
}






