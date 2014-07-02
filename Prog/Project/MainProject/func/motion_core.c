#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "motion_core.h"
#include "motion_core_internals.h"
#include "events_ui.h"



#define FP32      32
#define FP16      16

#define DASSERT(x)      do {   if ( !(x) ) \
                               {   sprintf( stderr, "Assert: %s\n", # x );\
                                    while(1);\
                                }\
                        } while(0);


struct SMotionCoreInternals core;



/* *************************************************
 *
 *  Internal routines
 *
 * *************************************************/


/*--------------------------
 *  Fifo routines
 *-------------------------*/

static struct SMotionFifoElement *internal_seq_fifo_get_fill_pointer( void )
{
    if ( core.seq_fifo.c >= MAX_SEQ_FIFO )
        return NULL;
    return &core.seq_fifo.seq[ core.seq_fifo.w ];
}


static int internal_seq_fifo_insert( struct SMotionFifoElement *seq )
{
    if ( core.seq_fifo.c >= MAX_SEQ_FIFO )
        return -1;

    if ( seq != NULL )
        core.seq_fifo.seq[ core.seq_fifo.w ] = *seq;

    core.seq_fifo.w++;

    if ( core.seq_fifo.w == MAX_SEQ_FIFO )
        core.seq_fifo.w = 0;
    core.seq_fifo.c++;
    return 0;
}


static int internal_seq_fifo_get( struct SMotionFifoElement *seq )
{
    if ( core.seq_fifo.c == 0 )
        return -1;

    *seq = core.seq_fifo.seq[ core.seq_fifo.r++ ];

    if ( core.seq_fifo.r == MAX_SEQ_FIFO )
        core.seq_fifo.r = 0;
    core.seq_fifo.c--;
    return 0;
}


static int internal_seq_fifo_peek( struct SMotionFifoElement **seq, bool next )
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


static void internal_sequence_precalculate( struct SMotionSequence *mseq, struct SMotionFifoElement *fseq )
{
    // fills    .distances
    //          .Ttot
    //          .dirmask

    int i;
    uint32 dirmask = 0;
    uint32 *dists = fseq->params.go_to.distances;
    uint64 sum = 0LL;
    double L = 0;
    double T = 0;

    // get the distances and direction on each axis
    dirmask = 0;
    for ( i=0; i<CNC_MAX_COORDS; i++ )
    {
        dirmask |= ( internal_run_helper_get_distance( core.seq_fifo.pcoord.coord[i],
                                                       mseq->params.go_to.coord.coord[i],
                                                       &dists[i] ) << i );
    }
    fseq->params.go_to.dirmask = dirmask;

    // Calculate the total distance and total time
    // L = sqrt( x*x + y*y + z*z + a*a )
    for ( i=0; i<CNC_MAX_COORDS; i++ )
    {
        sum += ((uint64)dists[i]) * ((uint64)dists[i]);
    }

    // T = L / sps_speed -> how many seconds we should step
    // T = ( L * STEP_SEC ) / sps_speed -> how many step units we should step. This should be < 1
    // t[i] = T / dist

    L = sqrt( sum );                     // total lenght in steps (in fp32)
    T = L / mseq->params.go_to.feed;     // Total time in seconds

    // to fit in 32 bit - T(sec) < 214,748sec -> 59h - we should newer reach this value
    DASSERT ( (uint32)(T) < 214748 );

    // convert double time in fp32
    fseq->params.go_to.Ttot = (uint64)( T * ( (1LL<<FP32) * STEP_SEC ) );         // Total time in sysclocks in fp32
}


static void internal_run_goto( struct SMotionFifoElement *seq )
{
    uint32 *dists = seq->params.go_to.distances;
    int i;

    struct SMotionFifoElement *next_seq = NULL;

    // peek the next sequence for checking the speeds for speed difference
    if ( internal_seq_fifo_peek( &next_seq, false) == 0 )
    {
        if ( next_seq->seqType != SEQ_TYPE_GOTO )
            next_seq = NULL;
    }


    // calculate step speed increment on each axis
    for ( i=0; i<CNC_MAX_COORDS; i++ )
    {
        if ( dists[i] )
            core.status.op.go_to.StepCkInc[i] = seq->params.go_to.Ttot / (dists[i]+1);        // nr. of sysclock / step in fp.32
        else
            core.status.op.go_to.StepCkInc[i] = seq->params.go_to.Ttot;

        DASSERT( core.status.op.go_to.StepCkInc[i] >= (1LL<<FP32) );
        core.status.op.go_to.StepCkCntr[i] = core.status.op.go_to.StepCkInc[i];
    }

    core.status.is_running = true;
    core.status.crt_seq = *seq;
    core.status.op.go_to.Tctr = 0;

    return;
}




static void internal_seq_run( struct SMotionFifoElement *seq )
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
                core.crt_coord.coord[i] += (core.status.crt_seq.params.go_to.dirmask & (1 << i)) ? 1 : -1;
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
        stepper_insert_clock( stepmask, core.status.crt_seq.params.go_to.dirmask );
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
    // if terminated or it isn't running
    if ( core.status.is_running == false )
    {
        struct SMotionFifoElement seq;
        if ( internal_seq_fifo_get( &seq ) == 0 )
        {
            internal_seq_run( &seq );
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

    return true;
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
    struct SMotionFifoElement *fseq;

    fseq = internal_seq_fifo_get_fill_pointer();
    if ( fseq == NULL )
        return -1;

    fseq->cmdID = seq->cmdID;
    fseq->seqID = seq->seqID;
    fseq->seqType = seq->seqType;

    if ( seq->seqType == SEQ_TYPE_GOTO )
    {
        // for motion sequence we need to precalculate the parameters
        if ( core.seq_fifo.pcoord_updated == 0 )
        {
            core.seq_fifo.pcoord = core.crt_coord;          // update the end coordinates with the current soft coordinate
            core.seq_fifo.pcoord_updated = 1;
        }
        fseq->params.go_to.coord = seq->params.go_to.coord;
        internal_sequence_precalculate( seq, fseq );
        core.seq_fifo.pcoord = seq->params.go_to.coord;     // save the end coordinates of the introduced sequence
    }
    else
    {
        // for the other sequences
        fseq->params.spindle = seq->params.spindle;
        //location for note#0001 from motion_core_internals.h
    }

    return internal_seq_fifo_insert( NULL );
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
        struct SMotionFifoElement *seq;
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
        struct SMotionFifoElement *seq;
        if ( internal_seq_fifo_peek( &seq, false ) )
            return CMD_ID_INVALID;
        return seq->seqID;
    }
}






TSpindleSpeed mconv_mmpm_2_sps( uint32 feed_mmpm )
{
    return ((TSpindleSpeed)( (feed_mmpm * STEP_MM) / 60 ));
}






