
// ------------------------------------------------------------
// Step pulse generator:
// How this works:
// If on a coordinate we have to move from point 0 to point X in N steps:
// 
// Time base is system tick defined in STEP_CLOCK. 
// Distance done on coordinate:  D = v * t
//                               v is steps/STEP_CLOCK 
//                               t is incremental and quantized in STEP_CLOCK on each pulse
//                          so we have D = (steps/STEP_CLOCK) * STEP_CLOCK => D = steps * Nr. of clock cycles executed
//      =? we will calculate D incrementally.
//                          D(n) = D(n-1) + Stp     where Stp is step speed in steps/STEP_CLOCK units for the coordinate
//                          D is composed from Dint:Dfract
//                                  Dint - integer part - this is the actual step distance quantized by the motors
//                                  Dfract - fractional part - step speed is added to this, when it overflows it increments Dint
//                          ! As a rule: Stp should be under 0.5 in FP32 !!! ( this is < 1 step pulse per 2 sys clocks )
// 
// 

// ------------------------------------------------------------
// Distance: Calibrating FP precision:
// consider 40x1600 step for rotary table ( 64000 step / 360* ) -> 0*0'20.25" -> 0.005mm on 100mm dia. rotary table
//          500x200x300mm xyz work area
// --> maximum length = 254747 -> 0x3E31B ( we need 18 bit to encode this )
// with everything at maximum:
//          254746.933249451
// c.1 dec A 1 step:
//          254746.682          -> 0.251 diff on 1 step
// c.2 have 1 step dist on A [0->1]:
//          246576.560118759    -> 246576.560120787   => 0.000002028 - to sense it we need 0x7A121 -> 0x7FFFF 
// c.3 have 1 step dist on X [0->1]:
//          157784.663386528    -> 157784.663389697   => -0.000003169 - 0x4E200
// So for worst case scenario - we need L(f) * (1<<19)
// Test the formula with the cases abowe
//      c.1:    133560760139  -> 133560628423   => 131,716
//      c.2:    129277131551  -> 129277131552   => 1            - so it senses the absolute minimum changes also
//      c.3:     82724605597  ->  82724605599   => 2
// L after the 19fp shift becomes a 37bit value

// ------------------------------------------------------------
// stepspeed = steps / stepclock.        mm/min -> stp/ck:
// Nmm/min = N * 400 / 60 step/sec = ( N * 400 )/ ( 60 * STEP_CLOCK ) 
// 
// speed on an axis =  Dist * speed / L  ( ax.speed < total speed )
// 
// minimum allowed speed 20mm/min -> 0.002(6)   steps/stepclock
// Worst case scenario:
//      going XYZdist: 500,200,300mm with 20mm/min -> 200000, 80000, 120000 steps 
//              Adist: 0.005mm (r50mm)             -> 1 step
// Speeds for each axis:
//              sxyz = 0.002163, 0.000865, 0.00129
//              sa   = 1.081*10e-8      
//      sa if 2steps = 2.16*10e-8
// Smallest value is 10e-8 = 1 / 100,000,000 = 1 / 0x05f5e100  - consider with 1 / 0x07ff ffff  -> minimum fp. shift = 27bit.
// So 1<<32 will give sufficient precision

// ------------------------------------------------------------
// Acceleration:
// acceleration is a speed increment in steps/stepclock   
// Ex. accelerate from 150mm/min -> 1500mm/min in 0.5sec:
//     1350mm/min in 0.5sec -> 9000 steps/sec difference in 0.5sec or in stepclock/0.5 
// To obtain acceleration increment at step clock (speed is increased or decreased with this value):
// 
// Acc(stp/sck^2) = StpSpeedDif / TimePeriod(in stepck units)
// Using the example 0.5sec for 1350mm/min -> StpSpeedDif = 9000/stepck = 0.18
//                                            TimePeriod = 0.5s = 25000 stepck
//      Acceleration = 0.0000072 stp/sck^2
// But this is for the toolpath. Acceleration is scaled for each axis like the speeds:
// Worst case scenario:  XYZ:500,200,300mm, and 1 step on A axis
//      0.25 sec iv:    AccFP42 = 0x0000013B
//      0.5  sec iv:    AccFP42 = 0x0000009D   
//      1    sec iv:    AccFP42 = 0x0000004E  
// Worst case scenario:  XYZ:0,0,0mm, and 360* on A axis ( max value on acceleration )
//      0.25 sec iv:    AccFP42 = 0x03c65e1d
//      0.5  sec iv:    AccFP42 = 0x01e32f03   
//      1    sec iv:    AccFP42 = 0x00f19400
// So we can hold acceleration in a 32bit variable for each channel
//
// ------------------------------------------------------------
// Acceleration profile calculation:
// we have the followings:  Vfs, Vfc, Vfe   - Start speed, Constant speed, End speed.
//                          D - distance in steps
// We need to know: D1 - distance when the constant speed applies
//                  D2 - distance when the next acc/dec. begins
//                  a1 - till D1 to accelerate or decelerate
//                  a2 - after D2 to accelerate or decelerate
// 
//    - formula:    V2^2 = V1^2 + 2*a*D
//              so  D = ( V2^2 - V1^2 ) / 2 * ACC
// 
// - !!! First validity condition:
//          D (Vfs-Vfe) should be <= D  
// 
// - Calculate i1 and i2 using Vfs->Vfc and Vfc->Vfe as speeds
// - If (Vfs<Vct) -> a1 positive
//      (Vfs>Vct) -> a1 negative
// - If (Vfe<Vct) -> a2 negative
//      (Vfe>Vct) -> a2 positive
// 
// - If (a1 ^ a2 )  (means a1/a2 alternates)
//      if  i1 <= i2 -> trapezoidal: D1 = i1, D2 = i2
//      if  i1 >  i2 -> triangular:  calculate i3, D1 = D2 = i3
//      i3 = (i1-i2)/2 + i2
// 
// 
// - If (a1 ^ a2) = 0   ( a1/a2 both pos/neg)
//      D1 = i1, D2 = i2
//      NOTE: i1 < i2 all times if the validity condition is fulfilled
// 

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "motion_core.h"
#include "hw_stuff.h"
#include "motion_core_internals.h"
#include "events_ui.h"


#ifdef ON_QT_PLATFORM
static struct STIM1 stim;
static struct STIM1 *TIM1 = &stim;
#endif


#define FP32      32
#define FP16      16
#define FPlen     19

#define DASSERT(x)      do {   if ( !(x) ) \
                               {   sprintf( stderr, "Assert: %s\n", # x );\
                                    while(1);\
                                }\
                        } while(0);


struct SMotionCoreInternals core;
struct SMotionCoreISR       isr;

extern void event_ISR_set10ms( void );

/* *************************************************
 *
 *  ISR part 
 *
 * *************************************************/

static uint32 ISR_counter = 0;


static inline void local_isr_step_channel( int i )
{
    // increment distance counter
    isr.op.D[i] += isr.crt_action.p.Stp_crt[i];

    // step when integer part changes
    if ( isr.op.Dprev[i] != (uint32)(isr.op.D[i] >> 32) )
    {
        switch ( i )
        {
            case 0: HW_StepClk_X(); break;  
            case 1: HW_StepClk_Y(); break;  
            case 2: HW_StepClk_Z(); break;  
            case 3: HW_StepClk_A(); break;  
        }

        // inc/dec absolute position counters
        if ( isr.crt_action.dir_mask & ( 1 << i ) )
            isr.crt_poz.coord[i]++;
        else
            isr.crt_poz.coord[i]--;

        // if reached the coordinate on axis - disable it
        if ( isr.crt_action.p.dest_poz.coord[i] == isr.crt_poz.coord[i] )
            isr.crt_action.channel_active &= ~(1 << i);

        isr.op.Dprev[i] = (uint32)( isr.op.D[i] >> 32 );                // save for recognizing the next step
        isr.ckoff[i] = STEP_CKOFF;
        isr.ckmask |= ( 1 << i );
    }
}


static inline void local_isr_step_turn_channel_off( int i )
{
    isr.ckoff[i]--;
    if ( isr.ckoff[i] == 0 )
    {
        switch ( i )
        {
            case 0: HW_ResetClk_X(); break; 
            case 1: HW_ResetClk_Y(); break; 
            case 2: HW_ResetClk_Z(); break; 
            case 3: HW_ResetClk_A(); break; 
        }
        isr.ckmask &= ~( 1 << i );
    }
}

static inline void local_isr_recalculate_speeds( void )
{
    // recalculate speeds
    bool accsens;

    // acceleration
    if ( isr.state == MCISR_STATUS_RUP )             // if accelerating
    {
        if (isr.op.D[isr.crt_action.ax_max_dist] >= isr.crt_action.p.accdec[0]) // and max. acceleration point is reached
        {
            int i;
            // set state to constant and (for accuracy) set up calculated constant speeds
            isr.state = MCISR_STATUS_CT;
            for (i=0; i<CNC_MAX_COORDS; i++)
            {
                isr.crt_action.p.Stp_crt[i] = isr.crt_action.p.Stp_ct[i];
                isr.op.ctr_speed64[i] = (uint64)isr.crt_action.p.Stp_ct[i] << 32LL;
            }
            return;
        }
        accsens = isr.crt_action.p.accsens & 0x01;      // direction for start phase
    }
    else
    {       
        accsens = isr.crt_action.p.accsens & 0x02;      // direction for end phase
    }

    // constant
    if ( isr.state == MCISR_STATUS_CT )                 // if constant (or just finished accelerating)
    {
        if (isr.op.D[isr.crt_action.ax_max_dist] >= isr.crt_action.p.accdec[1])   // and deceleration point is reached
            isr.state = MCISR_STATUS_RDOWN;
        return;
    }

    // if it is still inside of routine - means that we accelerate or decelerate - calculate speeds
    {
        int i;
        for (i=0; i<CNC_MAX_COORDS; i++)
        {
            if ( accsens )
                isr.op.ctr_speed64[i] += (uint64)isr.crt_action.p.Acc[i] << 22LL;       // Acc is FP42 make it FP64
            else if (isr.op.ctr_speed64[i] > ((uint64)isr.crt_action.p.Acc[i] << 22LL) )
                isr.op.ctr_speed64[i] -= (uint64)isr.crt_action.p.Acc[i] << 22LL;       // Acc is FP42 make it FP64

            isr.crt_action.p.Stp_crt[i] = isr.op.ctr_speed64[i] >> 32LL;
        }
    }
}


static void local_isr_set_directions( uint32 dir )
{
    if ( dir & (1 << COORD_X) )
        HW_SetDirX_Plus();
    else
        HW_SetDirX_Minus();

    if ( dir & (1 << COORD_Y) )
        HW_SetDirY_Plus();
    else
        HW_SetDirY_Minus();

    if ( dir & (1 << COORD_Z) )
        HW_SetDirZ_Plus();
    else
        HW_SetDirZ_Minus();

    if ( dir & (1 << COORD_A) )
        HW_SetDirA_Plus();
    else
        HW_SetDirA_Minus();
}


static void local_isr_peek_next_action( void )
{
    int i;
    isr.crt_action = isr.next_action;       // copy the action struct
    isr.next_action.channel_active = 0;     // mark the user level action struct as free

    StepDBG_LineSegment( &isr.crt_poz, &isr.crt_action.p.dest_poz );

    memset( &isr.op, 0, sizeof(isr.op) );

    isr.state = MCISR_STATUS_RUP;
    for (i=0; i<CNC_MAX_COORDS; i++)
    {
        isr.op.ctr_speed64[i] = ((uint64)isr.crt_action.p.Stp_crt[i]) << 32LL;
    }

    // set up motor directions
    local_isr_set_directions( isr.crt_action.dir_mask );
}


void StepTimerIntrHandler (void)
{
    // Clear update interrupt bit
    TIMER_SYSTEM->SR = (uint16)~TIM_FLAG_Update;

    if ( isr.state )
    {
        int i;

        StepDBG_TickCount();

        for ( i=0; i<CNC_MAX_COORDS; i++ )
        {
            if ( isr.ckmask & (1 << i) )
                local_isr_step_turn_channel_off(i);

            if ( isr.crt_action.channel_active & (1 << i) ) // channel is in move
                local_isr_step_channel(i);
        }

        local_isr_recalculate_speeds();

        // if movement action is finished, set to idle and if available start a new one
        if ( isr.crt_action.channel_active == 0 )
        {
            isr.state = MCISR_STATUS_IDLE;
            StepDBG_SegmentFinished();
            if ( isr.next_action.channel_active )
                local_isr_peek_next_action();
        }
    }
    else
    {   
        // check if there is a new action
        if ( isr.next_action.channel_active )
            local_isr_peek_next_action();
        // check if clock pulse should be deactivated
        if ( isr.ckmask )
        {
            int i;
            for ( i=0; i<CNC_MAX_COORDS; i++ )
            {
                if ( isr.ckmask & (1 << i) )
                    local_isr_step_turn_channel_off(i);
            }
        }
    }
    
    ISR_counter++;
    if ( ISR_counter == SYSTEM_T_10MS_COUNT )
    {
        ISR_counter = 0;
        event_ISR_set10ms();
    }
}




/*--------------------------
 *  User level interaction
 *-------------------------*/

void    stepper_set_coord( struct SStepCoordinates *coord )
{
    __disable_interrupt();
    isr.crt_poz = *coord;
    __enable_interrupt();
}

void    stepper_get_coord( struct SStepCoordinates *coord )
{
    __disable_interrupt();
    *coord  = isr.crt_poz;
    __enable_interrupt();
}

int     stepper_add_action( struct SMotionCoreISRaction *action )
{
    if ( isr.next_action.channel_active )
        return -1;

    __disable_interrupt();
    isr.next_action = *action;
    __enable_interrupt();
}

#define stepper_check_action_busy()     ( isr.crt_action.channel_active )       // do not do double buffering - check always the current action

#define stepper_check_in_progress()     ( isr.next_action.channel_active || isr.crt_action.channel_active )

void    stepper_stop_and_clear()
{
    __disable_interrupt();
    isr.state = MCISR_STATUS_IDLE;
    isr.next_action.channel_active = 0;
    isr.crt_action.channel_active = 0;
    __enable_interrupt();
}

void    stepper_insert_step( uint32 coords, uint32 dirmask )
{
    int i;
    local_isr_set_directions( dirmask ); 

    __disable_interrupt();

    if ( coords & (1 << COORD_X) )
        HW_StepClk_X();
    if ( coords & (1 << COORD_Y) )
        HW_StepClk_Y();
    if ( coords & (1 << COORD_Z) )
        HW_StepClk_Z();
    if ( coords & (1 << COORD_A) )
        HW_StepClk_A();

    for ( i=0; i<CNC_MAX_COORDS; i++ )
    {
        if ( coords & (1 << i) )
        {
            if ( isr.crt_action.dir_mask & ( 1 << i ) )
                isr.crt_poz.coord[i]++;
            else
                isr.crt_poz.coord[i]--;
            isr.ckoff[i] = STEP_CKOFF + 1;
        }
    }

    isr.ckmask |= coords;
    __enable_interrupt();
}




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

static uint32 internal_calculate_stepspeed( uint32 feedspeed )
{
    return ( ((uint64)feedspeed * 400) << FP32 ) / ( 60 * STEP_CLOCK );         // feedspeed 12bit + 9bit + 32bit = 53bit
}                                                                               // returns fp32

static uint32 internal_calculate_acc_dist_fp16( uint32 v1, uint32 v2 )
{
    // D = ( V2^2 - V1^2 ) / 2 * ACC
    // v1, v2 are in fp32 ->  v^2 will produce fp64, divided with ACCfp32 -> fp32, will divide with fp16 to obtain distance in fp16
    return  ((uint64)v2*v2-(uint64)v1*v1) / ( (uint64)ACC_FACTOR_FP32 << 17 );                  // the 17 is from 2*Acc: (fp16)16 + 1 (shift it one more)
}


static inline int internal_calculate_accdec_distances( struct SMotionFifoElement *fseq, 
                                                       uint64 L, 
                                                       uint32 *dists,
                                                       uint32 startspeed,
                                                       uint32 stepspeed,
                                                       uint32 endspeed )
{
    uint32  i1;
    uint32  i2;
    int64   D_i1;   // in FPlen
    int64   D_i2;   // (total distance - i2) in FPlen

    // check validity D (Vfs-Vfe) should be <= D  
    if ( startspeed <= endspeed )
        i1 = internal_calculate_acc_dist_fp16( startspeed, endspeed );
    else
        i1 = internal_calculate_acc_dist_fp16( endspeed, startspeed );

    if ( i1 >= (uint32)( L >> (FPlen-16) ) )
        return -1;      // ERROR - should not happen

    i1 = 0;
    i2 = 0;

    // calculate start acceleration
    fseq->params.go_to.p.accsens = 0;
    if ( startspeed < stepspeed )
    {
        i1 = internal_calculate_acc_dist_fp16( startspeed, stepspeed );
        fseq->params.go_to.p.accsens |= 0x01;   // bit 0 - set to '1' - means it accelerate (positive acceleration)
    }
    else if ( startspeed > stepspeed )
        i1 = internal_calculate_acc_dist_fp16( stepspeed, startspeed );

    // calculate end acceleration
    if ( stepspeed < endspeed )
    {
        i2 = internal_calculate_acc_dist_fp16( stepspeed, endspeed );
        fseq->params.go_to.p.accsens |= 0x02;   // bit 1 - set to '1' - means it accelerate (positive acceleration)
    }
    else if ( stepspeed > endspeed )
        i2 = internal_calculate_acc_dist_fp16( endspeed, stepspeed );

    // Note: L is in FPlen, i2 is in FP16
    D_i2 = (int64)L - (int64)( (uint64)i2 << (FPlen - FP16) );      // it is in FPlen
    D_i1 = (int64)( (uint64)i1 << (FPlen - FP16) );

    // for the same acceleration direction
    if ( (fseq->params.go_to.p.accsens == 0x01) || (fseq->params.go_to.p.accsens == 0x02) ) // sign differs - accelerate/decelerate
    {
        if ( D_i1 > D_i2 )
        {
            // if  i1 >  i2 -> triangular:  calculate i3, D1 = D2 = i3  where i3 = (i1-i2)/2 + i2
            D_i1 = D_i2 + ( D_i1 - D_i2 ) / 2;
            D_i2 = D_i1;
        }
        // else if  i1 <= i2 -> trapezoidal: D1 = i1, D2 = i2
    }
    // else if both negative or positive: D1 = i1, D2 = D_i2 

    // transpose i1, i2 to distance on the longest axis
    // axdist = D_ix * dax / L          - 37bit + 18bit = 55bit - can add 9 bits for precision, 
    // to obtain fp32 on distance we will add 9 bits by downshift L, and 14 bits by shifting the result:  9 + 9 + 14 -> 32 bit
    fseq->params.go_to.p.accdec[0] = ( ( ((uint64)D_i1 * (dists[fseq->params.go_to.ax_max_dist] << 9)) / ( L >> 9 ) ) << 14 );
    if ( D_i1 == D_i2 )
        fseq->params.go_to.p.accdec[1] = fseq->params.go_to.p.accdec[0];
    else
        fseq->params.go_to.p.accdec[1] = ( ( ((uint64)D_i2 * (dists[fseq->params.go_to.ax_max_dist] << 9)) / ( L >> 9 ) ) << 14 );
    return 0;
}


static int internal_sequence_precalculate( struct SMotionSequence *mseq, struct SMotionFifoElement *fseq )
{
    uint32 dists[CNC_MAX_COORDS];
    uint32 dirmask = 0;
    uint32 maxdist = 0;
    uint64 sum = 0LL;
    int i;
    uint64 L = 0;
    uint32 stepspeed;
    uint32 startspeed;
    uint32 endspeed;

    // get the distances and direction on each axis
    dirmask = 0;
    for ( i=0; i<CNC_MAX_COORDS; i++ )
    {
        dirmask |= ( internal_run_helper_get_distance( core.seq_fifo.pcoord.coord[i],
                                                       mseq->params.go_to.coord.coord[i],
                                                       &dists[i] ) << i );
    }
    fseq->params.go_to.dir_mask = (uint8)dirmask;
    fseq->params.go_to.p.dest_poz = mseq->params.go_to.coord;

    // Calculate the total distances
    // L = sqrt( x*x + y*y + z*z + a*a )
    fseq->params.go_to.channel_active = 0;
    for ( i=0; i<CNC_MAX_COORDS; i++ )
    {
        sum += ((uint64)dists[i]) * ((uint64)dists[i]);
        if ( dists[i] )
        {
            fseq->params.go_to.channel_active |= ( 1 << i );
        }
        if ( maxdist < dists[i] )
        {
            maxdist = dists[i];
            fseq->params.go_to.ax_max_dist = (uint8)i;
        }
    }
                                                            
    L = (uint64)( sqrt( sum ) * (1LL<<FPlen) );

    // Calculate stepspeed ( see explanation abowe )
    stepspeed = internal_calculate_stepspeed( mseq->params.go_to.feed_speed );      // stepspeed in fp.32 - it will use 31 bit always
    if ( mseq->params.go_to.feed_speed != mseq->params.go_to.feed_bgn )
        startspeed = internal_calculate_stepspeed( mseq->params.go_to.feed_bgn ); 
    else
        startspeed = stepspeed;
    if ( mseq->params.go_to.feed_speed != mseq->params.go_to.feed_end )
        endspeed = internal_calculate_stepspeed( mseq->params.go_to.feed_end ); 
    else
        endspeed = stepspeed;

    DASSERT( (stepspeed & 0x80000000) == 0 );                                       // should never overrun 31 bits
    DASSERT( (startspeed & 0x80000000) == 0 );                                       // should never overrun 31 bits
    DASSERT( (ACC_FACTOR_FP32 & 0xFFFF0000) == 0 );                                 // acceleration factor should fit in 16 bit

    for ( i=0; i<CNC_MAX_COORDS; i++ )
    {
        // Calculation: axspeed = (( stpspeedFP32 * dist ) * ( 1<<FPlen)) / L   -> will maintain FP32 for axspeed
        // Problem is: stpspeedFP32 * dist * (1<<FPlen)  --->   31bit + 18bit + 19bit = 68bit - need to get rid of 4 bits
        // since stepspeed encoding can be done on 27 bit - change the formula:
        // axspeed = (( (stpspeedFP32) * dist ) * ( 1<<(FPlen-4))) / ( L >> 4 )
        fseq->params.go_to.p.Stp_ct[i] = ( (((uint64)stepspeed) << (FPlen-4)) * dists[i] ) / ( L >> 4 ); 

        if ( startspeed == stepspeed )
            fseq->params.go_to.p.Stp_crt[i] = fseq->params.go_to.p.Stp_ct[i];
        else
            fseq->params.go_to.p.Stp_crt[i] = ( (((uint64)startspeed) << (FPlen-4)) * dists[i] ) / ( L >> 4 ); 

        // Acc[i] = Dist[i] * ACCFACTOR / totalDist  + 10bit shift to have FP42
        // To get acceleration in FP42 must shift with 10 bit additional.   nominator bit count: 16 + 10 + 19 + 18 = 63;    the (16+10) + 16 gives the fp42
        // max error: 5steps / 500mm
        fseq->params.go_to.p.Acc[i] = ( (((uint64)ACC_FACTOR_FP32) << (FPlen+10)) * dists[i] ) / L;
    }

    // Calculate acceleration/deceleration distances
    if ( internal_calculate_accdec_distances( fseq, L, dists, startspeed, stepspeed, endspeed ) )
        return -1;

    return 0;
}


static inline void internal_seqpoll_run_goto( struct SMotionFifoElement *seq )
{
    struct SMotionCoreISRaction action;
    // do not introduce new action till isr didn't finished completely the current one
    if ( stepper_check_action_busy() )
        return;

    action.p = seq->params.go_to.p;
    action.channel_active = seq->params.go_to.channel_active;
    action.ax_max_dist = seq->params.go_to.ax_max_dist;
    action.dir_mask = seq->params.go_to.dir_mask;
    action.seq_id = seq->seqID;

    stepper_add_action( &action );

    core.status.is_running = true;
    core.status.crt_seq = *seq;
    return;
}

static inline void internal_seqpoll_run_hold_time( struct SMotionFifoElement *seq )
{

}

static inline void internal_seqpoll_run_spindle_speed( struct SMotionFifoElement *seq )
{

}



static void internal_sequencer_poll( struct SEventStruct *evt )
{
    // if sequence is in run - keep it working
    if ( core.status.is_running )
    {
        switch ( core.status.crt_seq.seqType )
        {
            case SEQ_TYPE_GOTO:
                if ( stepper_check_in_progress() == false )  // stepping action is finished (and no new one initiated)
                {
                    core.status.is_running = false;
                }
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
        // start the next sequence 
        struct SMotionFifoElement seq;
        if ( internal_seq_fifo_get( &seq ) == 0 )
        {
            switch ( seq.seqType )
            {
                case SEQ_TYPE_GOTO:
                    internal_seqpoll_run_goto( &seq );
                    break;
                case SEQ_TYPE_HOLD:
                    internal_seqpoll_run_hold_time( &seq );
                    break;
                case SEQ_TYPE_SPINDLE:
                    internal_seqpoll_run_spindle_speed( &seq );
                    break;
            }
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
    memset( &isr, 0, sizeof(isr) );
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
    stepper_set_coord( coord );
}


void motion_get_crt_coord( struct SStepCoordinates *coord )
{
    stepper_get_coord( coord );
}


void motion_set_max_travel( struct SStepCoordinates *coord )
{
    core.max_travel = *coord;
}


int motion_step( uint32 axis, uint32 dir )
{
    if ( core.status.is_running )
        return -1;

    while ( isr.ckmask );

    // we can use directly the the isr coordinate values because there is no action in progress

    if ( dir )
    {
        if ( (isr.crt_poz.coord[axis] < core.max_travel.coord[axis]) || (core.max_travel.coord[axis] == 0) )
            stepper_insert_step( (1<<axis), (1<<axis) );
    }
    else
    {
        if ( (isr.crt_poz.coord[axis] > 0) || (core.max_travel.coord[axis] == 0) )
        {
            if (isr.crt_poz.coord[axis] == 0 )      // if zero coordinate - do an increment to prevent negative wrap arround
                isr.crt_poz.coord[axis]++;              
            stepper_insert_step( (1<<axis), 0 );
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
        {                                                   // if first run after a stop/flush
            stepper_get_coord( &core.seq_fifo.pcoord );     // update the end coordinates with the current coordinate
            core.seq_fifo.pcoord_updated = 1;
        }
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






