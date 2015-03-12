
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

extern void event_ISR_set10ms(void);
extern void event_ISR_set100us(void);


/* *************************************************
 *
 *  ISR part 
 *
 * *************************************************/

static uint32 ISR_counter_1 = 0;
static uint32 ISR_counter_2 = 0;

static inline int local_isr_speed_scale(void)
{
    if ( isr.scale_factor == 0 )        // no speed scaling - consider each tick
        return 1;
    else
    {
        if ( isr.scale_factor < 0 )     // slow down clock by dropping ticks
        {
            isr.sf_ctr += isr.sf_val;
            if ( (isr.sf_ctr & 0xFFFF0000) == 0)        // check if counter surpassed 16bit -> means that tick can be considered
                return 1;
            isr.sf_ctr &= ~0xFFFF0000;                  // clean the upper 16bit
            return 0;
        }
        else                            // speed up clock by duplicating addition
        {
            isr.sf_ctr += isr.sf_val;
            if ( isr.sf_ctr & 0xFFFF0000)               // check if counter surpassed 16bit -> means that tick needs to be duplicated
            {
                isr.sf_ctr &= ~0xFFFF0000;              // clean the upper 16bit
                return 2;
            }
            return 1;
        }
    }
    return 1;
}


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
            isr.request = isrur_set_ct_pwr;
            return;
        }
        accsens = isr.crt_action.p.accsens & 0x01;      // direction for start phase
        StepDBG_Accelerations( 0, accsens );
    }
    else
    {       
        accsens = isr.crt_action.p.accsens & 0x02;      // direction for end phase
        StepDBG_Accelerations( 2, accsens );
    }

    // constant
    if ( isr.state == MCISR_STATUS_CT )                 // if constant (or just finished accelerating)
    {
        StepDBG_Accelerations( 1, 0 );
        if (isr.op.D[isr.crt_action.ax_max_dist] >= isr.crt_action.p.accdec[1])   // and deceleration point is reached
        {
            isr.state = MCISR_STATUS_RDOWN;
            isr.request = isrur_set_end_pwr;
        }
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


static void local_isr_fetch_new_action( void )
{
    if ( isr.request_on_hold )      // do not execute anything till user level didn't fulfilled the previous request
        return;

    if ( isr.stepper_fifo.stp[ isr.stepper_fifo.r ].seqType != SEQ_TYPE_GOTO )
    {
        // notify user level about external inband operation
        isr.request = isrur_execute_inband;
        isr.request_on_hold = true;                 // prevent re-dispatching the request
        return;
    }
    else
    {
        int i;
        struct SMFEL_Goto *elem;

        // set it up as a current action
        elem = &isr.stepper_fifo.stp[ isr.stepper_fifo.r ].params.go_to;
        isr.crt_action = elem->act;             // copy the action struct (less overhead than using pointers)
        elem->act.channel_active = 0;       // mark that this element is running

        // set up power level
        for (i=0; i<CNC_MAX_COORDS; i++)
        {
            if ( core.status.pwr[i].set_pwr == mpwr_auto )
            {
                HW_SetPower( i, elem->pwr[i].pwr_start );
            }
        }
        isr.request = isrur_set_start_pwr;

        StepDBG_LineSegment( &isr.crt_poz, &isr.crt_action.p.dest_poz, isr.stepper_fifo.stp[ isr.stepper_fifo.r ].seqID );
        memset( &isr.op, 0, sizeof(isr.op) );

        isr.state = MCISR_STATUS_RUP;
        for (i=0; i<CNC_MAX_COORDS; i++)
        {
            isr.op.ctr_speed64[i] = ((uint64)isr.crt_action.p.Stp_crt[i]) << 32LL;
        }

        // set up motor directions
        local_isr_set_directions( isr.crt_action.dir_mask );
    }
}


void StepTimerIntrHandler (void)
{
    // Clear update interrupt bit
    TIMER_SYSTEM->SR = (uint16)~TIM_FLAG_Update;

    if ( isr.state )
    {
        int i;
        int n;

        StepDBG_TickCount();

        n = local_isr_speed_scale();
        while ( n )
        {
            for ( i=0; i<CNC_MAX_COORDS; i++ )
            {
                if ( isr.ckmask & (1 << i) )
                    local_isr_step_turn_channel_off(i);

                if ( isr.crt_action.channel_active & (1 << i) ) // channel is in move
                {
                    local_isr_step_channel(i);
                }
            }

            local_isr_recalculate_speeds();

            // if movement action is finished, set to idle and if available start a new one
            if ( isr.crt_action.channel_active == 0 )
            {
                isr.state = MCISR_STATUS_IDLE;
                StepDBG_SegmentFinished();

                // release element
                isr.stepper_fifo.r++;
                if ( isr.stepper_fifo.r == MAX_STEP_FIFO )
                    isr.stepper_fifo.r = 0;
                isr.stepper_fifo.c--;

                // check for the next
                if ( isr.stepper_fifo.c )
                    local_isr_fetch_new_action();
            }
            n--;
        }
    }
    else
    {   
        // check if there is a new action
        if ( isr.stepper_fifo.c )
            local_isr_fetch_new_action();
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

    // timing signals
    ISR_counter_1++;
    if ( ISR_counter_1 == SYSTEM_T_100US_COUNT )
    {
        ISR_counter_1 = 0;
        event_ISR_set100us();
        ISR_counter_2++;
        if ( ISR_counter_2 == SYSTEM_T_10MS_COUNT )
        {
            ISR_counter_2 = 0;
            event_ISR_set10ms();
        }
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


#define stepper_check_in_progress()     ( isr.stepper_fifo.c )                  // stepper ISR is running a sequence

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

void stepper_stop()
{
    __disable_interrupt();
    isr.crt_action.channel_active = 0;
    isr.stepper_fifo.c = 0;
    isr.stepper_fifo.w = isr.stepper_fifo.r;        // preserve read pointer to be able to retreive the last executed sequence
    isr.request = 0;
    isr.request_on_hold = false;
    __enable_interrupt();
}


void stepper_scale_factor( int32 factor )
{
    uint32 inc_val = 0;

    if ( factor )
    {
        if ( factor < 0 )
            inc_val = 0x8000 + 0x8000 * ( 200 - factor) / -200;
        else
            inc_val = 0x10000 * factor / 200;
    }

    __disable_interrupt();
    isr.scale_factor = factor;
    isr.sf_ctr = 0;
    isr.sf_val = inc_val;
    __enable_interrupt();
}


int stepper_confirm_inband(void)
{
    if ( isr.request_on_hold == false )
        return -1;
    __disable_interrupt();
    // advance to the next element
    isr.stepper_fifo.r++;
    if ( isr.stepper_fifo.r == MAX_STEP_FIFO )
        isr.stepper_fifo.r = 0;
    isr.stepper_fifo.c--;
    // unblock the sequence fetcher
    isr.request_on_hold = false;
    __enable_interrupt();
    return 0;
}

struct SStepFifoElement *stepper_get_request(uint32 *val)
{
    // !!! Precaution - this returs the pointer with the element currently in run - it may change meantime - so 
    // use it only for the current isr.request in a single loop. (postponing can overwrite it's content)
    struct SStepFifoElement *elem;
    __disable_interrupt();
    *val = (uint32)isr.request;
    isr.request = 0;
    elem = &isr.stepper_fifo.stp[isr.stepper_fifo.r];
    __enable_interrupt();
    return elem;
}


struct SStepFifoElement *stepper_get_current_in_run(void)
{
    // !!! Precaution - this returs the pointer with the element currently in run - it may change meantime - so 
    // use it only for the current isr.request in a single loop. (postponing can overwrite it's content)
    struct SStepFifoElement *elem;
    __disable_interrupt();
    elem = &isr.stepper_fifo.stp[isr.stepper_fifo.r];
    __enable_interrupt();
    return elem;
}


/* *************************************************
 *
 *  Internal routines
 *
 * *************************************************/

/*------------------------------
 *  Input sequence fifo routines
 *----------------------------*/

static struct SMotionSequence *internal_sequence_fifo_get_fill_pointer( void )
{
    if ( core.sequence_fifo.c >= MAX_SEQ_FIFO )
        return NULL;
    return &core.sequence_fifo.seq[ core.sequence_fifo.w ];
}


static int internal_sequence_fifo_insert( struct SMotionSequence *seq )
{
    if ( core.sequence_fifo.c >= MAX_SEQ_FIFO )
        return -1;

    if ( seq != NULL )
        core.sequence_fifo.seq[ core.sequence_fifo.w ] = *seq;

    core.sequence_fifo.w++;

    if ( core.sequence_fifo.w == MAX_SEQ_FIFO )
        core.sequence_fifo.w = 0;
    core.sequence_fifo.c++;
    return 0;
}


static int internal_sequence_fifo_get( struct SMotionSequence *seq )
{
    if ( core.sequence_fifo.c == 0 )
        return -1;

    if ( seq )
        *seq = core.sequence_fifo.seq[ core.sequence_fifo.r ];
    core.sequence_fifo.r++;

    if ( core.sequence_fifo.r == MAX_SEQ_FIFO )
        core.sequence_fifo.r = 0;
    core.sequence_fifo.c--;
    return 0;
}


static int internal_sequence_fifo_peek( struct SMotionSequence **seq, bool next )
{
    int idx;

    *seq = NULL;
    if ( (core.sequence_fifo.c == 0) || ( next && (core.sequence_fifo.c == 1)) )
        return -1;

    idx = core.sequence_fifo.r;
    if ( next )
    {
        idx++;
        if ( idx == MAX_SEQ_FIFO )
            idx = 0;
    }

    *seq = &core.sequence_fifo.seq[ idx ];
    return 0;
}


static void internal_sequence_fifo_flush( void )
{
    core.sequence_fifo.c = 0;
    core.sequence_fifo.w = 0;
    core.sequence_fifo.r = 0;
}


static inline int internal_sequence_fifo_fullness( void )
{
    return core.sequence_fifo.c;
}


/*--------------------------
 *  Output step fifo routines
 *-------------------------*/

static struct SStepFifoElement *internal_step_fifo_get_fill_pointer( void )
{
    if ( isr.stepper_fifo.c >= MAX_STEP_FIFO )              // atomic op., can be used w/o mutex
        return NULL;
    return &isr.stepper_fifo.stp[ isr.stepper_fifo.w ]; // write pointer is operated by user level - no need for mutex
}


static int internal_step_fifo_insert( struct SStepFifoElement *stp )
{
    if ( isr.stepper_fifo.c >= MAX_STEP_FIFO )
        return -1;

    if ( stp != NULL )
        isr.stepper_fifo.stp[ isr.stepper_fifo.w ] = *stp;

    isr.stepper_fifo.w++;                                   // write pointer is operated by user level - no need for mutex

    if ( isr.stepper_fifo.w == MAX_STEP_FIFO )
        isr.stepper_fifo.w = 0;

    __disable_interrupt();
    isr.stepper_fifo.c++;
    __enable_interrupt();

    return 0;
}

static inline int internal_step_fifo_fullness( void )
{
    return isr.stepper_fifo.c;
}


/*--------------------------
 *  Power control routines
 *-------------------------*/

static void internal_pwr_set_axis_power( uint32 axis, uint32 pwr_level, uint32 delay )
{
    if ( pwr_level == core.status.pwr[axis].crt_power )
        return;

    core.status.pwr_check = true;
    core.status.pwr[axis].delay = delay;
    core.status.pwr[axis].is_dirty = true;
    core.status.pwr[axis].crt_power = pwr_level;
    if ( delay == 0 )
        HW_SetPower( axis, pwr_level );
}


static inline void internal_pwr_check_and_update_status( bool tick )
{
    if ( core.status.pwr_check )                    // check power setup status
    {
        int i;
        core.status.pwr_check = false;
        for ( i=0; i<CNC_MAX_COORDS; i++ )
        {
            if ( core.status.pwr[i].is_dirty )              // if setup in pending - check for completion
            {
                if ( core.status.pwr[i].delay )                     // if delayed
                {
                    core.status.pwr_check = true;
                    if ( tick )                                                 // count down on 10ms interval
                    {
                        core.status.pwr[i].delay--;
                        if ( core.status.pwr[i].delay == 0 )
                        {
                            HW_SetPower(i, core.status.pwr[i].crt_power);       // if reached 0 -> set the power
                        }
                    }
                }
                else                                            // if no delay - check the completion
                {
                    if ( HW_IsPowerSet(i) )
                        core.status.pwr[i].is_dirty = false;    // if is set - clear the dirty flag
                    else
                        core.status.pwr_check = true;           // if still not set - mark for chekup for the next time
                }
            }
        }
    }
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
    uint64 acc;
    acc = (uint64)ACC_FACTOR_FP32 << 17;
    return  ((uint64)v2*v2-(uint64)v1*v1) / ( acc );                  // the 17 is from 2*Acc: (fp16)16 + 1 (shift it one more)
}


static inline int internal_calculate_accdec_distances( struct SStepFifoElement *fstep, 
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
    fstep->params.go_to.act.p.accsens = 0;
    if ( startspeed < stepspeed )
    {
        i1 = internal_calculate_acc_dist_fp16( startspeed, stepspeed );
        fstep->params.go_to.act.p.accsens |= 0x01;   // bit 0 - set to '1' - means it accelerate (positive acceleration)
    }
    else if ( startspeed > stepspeed )
        i1 = internal_calculate_acc_dist_fp16( stepspeed, startspeed );

    // calculate end acceleration
    if ( stepspeed < endspeed )
    {
        i2 = internal_calculate_acc_dist_fp16( stepspeed, endspeed );
        fstep->params.go_to.act.p.accsens |= 0x02;   // bit 1 - set to '1' - means it accelerate (positive acceleration)
    }
    else if ( stepspeed > endspeed )
        i2 = internal_calculate_acc_dist_fp16( endspeed, stepspeed );

    // Note: L is in FPlen, i2 is in FP16
    D_i2 = (int64)L - (int64)( (uint64)i2 << (FPlen - FP16) );      // it is in FPlen
    D_i1 = (int64)( (uint64)i1 << (FPlen - FP16) );

    // for the same acceleration direction
    if ( (fstep->params.go_to.act.p.accsens == 0x01) || (fstep->params.go_to.act.p.accsens == 0x02) ) // sign differs - accelerate/decelerate
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
    fstep->params.go_to.act.p.accdec[0] = ( ( ((uint64)D_i1 * (dists[fstep->params.go_to.act.ax_max_dist] << 9)) / ( L >> 9 ) ) << 14 );
    if ( D_i1 == D_i2 )
        fstep->params.go_to.act.p.accdec[1] = fstep->params.go_to.act.p.accdec[0];
    else
        fstep->params.go_to.act.p.accdec[1] = ( ( ((uint64)D_i2 * (dists[fstep->params.go_to.act.ax_max_dist] << 9)) / ( L >> 9 ) ) << 14 );
    return 0;
}


static inline uint32 internal_calculate_cosTheta( uint64 Len1, struct SMotionSequence *crt_seq, struct SMotionSequence *next_seq )
{
    uint32 L1;
    uint32 L2;
    int i;
    int64 dot_prod = 0;

    // if there is no next sequence - consider CosTheta=0 which means that must stop
    if ( !next_seq || (next_seq->seqType != SEQ_TYPE_GOTO) )
        return 0;

    // calculate the dot product for the two vectors:
    // dx1 * dx2 + dy1 * dy2 + dz1 * dz2 + da1 * da2
    for ( i=0; i<CNC_MAX_COORDS; i++ )
    {
        int64 mul;
        int d1 = (crt_seq->params.go_to.coord.coord[i] - core.status.motion.pcoord.coord[i]);
        int d2 = (next_seq->params.go_to.coord.coord[i] - crt_seq->params.go_to.coord.coord[i]);
        mul = d1 * d2;
        dot_prod += mul;
    }

    // magnitude of vectors (lengths)
    L1 = (Len1 >> FPlen);
    L2 = (core.status.motion.next_L >> FPlen);

    // calculate cosT:
    // L1, L2 are in integer steps
    // dot prod is max 36 bit, we will add 24bits for precision - 60bits + sign
    if ( dot_prod > 0 )
        return (uint32)(( (uint64)dot_prod << 24 ) / ( (uint64)L1 * (uint64)L2 ));
    else
        return 0;   // for sharp angles ( < 90* ) we pass 0 because it needs a full stop
}


static inline uint64 internal_calculate_speeds_and_distance( struct SMotionSequence *crt_seq, struct SMotionSequence *next_seq, uint32 *feed_start, uint32 *feed_end )
{
    uint64 ret_val;
    uint64 sum;
    uint32 CosTheta;
    int i;

    uint32 fstart;
    uint32 fend;

    // check and calc for the current sequence
    if ( core.status.motion.next_precalc == false ) // no precalculation from the previous step (peeked next sequence - which is the current one now)
    {
        sum = 0LL;
        for ( i=0; i<CNC_MAX_COORDS; i++ )
        {
            uint32 dist;
            internal_run_helper_get_distance( core.status.motion.pcoord.coord[i], crt_seq->params.go_to.coord.coord[i], &dist );
            sum += ((uint64)dist) * ((uint64)dist);
        }
        ret_val = (uint64)( sqrt( sum ) * (1LL<<FPlen) );
    }
    else
    {
        ret_val = core.status.motion.next_L;
        core.status.motion.next_precalc = false;
    }

    // check and calc for the next sequence
    if ( next_seq && (next_seq->seqType == SEQ_TYPE_GOTO) )
    {
        sum = 0LL;
        for ( i=0; i<CNC_MAX_COORDS; i++ )
        {
            uint32 dist;
            internal_run_helper_get_distance( crt_seq->params.go_to.coord.coord[i], next_seq->params.go_to.coord.coord[i], &dist );
            sum += ((uint64)dist) * ((uint64)dist);
        }
        core.status.motion.next_L = (uint64)( sqrt( sum ) * (1LL<<FPlen) );
        core.status.motion.next_precalc = true;
    }
    else
    {
        core.status.motion.next_precalc = false;
    }

    // speed difference bw. sequences:  
    //      - plain feed speed difference 
    //      - angle bw. sequences

    // figure out the start speed
    if ( core.status.motion.prev_speed == 0 )       // starting from speed 0
    {   
        if ( crt_seq->params.go_to.feed > CNC_START_FEED )
            fstart = CNC_START_FEED;
        else
            fstart = crt_seq->params.go_to.feed;
    }
    else
    {                                               // else use the saved last speed (relative to this vector - it has cosTheta also)
        fstart = core.status.motion.prev_speed;
    }

    // figure out the end speed. CosTheta is given in FP24 - accurate till 4th decimal after dot
    CosTheta = internal_calculate_cosTheta( ret_val, crt_seq, next_seq );
    if ( CosTheta == 0 )
    {
        // full stop should be carried out no matter what speed has the next sequence
        core.status.motion.prev_speed = 0;      // indicating that this sequence stopped, start speed for the next sequence is from 0
        fend = CNC_MIN_FEED;                    //
    }
    else
    {
        if ( CosTheta > ( (uint32)(0.96 * (1<<24)) ) )     // if angle between vectors < 15* - do not consider it - regard it as straight line
            CosTheta = (1 << 24);
        fend = (uint32)( ((uint64)next_seq->params.go_to.feed * CosTheta) >> 24 );        // relative speed for the next sequence
        if ( fend < CNC_MIN_FEED )
            fend = CNC_MIN_FEED;
        if ( fend > crt_seq->params.go_to.feed )        // never accelerate the current sequence end because of the next sequence
        {
            fend = crt_seq->params.go_to.feed;
        }

    }
    
    // check if end speed is doable with the current start speed and acceleration
    {
        uint32 dist;
        uint32 fe;
        uint32 fs;

        fe = internal_calculate_stepspeed( fend );
        fs = internal_calculate_stepspeed( fstart );

        // formula: 
        // V^2 = V0^2 + 2*a*L
        // V = Sqrt( V0^2 + 2*a*L )
        if ( fend > fstart )
        {
            // distance = ( V^2 - V0^2 ) / 2*a      
            // dist in steps = distance * 400
            dist = internal_calculate_acc_dist_fp16( fs, fe ) + 1; 
            dist = dist >> 16;                      // convert back from fp16
            if ( dist >= (ret_val >> FPlen) )       // this cut of precision adds to the safety margin
            {
                // new end speed = Sqrt( V0^2 + 2 * acc * L(mm) ) = Sqrt( V0*V0 + 2*acc*L(step)/400 )
                double sq;
                sq = sqrt( fstart*fstart + (uint64)ACC_FACTOR_MMPM*(ret_val>>FPlen)/200 );
                fend = (uint32)sq - 1;
            }
        }
        else if ( fend < fstart )
        {
            dist = internal_calculate_acc_dist_fp16( fe, fs ) + 1; 
            dist = dist >> 16;                      // convert back from fp16
            if ( dist >= (ret_val >> FPlen) )       // this cut of precision adds to the safety margin
            {
                double sq;
                sq = sqrt( fstart*fstart - (uint64)ACC_FACTOR_MMPM*(ret_val>>FPlen)/200 );
                fend = (uint32)sq + 1;
            }
        }
    }

    // save the end speed for the next iteration with cosTheta
    if ( CosTheta )
    {
        core.status.motion.prev_speed = (uint32)( ((uint64)fend * CosTheta) >> 24 ); // relative speed for the next sequence
        if ( core.status.motion.prev_speed < CNC_MIN_FEED )
            core.status.motion.prev_speed = CNC_MIN_FEED;
    }

    *feed_start = fstart;
    *feed_end = fend;
    return ret_val;
}


#define STEPPWR_HIGH    0x147AE147      // for 600mm/min - speeds abowe this need full power
#define STEPPWR_MED     0x0A3D70A3      // for 300mm/min - speeds abowe this need high power
#define STEPPWR_LOW     0x01B4F81D      // for  50mm/min - speeds abowe this need medium power

static void internal_calculate_power_levels( struct SStepFifoElement *pstep )
{
    int i;
    for ( i=0; i<CNC_MAX_COORDS; i++ )
    {
        uint32 pwr_ct;
        uint32 pwr_bgn;
        uint32 pwr_end;

        // power for constant ramp
        if ( pstep->params.go_to.act.p.Stp_ct[i] > STEPPWR_HIGH )
            pwr_ct = mpwr_full;
        else if ( pstep->params.go_to.act.p.Stp_ct[i] >= STEPPWR_MED )
            pwr_ct = mpwr_high;
        else if ( pstep->params.go_to.act.p.Stp_ct[i] >= STEPPWR_LOW )
            pwr_ct = mpwr_med;
        else
            pwr_ct = mpwr_low;

        // power for start acceleration
        if ( pstep->params.go_to.act.p.Stp_ct[i] == pstep->params.go_to.act.p.Stp_crt[i] )
        {
            pwr_bgn = pwr_ct;
        }
        else
        {
            pwr_bgn = pwr_ct + 2;
            if ( pwr_bgn > mpwr_full )
                pwr_bgn = mpwr_full;
        }

        // power for end acceleration - hardcode to maximum - needed for emergency stop
        if ( pstep->params.go_to.act.p.Stp_ct[i] )
        {
            pwr_end = pwr_ct + 2;
            if ( pwr_end > mpwr_full )
                pwr_end = mpwr_full;
        }
        else
            pwr_end = 1;

        pstep->params.go_to.pwr[i].pwr_start = pwr_bgn;
        pstep->params.go_to.pwr[i].pwr_ct = pwr_ct;
        pstep->params.go_to.pwr[i].pwr_end = pwr_end;
    }
}


static int internal_sequence_precalculate( struct SMotionSequence *crt_seq, struct SMotionSequence *next_seq, struct SStepFifoElement *pstep )
{
    uint32 dists[CNC_MAX_COORDS];
    uint32 dirmask = 0;
    uint32 maxdist = 0;
    int i;
    uint64 L = 0;

    uint32 feed_start;
    uint32 feed_end;

    uint32 stepspeed;
    uint32 startspeed;
    uint32 endspeed;

    // get the distances and direction on each axis
    dirmask = 0;
    for ( i=0; i<CNC_MAX_COORDS; i++ )
    {
        dirmask |= ( internal_run_helper_get_distance( core.status.motion.pcoord.coord[i],
                                                       crt_seq->params.go_to.coord.coord[i],
                                                       &dists[i] ) << i );
    }
    pstep->params.go_to.act.dir_mask = (uint8)dirmask;
    pstep->params.go_to.act.p.dest_poz = crt_seq->params.go_to.coord;

    // Calculate the total distances
    // L = sqrt( x*x + y*y + z*z + a*a )
    pstep->params.go_to.act.channel_active = 0;
    for ( i=0; i<CNC_MAX_COORDS; i++ )
    {
        if ( dists[i] )
        {
            pstep->params.go_to.act.channel_active |= ( 1 << i );
        }
        if ( maxdist < dists[i] )
        {
            maxdist = dists[i];
            pstep->params.go_to.act.ax_max_dist = (uint8)i;
        }
    }

    if ( pstep->params.go_to.act.channel_active == 0 )
        return -1;

    L = internal_calculate_speeds_and_distance( crt_seq, next_seq, &feed_start, &feed_end );

    // Calculate stepspeed ( see explanation abowe )
    stepspeed = internal_calculate_stepspeed( crt_seq->params.go_to.feed );      // stepspeed in fp.32 - it will use 31 bit always
    if ( crt_seq->params.go_to.feed != feed_start )
        startspeed = internal_calculate_stepspeed( feed_start ); 
    else
        startspeed = stepspeed;
    if ( crt_seq->params.go_to.feed != feed_end )
        endspeed = internal_calculate_stepspeed( feed_end ); 
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
        pstep->params.go_to.act.p.Stp_ct[i] = ( (((uint64)stepspeed) << (FPlen-4)) * dists[i] ) / ( L >> 4 ); 

        if ( startspeed == stepspeed )
            pstep->params.go_to.act.p.Stp_crt[i] = pstep->params.go_to.act.p.Stp_ct[i];
        else
            pstep->params.go_to.act.p.Stp_crt[i] = ( (((uint64)startspeed) << (FPlen-4)) * dists[i] ) / ( L >> 4 ); 

        // Acc[i] = Dist[i] * ACCFACTOR / totalDist  + 10bit shift to have FP42
        // To get acceleration in FP42 must shift with 10 bit additional.   nominator bit count: 16 + 10 + 19 + 18 = 63;    the (16+10) + 16 gives the fp42
        // max error: 5steps / 500mm
        pstep->params.go_to.act.p.Acc[i] = ( (((uint64)ACC_FACTOR_FP32) << (FPlen+10)) * dists[i] ) / L;
    }

    internal_calculate_power_levels( pstep );

    // Calculate acceleration/deceleration distances
    if ( internal_calculate_accdec_distances( pstep, L, dists, startspeed, stepspeed, endspeed ) )
        return -1;

    return 0;
}


static int internal_step_precalculator( void )
{
    // poll the input fifo, calculate and push the results in output fifo
    // Since this routine is CPU intensive - schedule it's call once or less / main loop
    struct SStepFifoElement *pstep;
    struct SMotionSequence *crt_seq;
    struct SMotionSequence *next_seq;

    if ( core.status.is_started == false )
        return 0;
    if ( internal_sequence_fifo_fullness() == 0 )       // no elements in input fifo - nothing to calculate
        return 0;
    pstep = internal_step_fifo_get_fill_pointer();      // no space in output fifo - do not do anyhing
    if ( pstep == NULL )
        return 0;

    internal_sequence_fifo_peek( &crt_seq, false );     // peek the current sequence pointer from input fifo
    internal_sequence_fifo_peek( &next_seq, true );     // peek the next one for distance / speed / and angle info

    pstep->cmdID = crt_seq->cmdID;
    pstep->seqID = crt_seq->seqID;
    pstep->seqType = crt_seq->seqType;

    if ( crt_seq->seqType == SEQ_TYPE_GOTO )
    {
        // for motion sequence we need to precalculate the parameters
        if ( core.status.motion.pcoord_updated == false )
        {                                                        // if first run after a stop/flush
            stepper_get_coord( &core.status.motion.pcoord );     // update the end coordinates with the current coordinate
            core.status.motion.pcoord_updated = true;
        }

        if ( internal_sequence_precalculate( crt_seq, next_seq, pstep ) )
            return -1;

        core.status.motion.pcoord = crt_seq->params.go_to.coord;     // save the end coordinates of the introduced sequence
    }
    else
    {
        // for the other sequences
        // (this handles the hold time also - union)
        pstep->params.spindle = crt_seq->params.spindle;
        //location for note#0001 from motion_core_internals.h
    }

    internal_sequence_fifo_get( NULL );                 // dummy get to advance the read pointer            
    return internal_step_fifo_insert( NULL );
}


static void internal_idle_power(uint32 delay)
{
    int i;
    // if auto power - set up idle power after 20ms. If a new sequence is pushed in, it will override this
    for (i=0; i<CNC_MAX_COORDS; i++)
    {
        if ( core.status.pwr[i].set_pwr == mpwr_auto )
        {
            internal_pwr_set_axis_power(i, mpwr_low, delay );
        }
    }
}


static void internal_sequencer_poll( struct SEventStruct *evt )
{
    // NOTE: It enters here only for is_started state - no need to check inside

    // check for ISR requests
    if ( isr.request )  // no mutex needed here - just verifying
    {
        int i;
        uint32 req;
        struct SStepFifoElement *elem;

        elem = stepper_get_request(&req);

        switch ( req )
        {
            case isrur_set_start_pwr:                   // request for start power setup
                for (i=0; i<CNC_MAX_COORDS; i++)
                {
                    if ( core.status.pwr[i].set_pwr == mpwr_auto )
                        internal_pwr_set_axis_power(i, elem->params.go_to.pwr[i].pwr_start, 0);
                }
                break;
            case isrur_set_ct_pwr:
                for (i=0; i<CNC_MAX_COORDS; i++)        // request for constant power setup
                {
                    if ( core.status.pwr[i].set_pwr == mpwr_auto )
                        internal_pwr_set_axis_power(i, elem->params.go_to.pwr[i].pwr_ct, 0);
                }
                break;
            case isrur_set_end_pwr:
                for (i=0; i<CNC_MAX_COORDS; i++)        // request for end power setup
                {
                    if ( core.status.pwr[i].set_pwr == mpwr_auto )
                        internal_pwr_set_axis_power(i, elem->params.go_to.pwr[i].pwr_end, 0);
                }
                break;
            case isrur_execute_inband:
                if ( core.status.inband_cb == NULL )
                {
                     // no inband callback defined - unlock immediately
                    stepper_confirm_inband();
                }
                else
                {
                    // call the inband callback
                    internal_idle_power(100);
                    core.status.inband_ex = true;
                    core.status.inband_cb( elem->seqType, elem->params.spindle );       // spindle or hold - same address - because of union
                }
                break;
        }
    }

    // check for dynamic stuff if ISR is running a sequence
    if ( stepper_check_in_progress() )
    {
        core.status.is_running = true;      // informative only - to detect stopped state to prevent reentry

        // update the speed scale factor
        if ( evt->timer_tick_10ms && 
             (core.status.motion.scale_crt != core.status.motion.scale_set) )
        {
            if (core.status.motion.scale_crt < core.status.motion.scale_set)
            {
                core.status.motion.scale_crt +=2; 
                if ( core.status.motion.scale_crt > core.status.motion.scale_set )
                    core.status.motion.scale_crt = core.status.motion.scale_set;
            }
            else
            {
                core.status.motion.scale_crt -=2;
                if ( core.status.motion.scale_crt < core.status.motion.scale_set )
                    core.status.motion.scale_crt = core.status.motion.scale_set;
            }

            stepper_scale_factor( core.status.motion.scale_crt );
        }
    }
    else if ( core.status.is_running )
    {
        core.status.is_running = false;     // prevent reentry

        // set power to idle in 20ms
        internal_idle_power(2);

        // update scale factor to immediate value
        core.status.motion.scale_crt = core.status.motion.scale_set;
        stepper_scale_factor( core.status.motion.scale_crt );

        // stepper ISR stopped executing - input fifo has elements, but step fifo is empty
        if ( internal_sequence_fifo_fullness() )
        {
            evt->cnc_motion_warn_starving = 1;
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
    {
        if ( internal_step_precalculator() )
        {
            evt->cnc_motion_seq_fatal = 1;
        }
        internal_sequencer_poll( evt );
    }

    internal_pwr_check_and_update_status( evt->timer_tick_10ms );

}



/* *************************************************
 *
 *  Interface routines
 *
 * *************************************************/


void motion_pwr_ctrl( uint32 axis, enum EPowerLevel power )
{
    if ( core.status.pwr[axis].set_pwr == (uint8)power )
        return;

    core.status.pwr[axis].set_pwr = power;
    if ( (power == mpwr_auto) && (core.status.is_running == false) )
    {
        internal_pwr_set_axis_power( axis, mpwr_low, 0 );
    }
    if ( power != mpwr_auto )
    {
        internal_pwr_set_axis_power( axis, power, 0 );
    }
}

bool motion_pwr_is_set( uint32 axis )
{
    return core.status.pwr[axis].is_dirty ? false : true;
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
    if ( core.status.is_started )
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
    return internal_sequence_fifo_insert(seq);
}


int motion_sequence_start( void )
{
    struct SEventStruct evt = { 0, };
    if ( core.status.is_started )
        return 0;

    core.status.is_started = true;
    if ( internal_step_precalculator() )
    {
        core.status.is_started = false;
        return -1;
    }
    internal_sequencer_poll( &evt );
    return 0;
}


void motion_sequence_stop( void )
{
    int i;
    if ( core.status.is_started == false )
        return;

    // flush all the fifos and the IRQ
    stepper_stop();
    internal_sequence_fifo_flush();

    // set up low power for all axis after 100ms
    internal_idle_power(10);

    // reset internal status
    core.status.motion.pcoord_updated = false;
    core.status.motion.next_precalc = false;
    core.status.motion.prev_speed = 0;

    core.status.inband_ex = false;

    core.status.motion.scale_crt = core.status.motion.scale_set;
    stepper_scale_factor( core.status.motion.scale_set );

    // reset running flags
    core.status.is_running = false;
    core.status.is_started = false;
}


uint32 motion_sequence_crt_cmdID( void )
{
    struct SStepFifoElement *elem;
    elem = stepper_get_current_in_run();        // even in stopped state
    return elem->cmdID;
}


uint32 motion_sequence_crt_seqID( void )
{
    struct SStepFifoElement *elem;
    elem = stepper_get_current_in_run();        // even in stopped state
    return elem->seqID;
}


int motion_sequence_register_callback( motion_sequence_callback func )
{
    if ( core.status.is_started )
        return -1;

    core.status.inband_cb = func;
    return 0;
}


int motion_sequence_confirm_inband_execution(void)
{
    if ( core.status.inband_ex )
    {
        core.status.inband_ex = false;
        return stepper_confirm_inband();
    }
    return -1;
}


void motion_feed_scale( int factor )
{
    core.status.motion.scale_set = factor;
    if ( core.status.is_running == false )
    {
        core.status.motion.scale_crt = factor;
        stepper_scale_factor( factor );
    }
}



TSpindleSpeed mconv_mmpm_2_sps( uint32 feed_mmpm )
{
    return ((TSpindleSpeed)( (feed_mmpm * STEP_MM) / 60 ));
}






