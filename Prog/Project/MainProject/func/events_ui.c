
#include "events_ui.h"
#include "hw_stuff.h"

//////////////////////////////////////////////////////////////////
//
//      ISR RELATED DEFINES AND ROUTINES - handle with care
//
//////////////////////////////////////////////////////////////////


#ifdef ON_QT_PLATFORM
static struct STIM1 stim;
static struct STIM1 *TIM1 = &stim;
#endif

#define MAX_ISR_STEPS       8
#define MAX_ISR_WRAPMASK    0x07

volatile struct SEventStruct events = { 0, };


static struct SAxisControl
{
    struct SAxisData				// MAX_ISR_STEPS element step fifo from software
    {
        uint32   tick;				// clock bitmask
        uint32   dir;				// direction bitmask.   1 - cws or plus, 0 - ccws or minus
    } axis[MAX_ISR_STEPS];

    struct SStepCoordinates Poz;	// hardware step counter. Reflects the current milling head position

    uint32   wp;
    uint32   rp;
    uint32   c;
    uint32   SavedTicks;			// saved clock bitmask used internally. set up at 50us when direction is set up, executed and cleared at 100us

} AxisControl;


static struct SLEDOperations
{
    struct SLEDparams
    {
        uint16  type;
        uint16  counter;

    } led[3];
} LEDop;


static struct SButtonOp
{
    uint16  countdown;
    uint16  prew_state;
} ButtOp = {0, };


/////////////////////////////////////////////////////////////////////////////////////
/// -------- IRQ Routines ---------
/////////////////////////////////////////////////////////////////////////////////////

void StepTimerIntrHandler (void)
{
    static uint32 counter = 0;

    // Clear update interrupt bit
    TIMER_SYSTEM->SR = (uint16)~TIM_FLAG_Update;

    // generate clock signals
    if ( counter & 0x01 )
    {
        // process emergency stop button
        events.timer_tick_system    = 1;

        // Set Clock signals
        if ( AxisControl.SavedTicks & (1 << COORD_X) )
            HW_StepClk_X();
        if ( AxisControl.SavedTicks & (1 << COORD_Y) )
            HW_StepClk_Y();
        if ( AxisControl.SavedTicks & (1 << COORD_Z) )
            HW_StepClk_Z();
        if ( AxisControl.SavedTicks & (1 << COORD_A) )
            HW_StepClk_A();
        AxisControl.SavedTicks = 0;
    }
    // set up directions and check for clock signal
    else
    {
        HW_StepClk_Reset();

        if ( BtnGet_Emerg() )
        {
            events.emerg_button = 1;
        }

        // set up driver dirs
        if ( AxisControl.c && (events.emerg_button == 0) )
        {
            uint32 tick;
            uint32 dir;

            tick = AxisControl.axis[AxisControl.rp].tick;
            dir = AxisControl.axis[AxisControl.rp].dir;

            if ( tick & (1 << COORD_X) )
            {
                if ( dir & (1 << COORD_X) )
                {
                    HW_SetDirX_Plus();
                    AxisControl.Poz.coord[COORD_X]++;
                }
                else
                {
                    HW_SetDirX_Minus();
                    AxisControl.Poz.coord[COORD_X]--;
                }
            }
            if ( tick & (1 << COORD_Y) )
            {
                if ( dir & (1 << COORD_Y) )
                {
                    HW_SetDirY_Plus();
                    AxisControl.Poz.coord[COORD_Y]++;
                }
                else
                {
                    HW_SetDirY_Minus();
                    AxisControl.Poz.coord[COORD_Y]--;
                }
            }
            if ( tick & (1 << COORD_Z) )
            {
                if ( dir & (1 << COORD_Z) )
                {
                    HW_SetDirZ_Plus();
                    AxisControl.Poz.coord[COORD_Z]++;
                }
                else
                {
                    HW_SetDirZ_Minus();
                    AxisControl.Poz.coord[COORD_Z]--;
                }
            }
            if ( tick & (1 << COORD_A) )
            {
                if ( dir & (1 << COORD_A) )
                {
                    HW_SetDirA_Plus();
                    AxisControl.Poz.coord[COORD_A]++;
                }
                else
                {
                    HW_SetDirA_Minus();
                    AxisControl.Poz.coord[COORD_A]--;
                }
            }
            AxisControl.SavedTicks = tick;
            AxisControl.rp++;
            AxisControl.rp &= MAX_ISR_WRAPMASK;
            AxisControl.c--;
            if ( AxisControl.c == 0 )
            {
                events.cnc_warn_last_step = 1;
            }

        }
        else
        {
            events.cnc_warn_no_more_steps = 1;
        }
    }

    counter++;
    if ( counter == SYSTEM_T_10MS_COUNT )
    {
        counter = 0;
        events.timer_tick_10ms  = 1;
    }

}



/////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////

uint32  GetStepQ( void )
{
    return AxisControl.c;
}


uint32 ClearStepQ( void )
{
    uint32  stepsInQ;
    __disable_interrupt();
    stepsInQ = AxisControl.c;
    AxisControl.c     = 0;
    AxisControl.rp    = 0;
    AxisControl.wp    = 0;
    AxisControl.SavedTicks = 0;
    __enable_interrupt();
    return stepsInQ;
}

uint32  InsertClock( uint32 tick, uint32 dir )        // clock and dir are bitfields with the 4 axis
{
    __disable_interrupt();
    if ( AxisControl.c == MAX_ISR_STEPS )
    {
        __enable_interrupt();
        return 1;       // fifo full;
    }

    AxisControl.axis[AxisControl.wp].dir    = dir;
    AxisControl.axis[AxisControl.wp].tick   = tick;
    AxisControl.wp++;
    AxisControl.wp &= 0x03;
    AxisControl.c++;
    __enable_interrupt();
    return 0;
}


void    StepCounter_Setup( struct SStepCoordinates *coord )
{
    __disable_interrupt();
    AxisControl.Poz = *coord;
    __enable_interrupt();
}

void    StepCounter_Get( struct SStepCoordinates *coord )
{
    __disable_interrupt();
    *coord  = AxisControl.Poz;
    __enable_interrupt();
}



void Process_LED( void )
{
    // routine called at 10ms interval
    static
    uint32 i;

    for (i=0; i<3; i++)
    {
        if ( LEDop.led[i].type == LED_blink_fast )
        {
            // 4 blinks / sec
            if ( LEDop.led[i].counter == 0 )
            {
                LED_On( i );
            }
            if ( LEDop.led[i].counter == 13 )
            {
                LED_Off( i );
            }
            LEDop.led[i].counter++;
            if ( LEDop.led[i].counter == 25 )   // 250ms
            {
                LEDop.led[i].counter = 0;
            }
        }
        else if ( LEDop.led[i].type == LED_blink_slow )
        {
            // 1 blink / sec
            if ( LEDop.led[i].counter == 0 )
            {
                LED_On( i );
            }
            else if ( LEDop.led[i].counter == 50 )
            {
                LED_Off( i );
            }
            LEDop.led[i].counter++;
            if ( LEDop.led[i].counter == 100 )  // 1000ms
            {
                LEDop.led[i].counter = 0;
            }
        }
    }
}//END: Process_LED



void Process_Button( struct SEventStruct *ev )
{
    uint16 prew_state = ButtOp.prew_state;

    ButtOp.countdown++;
    if ( ButtOp.countdown < 10 )
        return;

    ButtOp.countdown = 0;

    if ( !(prew_state & BUTTON_RESUME) && BtnGet_Resume() )
    {
        ev->button_pressed_resume = 1;
        prew_state |= BUTTON_RESUME;
    }
    else if ( (prew_state & BUTTON_RESUME) && !BtnGet_Resume() )
    {
        prew_state &= ~BUTTON_RESUME;
    }
    if ( !(prew_state & BUTTON_TOOLCH) && BtnGet_Home() )
    {
        ev->button_pressed_toolchange = 1;
        prew_state |= BUTTON_TOOLCH;
    }
    else if ( (prew_state & BUTTON_TOOLCH) && !BtnGet_Home() )
    {
        prew_state &= ~BUTTON_TOOLCH;
    }
    if ( (prew_state & BUTTON_EMERG) && !BtnGet_Emerg() )
    {
        prew_state &= ~BUTTON_EMERG;
    }
    ButtOp.prew_state =  prew_state;
}//END: Process_Button



void LED_Op( uint32 led, enum ELED_operation op )
{
    switch ( op )
    {
        case LED_on:
            LED_On( led );
            break;
        case LED_off:
            LED_Off( led );
            break;
        case LED_blink_fast:
        case LED_blink_slow:
            if ( LEDop.led[led].type != (uint16)op )
            {
                LEDop.led[led].counter = 0;
            }
            break;
    }
    LEDop.led[led].type = (uint16)op;
}


struct SEventStruct Event_Poll(void)
{
    uint32  ev;
    struct SEventStruct evtemp = { 0, };

    // process main communication port events
/*old
 *  ev = Uart_GetFlag( PORT_COM );

    if ( ev & UART_FLAG_EVENT_CHAR )
    {
        evtemp.comm_new_request = 1;
    }
    if ( ev & UART_FLAG_ERR_HW_OVERFLOW )
    {
        Uart_Send( PORT_COM, ">E: uart_hw_overflow\n\r");
        evtemp.comm_error_in_full   = 1;
    }
    if ( ev & UART_FLAG_ERR_SW_OVERFLOW )
    {
        Uart_Send( PORT_COM, ">E: uart_buff_overflow\n\r");
        evtemp.comm_error_in_full   = 1;
    }

    Uart_ClearFlag( PORT_COM, UART_FLAG_EVENT_CHAR | UART_FLAG_ERR_SW_OVERFLOW | UART_FLAG_ERR_HW_OVERFLOW );
*/

    // process separately the emergency button - if pressed take the event immediately
    if ( events.emerg_button && BtnGet_Emerg() )
    {
        events.button_pressed_emerg = 1;
        ButtOp.countdown = 0;
        ButtOp.prew_state |= BUTTON_EMERG;
    }
    // LED and Keyboard operations
    if ( events.timer_tick_10ms )
    {
        Process_LED();
        Process_Button( &evtemp );
    }

    __disable_interrupt();
    *((uint32*)&events) |= *((uint32*)&evtemp);
    __enable_interrupt();

    return events;
}//END: Event_Get


void Event_Clear( struct SEventStruct evmask)
{
    uint32 *ev  = (uint32*)&events;
    uint32 tmp  = *(  (uint32*)(&evmask) );

    __disable_interrupt();
    *ev &= ~( tmp );
    __enable_interrupt();

}//END: Event_Clear


