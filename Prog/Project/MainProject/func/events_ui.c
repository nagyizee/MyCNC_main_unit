
#include "events_ui.h"
#include "hw_stuff.h"

//////////////////////////////////////////////////////////////////
//
//      ISR RELATED DEFINES AND ROUTINES - handle with care
//
//////////////////////////////////////////////////////////////////


volatile struct SEventStruct events = { 0, };


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


// this is called by step timer ISR
void event_ISR_set10ms(void)
{
    events.timer_tick_10ms  = 1;
}

void event_ISR_set100us(void)
{
    events.timer_tick_100us = 1;
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
    struct SEventStruct evtemp = { 0, };

    // process main communication port events
/*old
    uint32  ev;
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


