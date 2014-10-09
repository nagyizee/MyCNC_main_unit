/**
  ******************************************************************************
  * 
  *   System and User Interface related timing routines
  *   Main step clock generator - final physical step fifo
  *   Handles the system timer, sensor infos and button / led actions
  * 
  ******************************************************************************
  */ 

#ifndef __EVENT_UI_H
#define __EVENT_UI_H

#ifdef __cplusplus
    extern "C" {
#endif

    #include "stm32f10x.h"
    #include "typedefs.h"
    #include "cnc_defs.h"


    struct SEventStruct
    {
        uint32  timer_tick_system:1;        // CNC system tick - timing given by STEP_QUANTUM, the minimum timing for max step speed
        uint32  timer_tick_10ms:1;          // 10ms timing tick

        uint32  comm_new_request:1;         // new message from communication port
        uint32  comm_error_in_full:1;       // input queue full error

        uint32  cnc_warn_no_more_steps:1;   // stepping fifo emptied
        uint32  cnc_warn_last_step:1;       // last step command issued from the fifo

        uint32  emerg_button:1;             // emergency button pressed - button state
        uint32  button_pressed_emerg:1;     // emergency button pressed - refresh - instant
        uint32  button_pressed_resume:1;
        uint32  button_pressed_toolchange:1;
    };


    enum    ELED_operation
    {
        LED_off = 0,
        LED_on,
        LED_blink_slow,
        LED_blink_fast
    };

    #define LED_SYSTEM      0
    #define LED_PROCESSING  1
    #define LED_FAILURE     2

    #define BUTTON_EMERG    0x01
    #define BUTTON_RESUME   0x02
    #define BUTTON_TOOLCH   0x04


    struct SEventStruct Event_Poll(void);
    void    Event_Clear(struct SEventStruct evmask);

    void    LED_Op( uint32 led, enum ELED_operation op );

#ifdef __cplusplus
    }
#endif

#endif
