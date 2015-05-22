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
        uint32  timer_tick_100us:1;         // CNC system tick - timing given by STEP_QUANTUM, the minimum timing for max step speed
        uint32  timer_tick_10ms:1;          // 10ms timing tick

        uint32  cnc_motion_warn_starving:1; // if motion core is running sequences, input fifo has sequences but output fifo is emptied and IRQ staves
        uint32  cnc_motion_seq_fatal:1;     // fatal error produced in sequence execution (like out of boundaries or impossible speed setup error)
        uint32  cnc_motion_seq_finished:1;  // ISR finished executing a sequence

        uint32  button_pressed_emerg:1;     // emergency button pressed - refresh - instant
        uint32  button_pressed_resume:1;
        uint32  internal_outband_gohome:1;

        uint32  fe_spindle_jam:1;           // flag set by front_end_poll when spindle jam is detected
        uint32  fe_op_completed:1;          // flag set by front_end_poll when an operation is finished - set for succeeded and failed also
        uint32  fe_op_failed:1;             // flag set by front_end_poll when an operation is failed ( set together with fe_op_completed )

        uint32  comm_command_ready:1;       // if a new command is available
        uint32  comm_input_overflow:1;      // input queue overflowed
        uint32  comm_timeout:1;             // communication timeout detected

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
