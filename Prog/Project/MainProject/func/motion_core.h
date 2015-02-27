#ifndef MOTION_CORE_H
#define MOTION_CORE_H

#ifdef __cplusplus
    extern "C" {
#endif

    #include "cnc_defs.h"
    #include "events_ui.h"


    #define SEQ_TYPE_GOTO       1       // linear motion
    #define SEQ_TYPE_SPINDLE    2       // spindle speed
    #define SEQ_TYPE_HOLD       3       // hold time


    #define CMD_ID_INVALID         ((uint32)(-1))

    enum EPowerLevel
    {
        mpwr_off = 0,           // power off the motor driver
        mpwr_low,               // low power - set for sustaining position
        mpwr_med,               // medium power - set for slow translation
        mpwr_high,              // high power - set for normal translation / low acceleration
        mpwr_full,              // full power - set for high translation / acceleration
        mpwr_auto               // automatic power setting - motion core takes care of power level - recommended
    };


    struct SMS_Goto
    {
        struct SStepCoordinates coord;      // coordinate to go to
        TFeedSpeed feed;                    // feed speed in mm/min - sequence speed
    };

    // motion sequence element
    struct SMotionSequence
    {
        uint8   cmdID;      // command ID got from the host. It is 0 - if it is generated by sequence generator
        uint8   seqID;      // incremental sequence ID generated in sequence generator
        uint8   seqType;    // see SEQ_TYPE_xxx defines
        uint8   res;
        union UMS_params
        {
            struct SMS_Goto     go_to;      // for SEQ_TYPE_GOTO
            TSpindleSpeed       spindle;    // for SEQ_TYPE_SPINDLE
            uint32              hold;       // for SEQ_TYPE_HOLD
            // note#0001 : If other parameters are added or current prms are changed in this union, change the code in motion_core.c, search for "note#0001" in the code
        } params;
    };


    // Init motion core
    void motion_init( void );

    // Main polling loop
    void motion_poll( struct SEventStruct *evt );


    // Force axis power. power can have the following values:
    // routine is async - wait for power setup for best result
    void motion_pwr_ctrl( uint32 axis, enum EPowerLevel power );

    // gets the motor driver pwr state - if returs true - power is set up
    bool motion_pwr_is_set( uint32 axis );


    // sets up the stepper coordinate counters
    // - do not use this when executing movement
    void motion_set_crt_coord( struct SStepCoordinates *coord );

    // get the current stepper coordinates
    void motion_get_crt_coord( struct SStepCoordinates *coord );

    // Set up endpoint values. NULL will clear them
    void motion_set_max_travel( struct SStepCoordinates *coord );


    // execute a single step on an axis
    // if maximum points are not initted - it will go without boundaries
    // routine is synchronous - will wait till step fifo is emptied -> can not generate high frequency stepping
    int motion_step( uint32 axis, uint32 dir );



    // sets the spindle speed
    // 0 - spindle stopped
    // 0-TBD - spindle speed
    // Call motion_spindle_is_ok() for busy state
    void motion_spindle( TSpindleSpeed speed );

    // Scale the spindle speed with +/- factor. Value is 0 - 500.
    // 0 - no scale, +500 - speed up with x5, -500 - speed down with x5
    // it applies the factor in progressive way, no need to wait for it
    void motion_spindle_scale( int factor );

    // returns true when spindle speed is OK
    bool motion_spindle_is_ok();



    // insert a motion sequence in the sequence fifo
    int motion_sequence_insert( struct SMotionSequence *seq );

    // start motion sequence - if queue is empty it will wait for new sequence insertion,
    // when sequence is inserted it will run it right away
    int motion_sequence_start( void );

    // stop motion sequence - will produce an immediate halt if motion is in execution
    // step fifo and sequence fifo is flushed
    void motion_sequence_stop( void );

    // get the current command ID which is in execution, or the last command ID which was interrupted
    uint32 motion_sequence_crt_cmdID( void );

    // get the current sequence ID which is in execution, or the last sequence ID which was interrupted
    uint32 motion_sequence_crt_seqID( void );


    // Scale the feed speed with +/- factor. Value is 0 - 500.
    // 0 - no scale, +200 - speed up with x2, -200 - speed down with x2
    // it applies the factor in progressive way, no need to wait for it
    void motion_feed_scale( int factor );


    // Utility routine - it converts mm/min to steps/sec
    TSpindleSpeed mconv_mmpm_2_sps( uint32 feed_mmpm );




#ifdef __cplusplus
    }
#endif


#endif // MOTION_CORE_H
