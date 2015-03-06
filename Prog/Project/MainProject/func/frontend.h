#ifndef FRONTEND_H
#define FRONTEND_H

#ifdef __cplusplus
    extern "C" {
#endif

    #include "cnc_defs.h"
    #include "events_ui.h"


    /*
     *  Front end module is a separated hardware with it's own communication stack
     *  That's why the operations are asynchronous
     *  Only one operation can be executed one time. Use front_end_chek_op_busy() for
     *  checking busy state, or check for ev.fe_op_completed field for completion.
     *  At completion the ev.fe_op_failed needs to be checked for failure (mainly communication failure)
     */


    #define FE_TOUCH_AXIS_A   0x01
    #define FE_TOUCH_AXIS_Z   0x02
    #define FE_TOUCH_AXIS_Y   0x04
    #define FE_TOUCH_AXIS_X   0x08
    #define FE_TOUCH_PROBE    0x10


    // main routines
    // -----------------------

    // init front-end internals, check the communication with the hardware module
    // return: 0 - front end operational, 1 - no front end available, -1 -error
    int front_end_init( void );

    // main front end polling routine
    void front_end_poll( struct SEventStruct *evt );

    // returns true if an operation is pending on the front end.
    bool front_end_chek_op_busy( void );


    // sindle routines
    // -----------------------

    // spindle power on/off
    // returns 0 for success, -1 if front end is busy with other operations
    // Async op.
    int front_end_spindle_power( bool enable );

    // set spindle speed in rpm
    // returns 0 for success, -1 if front end is busy with other operations
    // Async op.
    int front_end_spindle_speed( uint32 speed );

    // get the current RPM
    // returns 0 for success, -1 if front end is busy with other operations
    // Async op.
    int front_end_request_rpm( void );

    // get the requested rpm
    // return 0 and rpm on success, -1 if comm. failure, or request was not called
    int front_end_get_rpm( uint32 *rpm );


    // coordinate and movement related operations
    // ------------------------------------------

    // asks for measured coordinates. Operation is asynchronous, wait for front_end_chek_op_busy()
    // returns 0 if request operation is in pending, -1 if other op. is in pending
    // Async op.
    int front_end_request_coordinates( void );

    // if the coordinate request was successfull it returns the pointer to the coordinate structure.
    // in case of failure, faulty call, other op. in pending - it returns NULL
    struct SStepCoordinates *front_end_get_coordinates( void );

    // returns true when the coordinate request was sent.
    // used to check when exactly the command was shifted out to sync the front end coordinate latch with the application's coordinate latch
    // for a more precise comparison
    bool front_end_is_coordinate_request_sent( void );

    // reset the front end's internal coordinate counters to the maximum range values
    // returns 0 if request operation is in pending, -1 if other op. is in pending
    // Async op.
    int front_end_coordinate_reset_to_max( void );

    // request end point and probe touch detection for a list given in a bitmask
    // use the defines FE_TOUCH_?
    // will return -1 if other op. in pending or spindle is running - can be executed with stopped spindle only
    // Async op.
    int front_end_request_touch_detection( uint32 touch_mask );

    // if the request_touch_detection operation is completed - this returns the bitmask.
    // returns 0 - on succes, -1 if faulty call or pending
    int front_end_get_touched_list( uint32 *touch_mask );


    // generic and debug
    // ------------------------------------------

    // get the event and endpoint sensor list: evt[0] - list of events, evt[1] - list of endpoints + probe
    // evt must have 2 bytes
    // Sync op.
    int front_end_dbg_get_event_list( uint8 *evt );

    // get the physical sensor status:  ev[5] -- ev[0]: [xxxx XXXX][yyyy YYYY][zzzz ZZZZ][rpms prbe][xlim ylim][zlim aaaa]
    //                  xxxx, yyyy, zzzz - negative pulse for x,y,z encoders,
    //                  XXXX, YYYY, ZZZZ - positive pulse for x,y,z encoders
    //                  aaaa, xlim, ylim, zlim, prbe, rpms - signal on endpoint sensors, probe and rpm sensor
    //                  All of these values are 0x1 for closed contact (0V on pad) and 0x0 for open contact (Vcc pullup on pad)
    // sig must have 5 bytes
    // Sync op.
    int front_end_dbg_get_signals( uint8 *sig );

    // set raw control value for spindle regulator
    int front_end_dbg_set_raw_spindle( uint32 raw_val );




#ifdef __cplusplus
    }
#endif

#endif // FRONTEND_H
