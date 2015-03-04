#ifndef FRONTEND_INTERNALS_H
#define FRONTEND_INTERNALS_H

#ifdef __cplusplus
    extern "C" {
#endif

    #include "cnc_defs.h"
    #include "frontend.h"


    #define FE_MSG_RETRY        5           // retry 5 times
    #define FE_MSG_TIMEOUT      10          // timeout on message response 1ms


    enum EFrontEndOperations
    {
        feop_none = 0,          // no operation pending - busy state false
        feop_coord_get,         // get coordinates in progress
        feop_coord_reset,       // reset coordinates
        feop_touch_detect,      // touch detection in progress
        feop_spindle_pwr,       // spindle power
        feop_spindle_get_rpm,   // get spindle rpm
        feop_spindle_set_rpm,   // set spindle speed in progress
    };


    struct SFrontEndStatus
    {
        uint8   spindle_on;
        uint16  spindle_rpm_set;
        uint16  spindle_rpm_get;

    };



    enum EOpStatSpindlePwr
    {
        opst_spinpwr_wait_ack = 1,
    };


    struct SOpstat_spindlePwr
    {
        bool  value; 
    };


    struct SFrontEndOpStatus
    {
        uint8 phase;                    // operation phase - each operation has it's own EOpStat enum
        uint8 retries;                  // how many retries are available
        uint8 timeout_ctr;              // timeout counter in 100us units for receiving response
        uint8 reserved;
        union
        {
            bool sp_pwr;

        } p;
    };


    struct SFrontEndStruct
    {
        bool in_use;                            // set to true if front end is in use, false if not connected or could not be set up

        enum EFrontEndOperations    op;         // current operation
        bool                        no_change;  // marked when no interaction needs to be done with front end hardware - returns completion at the first poll call

        struct SFrontEndStatus      status;
        struct SFrontEndOpStatus    opstat;

    };


#ifdef __cplusplus
    }
#endif


#endif // FRONTEND_INTERNALS_H
