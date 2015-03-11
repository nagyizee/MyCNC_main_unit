#ifndef FRONTEND_INTERNALS_H
#define FRONTEND_INTERNALS_H

#ifdef __cplusplus
    extern "C" {
#endif

    #include "cnc_defs.h"
    #include "frontend.h"


    #define FE_MSG_RETRY        5           // retry 5 times
    #define FE_MSG_TIMEOUT      10          // timeout on message response 1ms

    #define FE_RPM_INVALID      0xffff

    #define FE_EV_SPINDLE_OK    0x01
    #define FE_EV_SPINDLE_JAM   0x08
    #define FE_EV_ENDPOINT      0x10
    #define FE_EV_A_AXIS        0x02
    #define FE_EV_PROBE         0x80

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
        bool    spindle_on;
        uint16  spindle_rpm_set;
        uint16  spindle_rpm_get;

        bool                        coord_updated;
        struct SStepCoordinates     coord;

        uint32  touch_mask;
    };



    enum EOpStat_SpindleSpeed
    {
        opstat_spsp_reset_flags = 1,        // call dummy evmask poll to clear event flag
        opstat_spsp_set_evmask1,            // set event masks for detecting spindle jam and spindle ok
        opstat_spsp_set_speed,              // set spindle speed
        opstat_spsp_wait_flag,              // wait for event flag
        opstat_spsp_set_evmask2,            // set event maks for detecting spindle jam only
        opstat_spsp_check_events            // check the captured events to see if spindle is OK
    };

    enum EOpStat_Touch
    {
        opstat_tch_set_evmask = 1,
        opstat_tch_reset_events,
        opstat_tch_wait_flag,
        opstat_tch_set_org_evmask,
        opstat_tch_get_events,
    };

    struct SOpStat_Touch
    {
        uint32   set_mask;                   // mask set up for touch detection
    };

    struct SFrontEndOpStatus
    {
        uint8 phase;                    // operation phase - each operation has it's own EOpStat enum
        uint8 retries;                  // how many retries are available
        uint16 timeout_ctr;             // timeout counter in 100us units for receiving response
        union
        {
            bool sp_pwr;
            uint32 sp_rpm;
            struct SOpStat_Touch touch;

        } p;

        uint8 out_msg_len;              // command message lenght
        uint8 out_resp_len;             // response length to be wait to
        uint8 out_msg[10];              // command message
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
