#ifndef FRONTEND_INTERNALS_H
#define FRONTEND_INTERNALS_H

#ifdef __cplusplus
    extern "C" {
#endif

    #include "cnc_defs.h"
    #include "frontend.h"


    enum EFrontEndOperations
    {
        feop_none = 0,          // no operation pending - busy state false
        feop_get_coord,         // get coordinates in progerss
        feop_touch_detect,      // touch detection in progress
        feop_set_spindle        // set spindle in progress
    };




    struct SFrontEndStruct
    {
        bool in_use;                        // set to true if front end is in use, false if not connected or could not be set up

        enum EFrontEndOperations    op;     // current operation

    };


#ifdef __cplusplus
    }
#endif


#endif // FRONTEND_INTERNALS_H
