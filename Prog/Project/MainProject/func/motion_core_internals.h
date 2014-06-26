#ifndef MOTION_CORE_INTERNALS_H
#define MOTION_CORE_INTERNALS_H

#ifdef __cplusplus
    extern "C" {
#endif

    #include "cnc_defs.h"
    #include "motion_core.h"

    #define MAX_SEQ_FIFO        20

    struct SMotionSequenceFifo
    {
        struct SMotionSequence  seq[MAX_SEQ_FIFO];
        uint8   c;
        uint8   w;
        uint8   r;
    };


    struct SMotionCoreInternals
    {
        struct SStepCoordinates crt_coord;      // current soft coordinates. They are considered before the step fifo
        struct SStepCoordinates max_travel;     // maximum step number on each axis

        struct SMotionSequenceFifo seq_fifo;    // motion sequence fifo

    };






#ifdef __cplusplus
    }
#endif


#endif // MOTION_CORE_INTERNALS_H
