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


    struct SMotionGotoOp
    {
        uint32  dirmask;                        // bitmask with directions
        uint32  Tctr;                           // systime loop counter in Int
        uint64  Ttot;                           // total runtime in fp32
        uint64  StepCkInc[CNC_MAX_COORDS];      // sysclock increments for each steps
        uint64  StepCkCntr[CNC_MAX_COORDS];     // sysclock for the next step

    };


    union UMotionOperation
    {
        struct SMotionGotoOp    go_to;


    };

    struct SMotionStatus
    {
        bool is_running;                    // sequencer is running an active sequence ( busy state )
        bool is_started;                    // sequencer is started

        struct SMotionSequence  crt_seq;    // current sequence in run
        union UMotionOperation  op;
    };


    struct SMotionCoreInternals
    {

        struct SStepCoordinates crt_coord;      // current soft coordinates. They are considered before the step fifo
        struct SStepCoordinates max_travel;     // maximum step number on each axis

        struct SMotionSequenceFifo seq_fifo;    // motion sequence fifo
        struct SMotionStatus    status;         // insternal status of the motion core
    };






#ifdef __cplusplus
    }
#endif


#endif // MOTION_CORE_INTERNALS_H
