#ifndef MOTION_CORE_INTERNALS_H
#define MOTION_CORE_INTERNALS_H

#ifdef __cplusplus
    extern "C" {
#endif

    #include "cnc_defs.h"
    #include "motion_core.h"

    #define MAX_SEQ_FIFO       32        // 
    #define MAX_STEP_FIFO      20        // 1840 bytes fifo

    #define START_FEED         100       // use 100mm/min as start-up feed - it is used for stopping also
    #define MINIMUM_FEED        20       // this is the absolute minimum feed speed usable on the machine

    struct SMotionCoreActionCommon              // struct. size = 84
    {
        struct SStepCoordinates dest_poz;       // destination coordinate
        uint32 Stp_crt[CNC_MAX_COORDS];         // current speed - fractional to 32:32fp format (or start speed)
        uint32 Stp_ct[CNC_MAX_COORDS];          // constant speed - fractional to 32:32fp format
        uint32 Acc[CNC_MAX_COORDS];             // acceleration increments on each axis ( steps/stepck^2 ) given in FP42 held in 32bit container
        uint64 accdec[2];                       // [0] - accelerate to this distance point, [1] - decelerate from this point - it is in FP32
        uint32 accsens;                         // bit 0 -> beginning 1-positive 0-negative, bit 1 -> ending 1-positive 0-negative 
    };



    struct SMFEL_Goto                           // struct size = 84 + 4 = 88
    {
        struct SMotionCoreActionCommon     p;   // parameters

        uint8 channel_active;                   // Bit 1 means that we have movement on the corresponding coordinate, 0 - if no movement on coordinate
        uint8 ax_max_dist;                      // axis index with maximum distance - acceleration / constant / deceleration points are given on this
        uint8 dir_mask;                         // direction mask ( 1- plus, 0- minus )
        uint8 reserved;
    };                                          // - total: 44bytes


    struct SStepFifoElement                     // motion element for stepper ISR
    {                                           // struct size = 4 + 88 = 92
                                            
        uint8   cmdID;      // command ID got from the host. It is 0 - if it is generated by sequence generator
        uint8   seqID;      // incremental sequence ID generated in sequence generator
        uint8   seqType;    // see SEQ_TYPE_xxx defines
        uint8   res;
        union UMFEL_params
        {
            struct SMFEL_Goto   go_to;      // for SEQ_TYPE_GOTO
            TSpindleSpeed       spindle;    // for SEQ_TYPE_SPINDLE
            uint32              hold;       // for SEQ_TYPE_HOLD
            // note#0001 : If other parameters are added or current prms are changed in this union, change the code in motion_core.c, search for "note#0001" in the code
        } params;
    };                      // - total: 48 bytes


    struct SMotionSequenceFifo                          // input fifo to the motion core
    {
        struct SMotionSequence    seq[MAX_SEQ_FIFO];
        uint8   c;
        uint8   w;
        uint8   r;
    };

    struct SMotionStepperFifo                           // output fifo for feeding the stepper ISR
    {
        struct SStepFifoElement    stp[MAX_STEP_FIFO];
        uint8   c;
        uint8   w;
        uint8   r;
    };


    #define GOTO_PHASE_CT       0
    #define GOTO_PHASE_ACCEL    1
    #define GOTO_PHASE_DECEL    2


    struct SMotionInternals
    {
        bool                        pcoord_updated;     // false if this is the first entry since stop or flush, true - if pcood is allready updated
        struct SStepCoordinates     pcoord;             // previous coordinate - used for distance calculation when introducing sequences in fifo

        bool                        next_precalc;       // false if no precalculation is done for the next sequence                                         
        uint64                      next_L;             // precalculated length of the next sequence

        TFeedSpeed                  prev_speed;         // last speed * cosTheta calculated from the previous sequence  

    };


    struct SMotionStatus
    {
        bool is_running;                        // sequencer is running an active sequence ( busy state )
        bool is_started;                        // sequencer is started (may not run anyhting if sequence fifo is empty)

        struct SStepFifoElement  crt_stp;       // sequence currently in run in ISR
        struct SMotionInternals  motion;        // internal status and saved precalculations for generating the next motion
    };


    struct SMotionCoreInternals                 // Main core structure
    {
        struct SStepCoordinates     max_travel;     // maximum step number on each axis
        struct SMotionStepperFifo   stepper_fifo;   // output fifo for the stepper IRQ
        struct SMotionSequenceFifo  sequence_fifo;  // input fifo in the motion core
        struct SMotionStatus        status;         // insternal status of the motion core
    };


    // ISR related defines

    #define MCISR_STATUS_IDLE       0       // no movement is done
    #define MCISR_STATUS_RUP        1       // speed ramp up
    #define MCISR_STATUS_CT         2       // constant speed
    #define MCISR_STATUS_RDOWN      3       // speed ramp down


    struct SMotionCoreISRaction
    {
        struct SMotionCoreActionCommon  p;      // parameters

        uint32 channel_active;                  // Bit 1 means that we have movement on the corresponding coordinate, 0 - if no movement on coordinate.
                                                //      it is used also to signal the application about finishing the action: if 0 - means that action is finished
        uint32 ax_max_dist;                     // axis index with maximum distance - acceleration / constant / deceleration points are given on this
        uint32 dir_mask;                        // direction mask ( 1- plus, 0- minus )
        uint32 seq_id;                          // sequence ID ( identifies the currently working sequence )
    };


    struct SMotionCoreISRop
    {
        uint64  D[CNC_MAX_COORDS];              // distance in 32:32fp - the upper part is the step integer part, the lower part is the fractional part
        uint32  Dprev[CNC_MAX_COORDS];
        uint64  ctr_speed64[CNC_MAX_COORDS];    // current speed in fp64. We need the extra precision for small acceleration accumulation

    };


    struct SMotionCoreISR
    {
        uint32  state;                          // see MCISR_STATUS_XXX defines

        struct SMotionCoreISRaction crt_action; // action passed from user level
        struct SMotionCoreISRaction next_action;// next movement action to be executed - set up by the user level core code
        struct SMotionCoreISRop     op;         // operation

        uint32  ckmask;                         // bitmask with channels where clock signal is set
        uint32  ckoff[CNC_MAX_COORDS];          // clock off timeout

        struct SStepCoordinates crt_poz;        // current position
                                                
    };



#ifdef __cplusplus
    }
#endif


#endif // MOTION_CORE_INTERNALS_H
