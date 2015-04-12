#ifndef CNC_SEQUENCER_INTERNALS_H
#define CNC_SEQUENCER_INTERNALS_H


#ifdef __cplusplus
    extern "C" {
#endif

    #include "cnc_defs.h"
    #include "cnc_sequencer.h"
    #include "command_if.h"


    #define CNCSEQ_CMD_FIFO_SIZE    64      // 28*32 = 1792 bytes

    #define SEQ_FEED_PWR_HIGH       700
    #define SEQ_FEED_PWR_MED        400
    #define SEQ_FEED_PWR_LOW        200     // TBD

    #define SEQ_PERIOD_COORD_CHK_FRUN       100     // 10.0ms
    #define SEQ_PERIOD_COORD_CHK_OUTBAND    100     // 10.0 ms
    #define SEQ_PERIOD_FRUN_SAFESTOP        1500    // 150.0ms

    #define SEQ_MAX_COORDDEV_FRUN       5       // maximum 5 steps are allowed as deviation at freerun  
    #define SEQ_MAX_COORDDEV_OUTBAND    2

    #define SEQ_MAX_MOVEMENT_RETRIALS   5

    enum EProcedureID
    {
        procid_none = 0,
        procid_getcoordinates,                      // get measured coordinates from front-end
        procid_spindle_pwrup,
        procid_spindle_pwrdown,
    };

    enum EOutbandStatus_freerun
    {
        obstat_frun_running = 0,
        obstat_frun_stopping,
    };

    enum EOutbandStatus_gohome
    {
        obstat_home_preparation = 0,        // used for stopping and power down the spindle
        obstat_home_motion,                 // moving milling head to home pozition
    };


    struct SSequencerSetup
    {
        bool                        fe_present;         // if front end is present
        struct SStepCoordinates     max_travel;         // maximum reachable coordinates
        struct SStepCoordinates     home_poz;           // machine home poz / tool change poz
        struct SStepCoordinates     z_probe;            // position of the tooltip probe
        uint16                      feed_max;           // maximum allowed feed speed
        uint16                      feed_rapid;         // rapid transition speed used for internal sequences
    };
    

    struct SCommandFifo
    {
        struct ScmdIfCommand    cmd[CNCSEQ_CMD_FIFO_SIZE];
        uint8 r;        // read pointer - elements are read but not erased so can not be overwritten
        uint8 e;        // eraser - write pointer can only go till this position
        uint8 w;        // write pointer
        uint8 wrtb;     // writeable locations - empty space in the fifo
        uint32 rdb;     // readable locations - valid data in the fifo
    };


    struct SOutbandStatus_freerun
    {
        uint32  safety_to;          // safety timeout
        uint32  coord_check;        // coordinate check period
    };

    struct SOutbandStatus_gohome
    {
        uint32 ccheck;              // coordinate check iv. for go home
        uint32 m_fail;              // failure counter
    };

    struct SProc_getCoord
    {
        struct SStepCoordinates coord_snapshot;     // coordinates in motion core when front-end receives the coordinate read command
        struct SStepCoordinates coord_fe;           // coordinates in motion core when front-end receives the coordinate read command
    };

    struct SCommandStatus
    {
        uint8                   last_cmdID;     // last successfully introduced inband cmdID, 0 if no commands were introduced since stop/flush
        uint8                   reserved;
        uint16                  last_feed;      // last set up feed speed
        struct SStepCoordinates last_coord;     // last complete coordinate used for goto command
    };


    union SStatusFlags 
    {
        struct 
        {
            uint32 err_code:8;          // fatal error code (valid if err_fatal is set)
            uint32 err_spjam:1;         // spindle jam detected
            uint32 err_step:1;          // missed step detected
            uint32 err_fatal:1;         // fatal error produced - everything halted

            uint32 run_outband:1;       // running an outband command 
            uint32 run_program:1;       // running a program from inband fifo
            uint32 run_ob_failed:1;     // set if an outband command failed - reset at new command
            uint32 run_ob_suceeded:1;   // set if an outband is finished with success - reset at new command
            uint32 run_sequence:1;      // used in conjungtion with run_outband or run_program when a sequence qeue is in run

            uint32 stat_restarted:1;    // set to 1 at start-up, reset at the first status read
            uint32 stat_ep_set:1;       // 0 at start-up, set when endpoint finding command is terminated or if no front-end and max travel is set up


        } f;
        uint32 byte;
    };

    struct SCNCOutband
    {
        struct ScmdIfCommand    command;        // cmd_type 0 - if no command in execution, otherwise see CMD_OBXXXXX defines
        uint32                  state;          // execution state
        union 
        {
            struct SOutbandStatus_freerun   frun;
            struct SOutbandStatus_gohome    gohome;

        } params;
    };

    struct SCNCProcedure
    {
        enum EProcedureID       procID;         // procedure ID
        union 
        {
            struct SProc_getCoord   getcoord;
            int                     spindle_off;

        }                       params;
    };

    struct SCNCStatus
    {
        struct SCommandStatus   cmd;            // generic command status
        union SStatusFlags      flags;          // cnc sequencer status flags
        struct SCNCOutband      outband;        // outband command execution status
        struct SCNCProcedure    procedure;      // internal procedure
    };


    struct SCNCSequencerInternals
    {
        struct SSequencerSetup  setup;
        struct SCommandFifo     cmd_fifo;
        struct SCNCStatus       status;
    };
        



#ifdef __cplusplus
    }
#endif

#endif // CNC_SEQUENCER_INTERNALS_H
