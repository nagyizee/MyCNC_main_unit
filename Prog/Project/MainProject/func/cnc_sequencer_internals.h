#ifndef CNC_SEQUENCER_INTERNALS_H
#define CNC_SEQUENCER_INTERNALS_H


#ifdef __cplusplus
    extern "C" {
#endif

    #include "cnc_defs.h"
    #include "cnc_sequencer.h"
    #include "command_if.h"


    #define CNCSEQ_CMD_FIFO_SIZE    12      // 28*32 = 1792 bytes

    #define SEQ_FEED_PWR_HIGH       700
    #define SEQ_FEED_PWR_MED        400
    #define SEQ_FEED_PWR_LOW        200     // TBD

    #define SEQ_PERIOD_COORD_CHK_FRUN       100     // 10.0ms
    #define SEQ_PERIOD_COORD_CHK_OUTBAND    100     // 10.0 ms
    #define SEQ_PERIOD_COORD_CHK_INBAND     20      // 2.0 ms   ( 0.05mm with max speed on 10khz pulse - 1500mm/min )
    #define SEQ_PERIOD_FRUN_SAFESTOP        2000    // 200.0ms

    #define SEQ_MAX_COORDDEV_FRUN       5       // maximum 5 steps are allowed as deviation at freerun  
    #define SEQ_MAX_COORDDEV_OUTBAND    2
    #define SEQ_MAX_COORDDEV_INBAND     2

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

    enum EOutbandStatus_findorg
    {
        obstat_forg_spindledn = 0,          // power down the spindle
        obstat_forg_back_off,               // clear the coarse end point sensors
        obstat_forg_search,                 // search the end points
        obstat_forg_finalize,               // reset the front-end coordinates
    };

    enum EOutbandStatus_findZ
    {
        obstat_findz_spindledn = 0,          // power down the spindle
        obstat_findz_goXY,                   // go to the Z probe position
        obstat_findz_search,                 // search the end points
    };


    struct SSequencerSetup
    {
        bool                        fe_present;         // if front end is present
        struct SStepCoordinates     max_travel;         // maximum reachable coordinates
        struct SStepCoordinates     home_poz;           // machine home poz / tool change poz
        struct SStepCoordinates     z_probe;            // position of the tooltip probe
        uint16                      feed_max;           // maximum allowed feed speed
        uint16                      feed_rapid;         // rapid transition speed used for internal sequences
        uint16                      spindle_max;        // maximum spindle speed
        uint16                      spindle_min;        // minimum spindle speed
        int32                       spindle_scale;      // scale the spindle speed up/down +/- 200%
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


    struct SOutbandProbeInfo
    {
        bool valid;                     // info is vslid in this structure
        struct SStepCoordinates poz;    // probe thouch pozition 
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

    struct SOutbandStatus_findOrg
    {
        uint32  axis_mask;          // axis with unfinished job

    };

    struct SProc_getCoord
    {
        struct SStepCoordinates coord_snapshot;     // coordinates in motion core when front-end receives the coordinate read command
        struct SStepCoordinates coord_fe;           // coordinates in motion core when front-end receives the coordinate read command
    };

    struct SInbandOp_SetSpindle
    {
        uint16  speed;              // set speed
        uint8   retry;              // retrial times
        uint8   timeout;            // timeout bw. retrials
        bool    not_callback;       // if true then restart needs to be done - not callback confirm
    };

    struct SInbandResumeParams
    {
        struct SStepCoordinates     last_erased_coord;      // final coordinates of the last movement command which was erased (or the position from where start was given)
        struct SStepCoordinates     stopped_soft_coord;     // software coordinates at the moment of stopping (may not coincide with physical coordinates in case of missed steps)
        struct SStepCoordinates     stopped_hard_coord;     // physical coordinates at the moment of stopping
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
            uint32 err_starv:1;         // motion core starvation
            uint32 err_fatal:1;         // fatal error produced - everything halted

            uint32 run_outband:1;       // running an outband command 
            uint32 run_program:1;       // running a program from inband fifo (can be combined with run_paused)
            uint32 run_paused:1;        // inband sequence paused
            uint32 run_ob_failed:1;     // set if an outband command failed - reset at new command
            uint32 run_ob_suceeded:1;   // set if an outband is finished with success - reset at new command

            uint32 stat_restarted:1;    // set to 1 at start-up, reset at the first status read
            uint32 stat_ep_set:1;       // 0 at start-up, set when endpoint finding command is terminated or if no front-end and max travel is set up
            uint32 stat_bpress:1;       // action because of button press

            uint32 spindle_pwr:1;       // set if spindle is powered up and ready to go
            uint32 spindle_on:1;        // set when spindle is running

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
            struct SOutbandStatus_findOrg   findorg;
            struct SOutbandStatus_gohome    findZ;

        } params;
        struct SOutbandProbeInfo    probe;
    };

    struct SCNCInband
    {
        bool    mc_started;         // if motion core is started - it will be started when input command fifo is empty or motion sequence fifo is full
        bool    cmd_on_hold;        // set when a command needs to generate more sequences, reset when all the sequences are pushed
        bool    restartable;        // inband execution is stopped in a restartable way

        uint32  cb_operation;       // callback operation executed, see CMD_IB_XXX. - 0 if none.
        union 
        {
            uint32                      wait;       // waiting countdown
            struct SInbandOp_SetSpindle spindle;    // spindle control
        }       cb_op;

        uint32  check_coord;
        uint32  coord_fail;

        struct SInbandResumeParams      resume;     // resume parameters used when inband command execution is interrupted and resume is requested
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

    struct SCNCMisc
    {
        uint32 spindle_speed;                           // speed set by inband or outband (not scaled) - Note: this doesn't reflect the ON status
        uint32 coord_dump_ctr;                          // coordinate dump period. set to 0 to disable dump
    };

    struct SCNCStatus
    {
        struct SCommandStatus   cmd;            // generic command status
        union SStatusFlags      flags;          // cnc sequencer status flags
        struct SCNCOutband      outband;        // outband command execution status
        struct SCNCInband       inband;         // inband related status
        struct SCNCProcedure    procedure;      // internal procedure
        struct SCNCMisc         misc;           // spindle setup
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
