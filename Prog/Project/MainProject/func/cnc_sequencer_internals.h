#ifndef CNC_SEQUENCER_INTERNALS_H
#define CNC_SEQUENCER_INTERNALS_H


#ifdef __cplusplus
    extern "C" {
#endif

    #include "cnc_defs.h"
    #include "cnc_sequencer.h"
    #include "command_if.h"


    #define CNCSEQ_CMD_FIFO_SIZE    64      // 28*32 = 1792 bytes

    struct SSequencerSetup
    {
        bool                        fe_present;         // if front end is present
        struct SStepCoordinates     max_travel;         // maximum reachable coordinates
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


    struct SCommandStatus
    {
        uint8                   last_cmdID;     // last successfully introduced inband cmdID, 0 if no commands were introduced since stop/flush
        uint8                   reserved;
        uint16                  last_feed;      // last set up feed speed
        struct SStepCoordinates last_coord;     // last complete coordinate used for goto command
    };

    struct SCNCStatus
    {
        struct SCommandStatus   cmd;
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
