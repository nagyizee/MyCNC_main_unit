#ifndef CNC_SEQUENCER_INTERNALS_H
#define CNC_SEQUENCER_INTERNALS_H


#ifdef __cplusplus
    extern "C" {
#endif

    #include "cnc_defs.h"
    #include "cnc_sequencer.h"
    #include "command_if.h"


    struct SSequencerSetup
    {
        bool    fe_present;         // if front end is present

    };
    




    struct SCNCSequencerInternals
    {
        struct SSequencerSetup  setup;


    };
        



#ifdef __cplusplus
    }
#endif

#endif // CNC_SEQUENCER_INTERNALS_H
