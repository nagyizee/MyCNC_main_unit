#ifndef CNC_SEQUENCER_H
#define CNC_SEQUENCER_H

#ifdef __cplusplus
    extern "C" {
#endif

    #include "cnc_defs.h"
    #include "events_ui.h"
    #include "motion_core.h"


    // init sequencer
    void sequencer_init();


    // poll the sequencer
    void sequencer_poll( struct SEventStruct *evt );


#ifdef __cplusplus
    }
#endif


#endif // CNC_SEQUENCER_H
