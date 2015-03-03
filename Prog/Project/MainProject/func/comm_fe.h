#ifndef COMM_FE_H
#define COMM_FE_H

#ifdef __cplusplus
    extern "C" {
#endif


    #include "cnc_defs.h"
    #include "events_ui.h"

    uint32 commfe_init();

    uint32 commfe_sendCommand( uint8 *fecmd, uint32 cmd_len );
    uint32 commfe_sendCommand_waitResponse( uint8 *fecmd, uint32 cmd_len, uint32 ret_len );



#ifdef __cplusplus
    }
#endif


#endif // COMM_FE_H
