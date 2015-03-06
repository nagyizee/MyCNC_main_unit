#ifndef COMM_FE_H
#define COMM_FE_H

#ifdef __cplusplus
    extern "C" {
#endif


    #include "cnc_defs.h"
    #include "events_ui.h"


    #define COMMFE_OK       0       // if response arrived with ACK
    #define COMMFE_NAK      1       // returned for NAK and also for faulty CkSum
    #define COMMFE_PENDING  2       // response has't arrived yet

    uint32 commfe_init();

    // cmd_len doesn't need to contain the checksum - it is added inside, !!! but fecmd should be of size cmd_len+1 !!!
    uint32 commfe_sendCommand( uint8 *fecmd, uint32 cmd_len );

    // check if response arrived. response should be of resp_len
    // returns: see COMMFE_xxx
    uint32 commfe_checkResponse( uint8 resp_len );

    // gets and flushes the response
    uint32 commfe_getResponse( uint8 *fecmd, uint32 resp_len );

    // flushes the response
    uint32 commfe_flushResponse( void );

    // returns 1 if output fifo is empty ( command is sent )
    uint32 commfe_is_outfifo_empty( void );


#ifdef __cplusplus
    }
#endif


#endif // COMM_FE_H
