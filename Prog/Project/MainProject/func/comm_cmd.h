#ifndef COMM_CMD_H
#define COMM_CMD_H

#ifdef __cplusplus
    extern "C" {
#endif

    #include "typedefs.h"



    #define COMFLAG_OK          0x00
    #define COMFLAG_OVERFLOW    0x01


    // comm port polling routine
    uint32 comm_cmd_poll(void);

    // flush input fifo
    void comm_cmd_InFlush(void);

    // get input data lenght
    uint32 comm_cmd_GetInLength(void);

    uint32 comm_cmd_GetOutFree(void);

    // returns a character from input fifo, -1 if queue is empty
    uint32 comm_rdChar(void);

    uint32 comm_wrChar( uint8 byte );

#ifdef __cplusplus
    }
#endif



#endif // COMM_CMD_H
