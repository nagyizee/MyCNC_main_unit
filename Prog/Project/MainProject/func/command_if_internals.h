#ifndef COMMAND_IF_INTERNALS_H
#define COMMAND_IF_INTERNALS_H

#ifdef __cplusplus
    extern "C" {
#endif

    #include "cnc_defs.h"
    #include "command_if.h"


    #define CSTATE_WAIT_HEADER  0           // Waiting header ( 0xAA )
    #define CSTATE_GET_ID       1           // Wait the commandID + payload indicator
    #define CSTATE_GET_PAYLOAD  2           // get data bytes
    #define CSTATE_GET_CKSUM    3           // get the checksum byte
    #define CSTATE_CMD_TO_GET   4           // upper layer should pull the command


    #define COMMREC_TIMEOUT     10          // 1ms timeout in data reception

    struct SCommandInterFaceInternals
    {
        uint32  state;
        uint16  data_ctr;
        uint16  time_out;

        uint16  p_len;              // packet lenght
        uint16  cmd_id;             // decoded command ID 

        uint8   use_cksum;
        uint8   cksum;
        uint8   reject_rest;        // reject the rest of packet
        uint8   is_bulk;            // mark that the command received is a bulk one - special readout is needed

        uint32  bulk_poz;           // pozition in bulk payload buffer. max value is p_len
    };


#ifdef __cplusplus
    }
#endif


#endif // COMMAND_IF_INTERNALS_H
