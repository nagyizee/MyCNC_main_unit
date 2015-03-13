#ifndef COMMAND_IF_H
#define COMMAND_IF_H

#ifdef __cplusplus
    extern "C" {
#endif

    #include "cnc_defs.h"

    // Command structure:
    //      [ 0xAA ] : [ data ][ cmdID ] : [ optional block ] : [ cksum ]
    //         8bit      1bit    7bit           n bytes          8 bit
    // 
    //      [ 0xAA ]  - header. - indicating start of packet
    //      [ data ]  - if set - optional block is valid, if not set, optional block is missing
    //      [ cmdID ] - command ID
    //      [ cksum ] - checksum. This is at byte pozition: 2 - if no data, 2 + n if data. Calculated including the header
    //      [ optional block ]:
    //              [ len ][ data(0) ]......[ data(M-1)] 
    //               8 bit
    //              
    //              [ len ]     - length of data field after this byte, noted with M.
    //                            n = M + 1
    //      
    //      Minimum command lenght for no-data variants:        3 bytes
    //                            for variant with payload:     5 bytes
    // 
    // 
    // Response structure:
    //      [ type ] : [ payload ]
    //         8bit       n bytes
    // 
    //      [ type ]:       response type  [ACK]/[NAK]/.... se below
    //      [ payload ]:    present only for [ACK] / [DMP]
    //                      [ pl ][ len ] : [ data(0) ]......[ data(M-1)][cksum]
    //                       1bit  8bit
    //              
    //              [ pl ]  - if payload is presend. It is 0 for inband batch response
    //              [ len ] - payload lenght M if [ pl ] is 1;   free cmd fifo space if [ pl ] is 0
    //              [ data ] - present only if [ pl ] is 1;
    //              [ cksum ] - present only if [ pl ] is 1;    calculated from [ pl ][ len ] till [data(M-1)]
    // 
    //      Minimum response lenght for non [ACK]:              1 bytes
    //                              for [ACK] inband:           2 bytes
    //                              for [ACK] outbands:         4 bytes
    // 
    // 
    // Commands are inband [IB] and outband [OB] - Inbands are stored in fifo, outbands are executed right away.
    // stand-alone [SA] commands can be executed only in stopped or paused state
    // 
    // Response to the master:
    //  [OB]    - only one command can be executed once. Response is:
    //              [ACK] + (data)  - immediately after reception and if execution is started without problems
    //              [NAK]           - faulty communication, timeout, incomplete data, checksum error
    //              [INV]           - erronous or invalid parameters 
    //              [PEN]           - an other outband command is in execution
    // 
    //  [IB]    - inband commands can be sent as bulk with one cksum. Maximum command bulk lenght can not exceed input communication fifo size
    //              [ACK] + [cmd fifo free space]
    //              [NAK]           - faulty communication, timeout, incomplete data, checksum error
    //              [OVF]           - comm. input fifo overflowed, all commands from this batch are discarded
    //              [REJ] + [cmdID] - commands from the command with cmdID are rejected because of command fifo full
    // 
    //  for dumps:  [DMP] + (data)  - transmitted periodically
    // 


    #define RESP_ACK        0xA5        // 1010 0101
    #define RESP_DMP        0xAF        // 1010 1111

    #define RESP_NAK        0x55        // 0101 0101
    #define RESP_INV        0x59        // 0101 1001
    #define RESP_PEN        0x56        // 0101 0110
    #define RESP_OVF        0x53        // 0101 0011
    #define RESP_REJ        0x5C        // 0101 1100

    // command ID definition:
    // setup commands
    #define CMD_OBSA_RESET                  0x00    // resets everything from fresh start state. Setup is needed after it.

    #define CMD_OBSA_SETUP_MAX_TRAVEL       0x01    // set the maximum travels on each axis 
                                                    // (defaults are 130x46x80x360)
    #define CMD_OBSA_SETUP_MAX_SPEEDS       0x02    // maximum allowed speed and fast transition speeds for internally generated sequences
                                                    // (defaults: max allowed: 1500mm/min, rapid: 1000mm/min)
    #define CMD_OBSA_SETUP_HOME_POZ         0x03    // set the home/tool change pozition. Default is 130:46:80.

    #define CMD_OBSA_SETUP_PROBE_POZ        0x04    // set the X:Y position for tooltip probing. If not defined, find_Z_zero command will be rejected


    // internal automation and movements            - stop command will cancel their execution and mark them as failed
    #define CMD_OBSA_FIND_ORIGIN            0x11    // search the maximums of each axis. Origin will be this coordinate - maximum travel

    #define CMD_OBSA_GO_HOME                0x12    // go to start position, same as Home/ToolChange

    #define CMD_OBSA_FIND_Z_ZERO            0x13    // used for tool-tip finding

    #define CMD_OBSA_STEP                   0x14    // step axis


    // operational commands
    #define CMD_OBSA_START                  0x21    // start the command execution or resume last interrupted command. 
                                                    // if started - it does nothing
    #define CMD_OB_PAUSE                    0x22    // pause execution of the current command, stops the spindle. if no commands are running it does nothing
                                                    // - emergency button does the same
    #define CMD_OB_STOP                     0x23    // clears up the running state, empty all the fifos, powers down the spindle

    #define CMD_OB_SCALE_FEED               0x24    // scale feed speed

    #define CMD_OB_SCALE_SPINDLE            0x25    // scale the spindle speed

    
    // get status
    #define CMD_OB_GET_CRT_COORD            0x31    // returns the current coordinates. Has flag to activate auto coordinate send

    #define CMD_OB_GET_CRT_CMD_ID           0x32    // returns the ID of the currently executed inband command, if stopped, the last command's ID

    #define CMD_OB_GET_STATUS               0x33    // returns - [ob] command execution status (none/ok, pending, failed),  (not considering this)
                                                    //         - currently executed (or stopped) cmdID 
                                                    //         - command fifo free space

    // inband commands
    #define CMD_IB_SPINDLE                  0x41    // set spindle speed (turn on/off also)

    #define CMD_IB_WAIT                     0x42    // wait x seconds before executing the next command

    #define CMD_IB_GOTO                     0x43    // go to coordinate. Coordinates and feed speed are given, each param. can be optional, will reuse last value

    #define CMD_IB_ARC                      0x44    // generate arc with helix - same param setup as for goto

    #define CMD_IB_DRILL                    0x45    // drill sequence with vertical depth, feed, chip clean quantum, return speed




#ifdef __cplusplus
    }
#endif


#endif // COMMAND_IF_H
