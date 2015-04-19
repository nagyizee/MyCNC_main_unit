#ifndef COMMAND_IF_H
#define COMMAND_IF_H

#ifdef __cplusplus
    extern "C" {
#endif

    #include "cnc_defs.h"
    #include "events_ui.h"

    // Command structure:
    //      [ 0xAA ] : [ data ][ cmdType ] : [ optional block ] : [ cksum ]
    //         8bit      1bit      7bit           n bytes          8 bit
    // 
    //      [ 0xAA ]  - header. - indicating start of packet
    //      [ data ]  - if set - optional block is valid, if not set, optional block is missing
    //      [ cmdType ] - command type
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
    //      [ payload ]:    present only for [ACK] / [DMP] / [REJ] / [INV]
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
    //              [NAK]           - (internal) faulty communication, timeout, incomplete data, checksum error
    //              [INV]           - erronous or invalid parameters 
    //              [PEN]           - an other outband command is in execution or inbands are in run in case of [SA] type outband
    // 
    //  [IB]    - inband commands can be sent as bulk with one cksum. Maximum command bulk lenght can not exceed input communication fifo size
    //              [ACK] + [cmd fifo free space]
    //              [NAK]           - faulty communication, timeout, incomplete data, checksum error
    //              [OVF]           - (internal) comm. input fifo overflowed, all commands from this batch are discarded
    //              [REJ] + [cmdID] - commands after the command with cmdID are rejected because of command fifo full
    //              [INV] + [cmdID] - commands after the command with cmdID are rejected because of command with cmdID has invalid parameters
    //                                  cmdID - means the last successfull command
    // 
    //  for dumps:  [DMP] + (data)  - transmitted periodically
    // 
    //  after reset or fresh start: [RST][RST][RST][RST]  -> this notifies the master about the start-up or reset
    // 

    #define CMD_INPUT_BUFFER_SIZE       256

    #define RESP_ACK        0xA5        // 1010 0101
    #define RESP_DMP        0xAF        // 1010 1111
    #define RESP_RST        0xA9        // 1010 1001

    #define RESP_NAK        0x55        // 0101 0101        - NAK is sent internally
    #define RESP_INV        0x59        // 0101 1001
    #define RESP_PEN        0x56        // 0101 0110
    #define RESP_OVF        0x53        // 0101 0011        - OVF is sent internally
    #define RESP_REJ        0x5C        // 0101 1100


    #define GENFAULT_TABLE_STUCK    0x01        // retried n times recover missing steps - no success
    #define GENFAULT_SPINDLE_STUCK  0x02        // retried n times recover spindle speed - no success
    #define GENFAULT_FRONT_END      0x03        // broken link with the front-end ( repeated timeout or data transfer failures )
                                                

    // command type definition:
    // setup commands
    #define CMD_OB_RESET                    0x55    // resets everything from fresh start state. Setup is needed after it. Overrides other standalone commands
                                                    // IN:      [0xAA][0x00][cksum]
                                                    // OUT:     [ACK][0x00] - if accepted and reset is started
                                                    //          [PEN] - if another is in execution
                                                    //          [INV] - if sequencer is running

    #define CMD_OBSA_SETUP_MAX_TRAVEL       0x01    // set the maximum travels on each axis 
                                                    // sets the current cordinates also - recommended to call it when machine is at the absolute max. pozition 
                                                    // (defaults are 130x46x80x360)
                                                    // IN:      [0xAA][0x81][0x0a][xxxx xxxx][xxxx xxxx][xxxx yyyy][yyyy yyyy][yyyy yyyy][zzzz zzzz][zzzz zzzz][zzzz aaaa][aaaa aaaa][aaaa aaaa][cksum]
                                                    // OUT:     [ACK][0x00] - if accepted
                                                    //          [PEN] - if another is in execution
                                                    //          [INV] - if sequencer is running / invalid data received

    #define CMD_OBSA_SETUP_MAX_SPEEDS       0x02    // maximum allowed speed and fast transition speeds for internally generated sequences
                                                    // (defaults: max allowed: 1500mm/min, rapid: 1000mm/min)
                                                    // IN:      [0xAA][0x82][0x04][mmmm mmmm][mmmm mmmm][rrrr rrrr][rrrr rrrr][cksum]
                                                    // OUT:     [ACK][0x00] - if accepted
                                                    //          [PEN] - if another is in execution
                                                    //          [INV] - if sequencer is running / invalid data received

    #define CMD_OBSA_SETUP_HOME_POZ         0x03    // set the home/tool change pozition. Default is what is set for max travel.
                                                    // IN:      [0xAA][0x83][0x0a][xxxx xxxx][xxxx xxxx][xxxx yyyy][yyyy yyyy][yyyy yyyy][zzzz zzzz][zzzz zzzz][zzzz aaaa][aaaa aaaa][aaaa aaaa][cksum]
                                                    // OUT:     [ACK][0x00] - if accepted
                                                    //          [PEN] - if another is in execution
                                                    //          [INV] - if sequencer is running / invalid data received

    #define CMD_OBSA_SETUP_PROBE_POZ        0x04    // set the X:Y position for tooltip probing. If not defined, find_Z_zero command will be rejected
                                                    // IN:      [0xAA][0x84][0x05][xxxx xxxx][xxxx xxxx][xxxx yyyy][yyyy yyyy][yyyy yyyy][cksum]
                                                    // OUT:     [ACK][0x00] - if accepted
                                                    //          [PEN] - if another is in execution
                                                    //          [INV] - if sequencer is running / invalid data received


    // internal automation and movements            - stop or pause command will cancel their execution and mark them as failed
    #define CMD_OBSA_FIND_ORIGIN            0x11    // search the maximums of each axis. Origin will be this coordinate - maximum travel
                                                    // - in case of front-end it will search the end points automatically. Master should assure that 
                                                    // milling head is cleared of any obstacle. Use the freerun commands.
                                                    // - if no front-end is present - this command will set up the current coordinates to max travel and returns 
                                                    // synchronously
                                                    // IN:      [0xAA][0x11][cksum]
                                                    // OUT:     [ACK][0x00] - if accepted / operation started
                                                    //          [PEN] - if another operation is in execution
                                                    //          [INV] - if sequencer is running / invalid data received

    #define CMD_OBSA_GO_HOME                0x12    // go to start position, same as Home/ToolChange
                                                    // IN:      [0xAA][0x12][cksum]
                                                    // OUT:     [ACK][0x00] - if accepted / operation started
                                                    //          [PEN] - if another operation is in execution
                                                    //          [INV] - if sequencer is running / invalid data received

    #define CMD_OBSA_FIND_Z_ZERO            0x13    // used for tool-tip finding. it will touch the Z probe, after this it will execute a go home procedure
                                                    // Note: be aware to clear any obstacle on XY plane at current heigth before executing this command
                                                    // IN:      [0xAA][0x13][cksum]
                                                    // OUT:     [ACK][0x00] - if accepted / operation started
                                                    //          [PEN] - if another operation is in execution
                                                    //          [INV] - if sequencer is running / invalid data received

    #define CMD_OBSA_STEP                   0x14    // step axis
                                                    // Stop is recommended after stepping finished since it doesn't detect missing steps
                                                    // IN:      [0xAA][0x94][0x01][aaaa dddd][cksum]
                                                    //          aaaa - step bits for a/z/y/x,  dddd - direction bits for a/z/y/x
                                                    // OUT:     [ACK][0x00] - if accepted / operation started
                                                    //          [PEN] - if another operation is in execution
                                                    //          [INV] - if sequencer is running / can not step
    
    #define CMD_OBSA_FREERUN                0x15    // freerun command for the given axis. Only one axis is permitted at the same time.
                                                    // command is cancelled by stop command or by 10ms timeout. To run it continuously it should
                                                    // be retransmitted in the timeout interval
                                                    // IN:      [0xAA][0x94][0x02][aaLd ffff][ffff ffff][cksum]
                                                    //          aa   - axis to be controlled
                                                    //          L    - if 1 then max travel is not used (proceed with care)
                                                    //          d    - direction: 1 - positive, 0 - negative
                                                    //          fff  - feed in mm/min
                                                    // OUT:     [ACK][0x00] - if accepted / operation started
                                                    //          [PEN] - if another operation is in execution
                                                    //          [INV] - if sequencer is running / can not step


    // operational commands
    #define CMD_OBSA_START                  0x21    // start the command execution or resume last interrupted inband set. 
                                                    // if started - it does nothing
                                                    // IN:      [0xAA][0x21][cksum]
                                                    // OUT:     [ACK][0x00] - executed
                                                    //          [PEN] - if another operation is in execution

    #define CMD_OB_PAUSE                    0x22    // pause execution of inband set, cancels the current command, stops the spindle. if no commands are running it does nothing
                                                    // - emergency button does the same
                                                    // - pause will make other outband commands pending till the spindle stop and coordinate update from frontend
                                                    //   will not be finished
                                                    // IN:      [0xAA][0x22][cksum]
                                                    // OUT:     [ACK][0x81][cmdID][cksum] - started executing, cmdID of the command currenty interrupted

    #define CMD_OB_STOP                     0x23    // clears up the running state, empty all the fifos, stops the spindle, updates the coordintes from front-end
                                                    // - stop is synchronous, ACK will be transmitted at finishing of the operation
                                                    //   use adequated timeout period.
                                                    // IN:      [0xAA][0x23][cksum]
                                                    // OUT:     [ACK][0x82][cmdIDex][cmdIDq][cksum] - stopped executing, 
                                                    //                                              cmdIDex - command ID of command currenty interrupted
                                                    //                                              cmdIDq  - the last command ID introduced in the fifo

    #define CMD_OB_SCALE_FEED               0x24    // scale feed speed
                                                    // IN:      [0xAA][0xA4][0x02][ssss ssss][ssss ssss][cksum]
                                                    //          ss...sss is in int16 format
                                                    // OUT:     [ACK][0x00] - if accepted / operation started
                                                    //          [INV] - invalid data

    #define CMD_OB_SCALE_SPINDLE            0x25    // scale the spindle speed
                                                    // IN:      [0xAA][0xA5][0x02][ssss ssss][ssss ssss][cksum]
                                                    //          ss...sss is in int16 format
                                                    // OUT:     [ACK][0x00] - if accepted / operation started
                                                    //          [INV] - invalid data
    
    // get status
    #define CMD_OB_GET_CRT_COORD            0x31    // returns the current coordinates. Has flag to activate auto coordinate send
                                                    // polling:
                                                    // IN:      [0xAA][0x31][cksum]
                                                    // OUT:     [ACK][0x8C][xxxx xxxx][xxxx xxxx][xxxx yyyy][yyyy yyyy][yyyy yyyy][zzzz zzzz][zzzz zzzz][zzzz aaaa][aaaa aaaa][aaaa aaaa][rrrr rrrr][rrrr rrrr][cksum]
                                                    // setting up dump:
                                                    // IN:      [0xAA][0xB1][0x01][0000 000s][cksum]        - s=1 dump is active
                                                    // OUT:     [ACK][0x00]

    #define CMD_OB_GET_CRT_CMD_ID           0x32    // returns the ID of the currently executed inband command, if stopped, the last command's ID
                                                    // IN:      [0xAA][0x32][cksum]
                                                    // OUT:     [ACK][0x81][cmdID][cksum]

    #define CMD_OB_GET_STATUS               0x33    // returns - [ob] command execution status (none/ok, pending, failed),  (not considering this)
                                                    //         - currently executed (or stopped) cmdID 
                                                    //         - command fifo free space
                                                    // IN:      [0xAA][0x33][cksum]
                                                    // OUT:     [ACK][0x84][GiSf Rscc][cmdIDex][cmdIDq][freesp][cksum]
                                                    //              cc -  00 - no outband operation in execution, or last one succeeded
                                                    //                    01 - last outband operation failed
                                                    //                    11 - outband operation in progress
                                                    //              s  - starvation detected
                                                    //              R  - sequencer is running
                                                    //              f  - coordinate fault (missed steps) detected but can be fixed (hopefully - otherwise general failure)
                                                    //              S  - spindle stuck detected but fixed
                                                    //              i  - initial status - marking a fresh reset
                                                    //              G  - general failure - must read error code  --- If this is set [freesp] holds the error code
                                                    //                  -- when this happens, sequencer stops and flushes everything
                                                    //                  -- main causes: unrecoverable spindle stuck / missing front-end link / unrecoverable missed step

    #define CMD_OB_GET_PROBE_TOUCH          0x34    // returns the touch point of the probe.
                                                    // IN:      [0xAA][0x34][cksum]
                                                    // OUT:     [ACK][0x88][xxxx xxxx][xxxx xxxx][xxxx yyyy][yyyy yyyy][yyyy yyyy][zzzz zzzz][zzzz zzzz][zzzz 0000][cksum]
                                                    //          [INV]    - if no origin was measured yet - call CMD_OBSA_FIND_Z_ZERO command
            
    // inband commands
    // single format:
    //      IN:     [0xAA] - [0xCn][cmdlen][cmdID][cmd params] - [cksum]
    // bulk format:
    //      IN:     [0xAA] - [0xC0][cmdTotalLen] - [0xCn][cmdlen][cmdID][cmd params]-[0xCn][cmdlen][cmdID][cmd params] ... [0xCn][cmdlen][cmdID][cmd params] - [cksum]
    //      where [cmdTotalLen] is the size in bytes of data till checksum (not considering it)

    #define CMD_IB_BULK                     0x40    // see abowe

    #define CMD_IB_SPINDLE                  0x41    // set spindle speed (turn on/off also)
                                                    // format:  [0xC1][0x03][cmdID][rrrr rrrr][rrrr rrrr]
                                                    //                                  rrr - rpm in uint16
                                                    // OUT:     [ACK][0fff ffff]        - free space in the command fifo after this command
                                                    //          [REJ / INV][cmdID]

    #define CMD_IB_WAIT                     0x42    // wait x seconds before executing the next command
                                                    // format:  [0xC2][0x02][cmdID][hhhh hhhh]
                                                    //                                  hhh - time in seconds
                                                    // OUT:     [ACK][0fff ffff]        - free space in the command fifo after this command
                                                    //          [REJ / INV][cmdID]

    #define CMD_IB_GOTO                     0x43    // go to coordinate. Coordinates and feed speed are given, each param. can be optional, will reuse last value
                                                    // format:  [0xC3][0xnn][cmdID][000f azyx][1111 1111][1111 1111][1111 2222][2222 2222] .... [5555 5555][5555 5555]
                                                    //              f - feed is provided, otherwise use the value from before                
                                                    //              a,z,y,x - coordinates are provided - otherwise use the values from before
                                                    //              1,2,3,4,5 - are the fields for which a bit is set.
                                                    //              - for a,x,y,z - the fields are 2.5 bytes - if reaching the end - the last bits are: 0000
                                                    //              - if f is set - it is always the last field and it is complete: [ffff ffff][ffff ffff] in uint16
                                                    //           maximum format lenght = 16 with all the fields
                                                    // 
                                                    // OUT:     [ACK][0fff ffff]        - free space in the command fifo after this command
                                                    //          [REJ / INV][cmdID]

    #define CMD_IB_ARC                      0x44    // generate arc with helix - same param setup as for goto
                                                    // TBD

    #define CMD_IB_DRILL                    0x45    // drill sequence with vertical depth, feed, chip clean quantum, return speed
                                                    // format:  [0xC5][0x0f][cmdID][xxxx xxxx][xxxx xxxx][xxxx yyyy][yyyy yyyy][yyyy yyyy][zzzz zzzz][zzzz zzzz][zzzz ZZZZ][ZZZZ ZZZZ][ZZZZ ZZZZ][nnnn nnnn][ffff ffff][ffff cccc][cccc cccc]           
                                                    //          xxx, yyy - horizontal coordinates of the hole
                                                    //          zzz      - top level of the material from where feed is considered
                                                    //          ZZZ      - depth of hole (relative to top zzz)
                                                    //          nnn      - chip removal steps ( 255 max )
                                                    //          fff      - drilling feed speed
                                                    //          ccc      - clearence distance from the top of the hole (zzz)
                                                    // 
                                                    // OUT:     [ACK][0fff ffff]        - free space in the command fifo after this command
                                                    //          [REJ / INV][cmdID]


    struct SCmdType_maxSpeeds
    {
        uint32  rapid;              // max speed for rapids inside on automated operations
        uint32  absolute;           // absolute maximum speed over the system
    };

    struct SCmdType_step
    {
        uint32  axis_mask;
        uint32  dir_mask;
    };

    struct SCmdType_freerun
    {
        uint8  axis;
        uint8  dir;
        uint16 feed;
        bool   no_limit;
    };

    struct SCmdType_GetCoord
    {
        bool    is_setup;           // command is a setup, not an immediate poll
        bool    dmp_enable;         // enable/disable dump (valid only if is_setup = true)
    };

    struct SCmdType_goto        // size: 4x4 + 2 + 2 = 20
    {
        struct SStepCoordinates coord;
        uint16  feed;
        uint16  valid_fields;       // 000f azyx  format
    };

    struct SCmdType_drill       // size: 4x4 + 4 + 2 + 2 = 24
    {
        struct SStepCoordinates coord;      // where A axis holds the hole bottom
        uint32  clearance;
        uint16  cycles;
        uint16  feed;
    };

    struct SCmdResp_inband
    {
        uint32  cmdID;          // command ID
        uint32  Qfree;          // free space in the inband queue
    };

    struct SCmdResp_stop
    {
        uint32  cmdIDex;        // command ID in execution which was interrupted
        uint32  cmdIDq;         // last command ID from inband queue which was flushed
    };

    struct SCmdResp_getCoord
    {
        uint16  has_data;       // structure has valid data
        uint16  rpm;            // measured rpm
        struct  SStepCoordinates coord;
    };

    struct SCmdResp_status
    {
        union
        {
            struct
            {
                uint8 ob_op_progress:2;             // 00 - no outband operation in execution, or last one succeeded
                                                    // 01 - last outband operation failed
                                                    // 11 - outband operation in progress
                uint8 starvation:1;                 // inband fifo starved
                uint8 run_seq:1;                    // sequencer is running inbands
                uint8 step_fault:1;                 // missed step detected
                uint8 spindle_fault:1;              // spindle stuck detected
                uint8 initial:1;                    // initial status - marking a fresh reset
                uint8 General_fault:1;              // general fault - system stopped automatically, fifos flushed - check cmdIDex and cmdIDq
            } f;
            uint8 val;
        } s_flags;

        uint8   fault_code;                     // general fault code ( see GENFAULT_XXX )
        uint8   cmdIDex;                        // command ID currently in execution
        uint8   cmdIDq;                         // last command ID in queue

        uint32  freeSpace;                      // free space in the queue
    };

    struct ScmdIfCommand        // size: 4 + 24 = 28 bytes
    {
        uint8   cmd_type;       // see definitions abowe
        uint8   cmdID;          // command ID for inband commands
        uint8   cmd_inband;     // command is an inband
        uint8   reserved;
        union
        {
            struct SStepCoordinates     max_travel;
            struct SCmdType_maxSpeeds   max_speeds;
            struct SStepCoordinates     home_poz;
            struct SStepCoordinates     probe_poz;
            struct SCmdType_step        step;
            struct SCmdType_freerun     frun;
            int32                       scale_feed;
            int32                       scale_spindle;
            struct SCmdType_GetCoord    get_coord;
            uint32                      coord_dump;         // 0 - coordinate dump off, 1 - coordinate dump on, 2 - poll coordinate
            uint32                      ib_spindle_speed;
            uint32                      ib_wait;
            struct SCmdType_goto        ib_goto;
            struct SCmdType_drill       ib_drill;
        }       cmd;
    };

    struct ScmdIfResponse
    {
        uint32  cmd_type;       // see definitions abowe
        uint32  resp_type;      // see RESP_XXX
        union
        {
            uint32                      cmdID;      // used for pause or get current cmdID for ACK or for INV / REJ in case of inbands
            struct SCmdResp_stop        stop;
            struct SCmdResp_getCoord    getCoord;
            struct SCmdResp_status      status;
            struct SStepCoordinates     touch;
            struct SCmdResp_inband      inband;     // response to inbands
        }       resp;           // valid for RESP_AKC only
    };

    // called for init command interface
    void cmdif_init(void);

    // command interface main loop. It alters the event structure - notifying about inband or outband command availability
    void cmdif_poll( struct SEventStruct *evt );


    // get the arrived command
    // returns 0 on success, -1 if no command, -2 if command is a bulk one - use cmdif_get_bulk() in that case, -3 if command is invalid
    int cmdif_get_command( struct ScmdIfCommand *command );

    // send response for a command. 
    // returns 0 on success, -1 if send fifo is full
    // as a rule - cmdif_get_inband() is called till fails - assuring reception of a whole batch - this routine will trow away
    // any excess data from the batch
    int cmdif_confirm_reception( struct ScmdIfResponse *response );

    // Get inbands from bulk, since inbands can be transmitted in bulk, call this routine till it returns -1
    // returns 0 on success, -1 if no more inbands in bulk
    int cmdif_get_bulk( struct ScmdIfCommand *command );
    

#ifdef __cplusplus
    }
#endif


#endif // COMMAND_IF_H
