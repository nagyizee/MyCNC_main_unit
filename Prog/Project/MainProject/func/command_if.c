#include "command_if.h"
#include "command_if_internals.h"
#include "comm_cmd.h"


struct SCommandInterFaceInternals comm;
uint8 comm_buffer[ CMD_INPUT_BUFFER_SIZE ];



static void internal_command_flush( void )
{
    comm.state = CSTATE_WAIT_HEADER;
    comm.reject_rest = 0;
    comm.data_ctr = 0;
    comm.time_out = 0;
    comm.is_bulk = 0;
    comm.bulk_poz = 0;
}

static void internal_add_cksum( uint8 byte )
{
    comm.cksum += byte;     // TBD
}

static uint8 internal_add_out_cksum( uint8 *cksum, uint8 byte )
{
    *cksum += byte;     // TBD
}


static void internal_respond_byte( uint8 byte )
{
    comm_wrChar( byte );
}


static int internal_check_command( uint32 cmd_id, uint32 cmd_len, bool check_ib )
{
    switch ( cmd_id )
    {
        case CMD_OBSA_RESET:
        case CMD_OBSA_FIND_ORIGIN:
        case CMD_OBSA_GO_HOME:
        case CMD_OBSA_FIND_Z_ZERO:
        case CMD_OBSA_START:
        case CMD_OB_PAUSE:
        case CMD_OB_STOP:
        case CMD_OB_GET_CRT_CMD_ID:
        case CMD_OB_GET_STATUS:
        case CMD_OB_GET_PROBE_TOUCH:
            if ( check_ib )
                return -1;
            if ( cmd_len == 0 )
                return 0;   // ok
            break;
        case CMD_OB_GET_CRT_COORD:
            if ( check_ib )
                return -1;
            if ( (cmd_len == 0) || (cmd_len == 1) )
                return 0;
            break;
        case CMD_OBSA_SETUP_MAX_TRAVEL:
        case CMD_OBSA_SETUP_HOME_POZ:
            if ( check_ib )
                return -1;
            if ( cmd_len == 10 )
                return 0;   // ok
            break;

        case CMD_OBSA_SETUP_MAX_SPEEDS:
            if ( check_ib )
                return -1;
            if ( cmd_len == 4 )
                return 0;   // ok
            break;

        case CMD_OBSA_SETUP_PROBE_POZ:
            if ( check_ib )
                return -1;
            if ( cmd_len == 5 )
                return 0;   // ok
            break;

        case CMD_OBSA_STEP:
            if ( check_ib )
                return -1;
            if ( cmd_len == 1 )
                return 0;   // ok
            break;

        case CMD_OB_SCALE_FEED:
        case CMD_OB_SCALE_SPINDLE:
            if ( check_ib )
                return -1;
        case CMD_IB_WAIT:
            if ( cmd_len == 2 )
                return 0;   // ok
            break;

        case CMD_IB_BULK:
            if ( check_ib )
                return -1;          // inband bulk is rejected also when in bulk decoding
            if ( cmd_len >= 4 )
                return 0;   // ok
            break;

        case CMD_IB_SPINDLE:
            if ( cmd_len == 3 )
                return 0;   // ok
            break;

        case CMD_IB_GOTO:
            if ( cmd_len >= 3 )
                return 0;   // ok
            break;

        case CMD_IB_DRILL:
            if ( cmd_len == 15 )
                return 0;   // ok
            break;
    }
    return -1;
}


static inline int internal_pc_setup_max_travel( struct ScmdIfCommand *command )
{   
    //     0         1        2     2       3          4          5          6       7    7        8          9
    //[xxxx xxxx][xxxx xxxx][xxxx yyyy][yyyy yyyy][yyyy yyyy][zzzz zzzz][zzzz zzzz][zzzz aaaa][aaaa aaaa][aaaa aaaa] - [cksum]

                                                //   [0000 xxxx][xxxx xxxx][xxxx xxxx]
    command->cmd.max_travel.coord[COORD_X] = (comm_buffer[0] << 12) | (comm_buffer[1] << 4) | (comm_buffer[2] >> 4);
    command->cmd.max_travel.coord[COORD_Y] = ( (comm_buffer[2] & 0x0f) << 16) | (comm_buffer[3]<<8) | (comm_buffer[4]);
    command->cmd.max_travel.coord[COORD_Z] = (comm_buffer[5] << 12) | (comm_buffer[6] << 4) | (comm_buffer[7] >> 4);
    command->cmd.max_travel.coord[COORD_A] = ( (comm_buffer[7] & 0x0f) << 16) | (comm_buffer[8]<<8) | (comm_buffer[9]);
    return 0;
}

static inline int internal_pc_setup_max_speed( struct ScmdIfCommand *command ) 
{   
    //                                     0         1          2          3 
    // IN:      [0xAA][0x82][0x04] - [mmmm mmmm][mmmm mmmm][rrrr rrrr][rrrr rrrr] - [cksum]
    command->cmd.max_speeds.absolute = (comm_buffer[0]<<8) | (comm_buffer[1]);
    command->cmd.max_speeds.rapid = (comm_buffer[2]<<8) | (comm_buffer[3]);
}

static inline int internal_pc_setup_home_poz( struct ScmdIfCommand *command )
{   
    //                          0          1        2    2       3          4          5          6        7    7       8          9
    //[0xAA][0x83][0x0a] - [xxxx xxxx][xxxx xxxx][xxxx yyyy][yyyy yyyy][yyyy yyyy][zzzz zzzz][zzzz zzzz][zzzz aaaa][aaaa aaaa][aaaa aaaa] - [cksum]
    command->cmd.home_poz.coord[COORD_X] = (comm_buffer[0] << 12) | (comm_buffer[1] << 4) | (comm_buffer[2] >> 4);
    command->cmd.home_poz.coord[COORD_Y] = ( (comm_buffer[2] & 0x0f) << 16) | (comm_buffer[3]<<8) | (comm_buffer[4]);
    command->cmd.home_poz.coord[COORD_Z] = (comm_buffer[5] << 12) | (comm_buffer[6] << 4) | (comm_buffer[7] >> 4);
    command->cmd.home_poz.coord[COORD_A] = ( (comm_buffer[7] & 0x0f) << 16) | (comm_buffer[8]<<8) | (comm_buffer[9]);
    return 0;
}

static inline int internal_pc_setup_probe_poz( struct ScmdIfCommand *command )
{   
    //                          0          1        2    2       3          4
    //[0xAA][0x84][0x05] - [xxxx xxxx][xxxx xxxx][xxxx yyyy][yyyy yyyy][yyyy yyyy] - [cksum]
    command->cmd.probe_poz.coord[COORD_X] = (comm_buffer[0] << 12) | (comm_buffer[1] << 4) | (comm_buffer[2] >> 4);
    command->cmd.probe_poz.coord[COORD_Y] = ( (comm_buffer[2] & 0x0f) << 16) | (comm_buffer[3]<<8) | (comm_buffer[4]);
    return 0;
}

static inline int internal_pc_step( struct ScmdIfCommand *command )
{
    //                           0
    // [0xAA][0x94][0x01] - [aaaa dddd] - [cksum]
    command->cmd.step.axis_mask = ( comm_buffer[0] >> 4 ) & 0x0f;
    command->cmd.step.dir_mask = comm_buffer[0] & 0x0f;
    return 0;
}

static int internal_pc_getscale( int32 *value )
{
    // [0xAA][0xA4][0x02][ssss ssss][ssss ssss][cksum]
    int16 *val_ptr;
    val_ptr = (int16)(comm_buffer);
    *value = *val_ptr;
    return 0;
}

static inline int internal_pc_getcrtCoord( struct ScmdIfCommand *command )
{
    if ( comm.p_len )
    {
        command->cmd.get_coord.is_setup = true;
        command->cmd.get_coord.dmp_enable = (comm_buffer[0] & 0x01) ? true : false;
    }
    else
    {
        command->cmd.get_coord.is_setup = false;
        command->cmd.get_coord.dmp_enable = false;
    }
    return 0;
}

static inline int internal_pc_spindle( uint8 *buffer, struct ScmdIfCommand *command )
{   
    //                0        1          2
    // [0xC1][0x03][cmdID][rrrr rrrr][rrrr rrrr]
    command->cmd_inband = 1;
    command->cmdID = comm_buffer[0];
    command->cmd.ib_spindle_speed = (comm_buffer[1]<<8) | (comm_buffer[2]);
    return 0;
}

static inline int internal_pc_wait( uint8 *buffer, struct ScmdIfCommand *command )
{   
    //                0        1
    // [0xC2][0x02][cmdID][hhhh hhhh]
    command->cmd_inband = 1;
    command->cmdID = comm_buffer[0];
    command->cmd.ib_wait = comm_buffer[1];
    return 0;
}

static inline int internal_pc_goto( uint8 *buffer, uint32 plen, struct ScmdIfCommand *command )
{   
    //                0         1         2                                                            n
    // [0xC3][0xnn][cmdID][000f azyx][1111 1111][1111 1111][1111 2222][2222 2222] .... [5555 5555][5555 5555]
    //              f - feed is provided, otherwise use the value from before                
    //              a,z,y,x - coordinates are provided - otherwise use the values from before
    //              1,2,3,4,5 - are the fields for which a bit is set.
    //              - for a,x,y,z - the fields are 2.5 bytes - if reaching the end - the last bits are: 0000
    //              - if f is set - it is always the last field and it is complete: [ffff ffff][ffff ffff] in uint16
    //           maximum format lenght = 16 with all the fields
    int coord_ptr = COORD_X;
    int elem_poz = 0;           // considers 4 bit packets
    uint32 fields;

    fields = comm_buffer[1];

    command->cmd_inband = 1;
    command->cmdID = comm_buffer[0];
    command->cmd.ib_goto.valid_fields = fields;

    for ( coord_ptr = 0; coord_ptr < CNC_MAX_COORDS; coord_ptr++ )
    {
        if ( fields & ( 1 << coord_ptr ) )      // if we have this coordinate defined
        {
            int ptr;
            ptr = (elem_poz >> 1);

            if ( (elem_poz & 0x01) == 0 )       // means that number begins with whole byte. like [xxxx xxxx]
                command->cmd.home_poz.coord[coord_ptr] = (comm_buffer[ptr] << 12) | (comm_buffer[ptr+1] << 4) | (comm_buffer[ptr+2] >> 4);
            else                                // else it begins with half byte. like [.... yyyy][yyyy yyyy]
                command->cmd.home_poz.coord[coord_ptr] = ( (comm_buffer[ptr] & 0x0f) << 16) | (comm_buffer[ptr+1]<<8) | (comm_buffer[ptr+2]);

            elem_poz += 5;
        }
        else
            command->cmd.home_poz.coord[coord_ptr] = -1;    // marking that coordinate is not defined
    }

    if ( elem_poz & 0x01 )      // do not leave half byte - last bits are always 0000 if not used
        elem_poz++;

    if ( fields & 0x10 )        // check for feed speed definition
    {
        int ptr;
        ptr = (elem_poz >> 1);
        command->cmd.ib_goto.feed = (comm_buffer[ptr]<<8) | (comm_buffer[ptr+1]);
        fields += 4;
    }

    if ( (fields >> 1) != plen )    // check if the parsed buffer is valid - payload lenght should be equal with calculation
        return -3;
    return 0;
}

static inline internal_pc_drill( uint8 *buffer, struct ScmdIfCommand *command )
{
    //                0        1          2          3          4          5          6          7          8          9          10         11         12         13         14           
    // [0xC5][0x0f][cmdID][xxxx xxxx][xxxx xxxx][xxxx yyyy][yyyy yyyy][yyyy yyyy][zzzz zzzz][zzzz zzzz][zzzz ZZZZ][ZZZZ ZZZZ][ZZZZ ZZZZ][nnnn nnnn][ffff ffff][ffff cccc][cccc cccc]           
    //          xxx, yyy - horizontal coordinates of the hole
    //          zzz      - top level of the material from where feed is considered
    //          ZZZ      - bottom of the hole
    //          nnn      - chip removal steps ( 255 max )
    //          fff      - drilling feed speed
    //          ccc      - clearence distance from the top of the hole (zzz)

    command->cmd_inband = 1;
    command->cmdID = comm_buffer[0];

    command->cmd.ib_drill.coord.coord[COORD_X] = (comm_buffer[1] << 12) | (comm_buffer[2] << 4) | (comm_buffer[3] >> 4);
    command->cmd.ib_drill.coord.coord[COORD_Y] = ( (comm_buffer[3] & 0x0f) << 16) | (comm_buffer[4]<<8) | (comm_buffer[5]);
    command->cmd.ib_drill.coord.coord[COORD_Z] = (comm_buffer[6] << 12) | (comm_buffer[7] << 4) | (comm_buffer[8] >> 4);
    command->cmd.ib_drill.coord.coord[COORD_A] = ( (comm_buffer[8] & 0x0f) << 16) | (comm_buffer[9]<<8) | (comm_buffer[10]);
    command->cmd.ib_drill.cycles = comm_buffer[11];
    command->cmd.ib_drill.feed = (comm_buffer[12] << 4) | (comm_buffer[13] >> 4);
    command->cmd.ib_drill.clearance = ( (comm_buffer[13] & 0x0f) << 8) | (comm_buffer[14]);
    return 0;
}


static int internal_sr_generic_1_byte( uint8 ack )
{
    if ( comm_cmd_GetOutFree() >= 1 )
    {
        comm_wrChar( ack );
        return 0;
    }
    return -1;
}

static int internal_sr_generic_2_bytes( uint8 ack, uint8 param )
{
    if ( comm_cmd_GetOutFree() >= 2 )
    {
        comm_wrChar( ack );
        comm_wrChar( param );
        return 0;
    }
    return -1;
}

static int internal_sr_ack_cmdID( uint8 cmdID )
{
    // [ACK][0x81][cmdID][cksum]
    uint8 cksum = (uint8)(COMMCKSUMSTART);
    if ( comm_cmd_GetOutFree() < 4 )
        return -1;

    comm_wrChar( RESP_ACK );
    comm_wrChar( 0x81 );
    internal_add_out_cksum( &cksum, 0x81 );
    comm_wrChar( cmdID );
    internal_add_out_cksum( &cksum, cmdID );
    comm_wrChar( cksum );
    return 0;
}

static inline int internal_sr_ack_stop( struct ScmdIfResponse *response )
{
    // [ACK][0x82][cmdIDex][cmdIDq][cksum]
    uint8 cksum = (uint8)(COMMCKSUMSTART);
    if ( comm_cmd_GetOutFree() < 4 )
        return -1;

    comm_wrChar( RESP_ACK );
    comm_wrChar( 0x82 );
    internal_add_out_cksum( &cksum, 0x82 );
    comm_wrChar( (uint8)response->resp.stop.cmdIDex );
    internal_add_out_cksum( &cksum, (uint8)response->resp.stop.cmdIDex );
    comm_wrChar( (uint8)response->resp.stop.cmdIDq );
    internal_add_out_cksum( &cksum, (uint8)response->resp.stop.cmdIDq );
    comm_wrChar( cksum );
    return 0;
}

static int internal_sr_dump_coordinates( uint8 resp_type, struct ScmdIfResponse *response )
{
    // [ACK][0x8C][xxxx xxxx][xxxx xxxx][xxxx yyyy][yyyy yyyy][yyyy yyyy][zzzz zzzz][zzzz zzzz][zzzz aaaa][aaaa aaaa][aaaa aaaa][rrrr rrrr][rrrr rrrr][cksum]
    uint8 cksum = (uint8)(COMMCKSUMSTART);
    uint8 data;
    int i;

    if ( comm_cmd_GetOutFree() < 15 )
        return -1;

    comm_wrChar( resp_type );
    comm_wrChar( 0x8C );
    internal_add_out_cksum( &cksum, 0x8C );

    for (i=0; i<2; i++)
    {
        // X/Z:  [xxxx xxxx][xxxx xxxx][xxxx 0000]    high bit to low bit
        data = ((response->resp.getCoord.coord.coord[i*2+0] >> 12) & 0xff);
        comm_wrChar(data);
        internal_add_out_cksum( &cksum, data );
        data = ((response->resp.getCoord.coord.coord[i*2+0] >> 4)  & 0xff);
        comm_wrChar(data);
        internal_add_out_cksum( &cksum, data );
        // Y/A:  [0000 yyyy][yyyy yyyy][yyyy yyyy]   
        data = ((response->resp.getCoord.coord.coord[i*2+0] << 4)  & 0xf0) | ((response->resp.getCoord.coord.coord[i*2+1] >> 16) & 0x0f);
        comm_wrChar(data);
        internal_add_out_cksum( &cksum, data );
        data = ((response->resp.getCoord.coord.coord[i*2+1] >> 8) & 0xff);
        comm_wrChar(data);
        internal_add_out_cksum( &cksum, data );
        data = ((response->resp.getCoord.coord.coord[i*2+1] >> 0) & 0xff);
        comm_wrChar(data);
        internal_add_out_cksum( &cksum, data );
    }
    data = ((response->resp.getCoord.rpm >> 8) & 0xff);
    comm_wrChar(data);
    internal_add_out_cksum( &cksum, data );
    data = (response->resp.getCoord.rpm & 0xff);
    comm_wrChar(data);
    internal_add_out_cksum( &cksum, data );
    comm_wrChar( cksum );
    return 0;
}

static inline int internal_sr_ack_status( struct ScmdIfResponse *response )
{
    // [ACK][0x84][GSfR rscc][cmdIDex][cmdIDq][freesp][cksum]
    uint8 cksum = (uint8)(COMMCKSUMSTART);
    if ( comm_cmd_GetOutFree() < 7 )
        return -1;

    comm_wrChar( RESP_ACK );
    comm_wrChar( 0x84 );
    internal_add_out_cksum( &cksum, 0x84 );

    comm_wrChar( (uint8)response->resp.status.s_flags.val );
    internal_add_out_cksum( &cksum, (uint8)response->resp.status.s_flags.val );
    comm_wrChar( response->resp.status.cmdIDex );
    internal_add_out_cksum( &cksum, response->resp.status.cmdIDex );
    comm_wrChar( response->resp.status.cmdIDq );
    internal_add_out_cksum( &cksum, response->resp.status.cmdIDq );
    if ( response->resp.status.s_flags.f.General_fault )
    {
        comm_wrChar( response->resp.status.fault_code );
        internal_add_out_cksum( &cksum, response->resp.status.fault_code );
    }
    else
    {
        comm_wrChar( (uint8)response->resp.status.freeSpace );
        internal_add_out_cksum( &cksum, (uint8)response->resp.status.freeSpace );
    }
    comm_wrChar( cksum );
    return 0;
}

static inline int internal_sr_ack_probe_touch( struct ScmdIfResponse *response )
{
    // [ACK][0x88][xxxx xxxx][xxxx xxxx][xxxx yyyy][yyyy yyyy][yyyy yyyy][zzzz zzzz][zzzz zzzz][zzzz 0000][cksum]
    uint8 cksum = (uint8)(COMMCKSUMSTART);
    uint8 data;
    int i;

    if ( comm_cmd_GetOutFree() < 11 )
        return -1;

    comm_wrChar( RESP_ACK );
    comm_wrChar( 0x88 );
    internal_add_out_cksum( &cksum, 0x88 );

    // X:  [xxxx xxxx][xxxx xxxx][xxxx 0000]    high bit to low bit
    data = ((response->resp.touch.coord[0] >> 12) & 0xff);
    comm_wrChar(data);
    internal_add_out_cksum( &cksum, data );
    data = ((response->resp.touch.coord[0] >> 4)  & 0xff);
    comm_wrChar(data);
    internal_add_out_cksum( &cksum, data );
    // Y:  [0000 yyyy][yyyy yyyy][yyyy yyyy]   
    data = ((response->resp.touch.coord[0] << 4)  & 0xf0) | ((response->resp.touch.coord[1] >> 16) & 0x0f);
    comm_wrChar(data);
    internal_add_out_cksum( &cksum, data );
    data = ((response->resp.touch.coord[1] >> 8) & 0xff);
    comm_wrChar(data);
    internal_add_out_cksum( &cksum, data );
    data = ((response->resp.touch.coord[1] >> 0) & 0xff);
    comm_wrChar(data);
    internal_add_out_cksum( &cksum, data );
    // Z:  [zzzz zzzz][zzzz zzzz][zzzz 0000]    high bit to low bit
    data = ((response->resp.touch.coord[2] >> 12) & 0xff);
    comm_wrChar(data);
    internal_add_out_cksum( &cksum, data );
    data = ((response->resp.touch.coord[2] >> 4)  & 0xff);
    comm_wrChar(data);
    internal_add_out_cksum( &cksum, data );
    data = ((response->resp.touch.coord[2] << 4)  & 0xf0);
    comm_wrChar(data);
    internal_add_out_cksum( &cksum, data );

    comm_wrChar( cksum );
    return 0;
}


static int internal_parse_command( uint32 cmd_id, uint8 *buffer, uint32 plen, struct ScmdIfCommand *command )
{
    int res = -3;

    command->cmd_type = cmd_id;
    command->cmdID = 0;
    command->cmd_inband = 0;

    switch ( cmd_id )
    {
        case CMD_OBSA_RESET:            res = 0; break;
        case CMD_OBSA_SETUP_MAX_TRAVEL: res = internal_pc_setup_max_travel( command ); break;
        case CMD_OBSA_SETUP_MAX_SPEEDS: res = internal_pc_setup_max_speed( command ); break;
        case CMD_OBSA_SETUP_HOME_POZ:   res = internal_pc_setup_home_poz( command ); break;
        case CMD_OBSA_SETUP_PROBE_POZ:  res = internal_pc_setup_probe_poz( command ); break;
        case CMD_OBSA_FIND_ORIGIN:      res = 0; break;
        case CMD_OBSA_GO_HOME:          res = 0; break;
        case CMD_OBSA_FIND_Z_ZERO:      res = 0; break;
        case CMD_OBSA_STEP:             res = internal_pc_step( command ); break;
        case CMD_OBSA_START:            res = 0; break;
        case CMD_OB_PAUSE:              res = 0; break;
        case CMD_OB_STOP:               res = 0; break;
        case CMD_OB_SCALE_FEED:         res = internal_pc_getscale( &command->cmd.scale_feed ); break;
        case CMD_OB_SCALE_SPINDLE:      res = internal_pc_getscale( &command->cmd.scale_spindle ); break;
        case CMD_OB_GET_CRT_COORD:      res = internal_pc_getcrtCoord( command ); break;
        case CMD_OB_GET_CRT_CMD_ID:     res = 0; break;  
        case CMD_OB_GET_STATUS:         res = 0; break;
        case CMD_OB_GET_PROBE_TOUCH:    res = 0; break;
        case CMD_IB_BULK:               res = -2; break;
        case CMD_IB_SPINDLE:            res = internal_pc_spindle( buffer, command ); break;
        case CMD_IB_WAIT:               res = internal_pc_wait( buffer, command ); break;
        case CMD_IB_GOTO:               res = internal_pc_goto( buffer, plen, command ); break;
        case CMD_IB_DRILL:              res = internal_pc_drill( buffer, command ); break;
    }

    return res;
}


static int internal_send_response( struct ScmdIfResponse *response )
{
    int res = 0;
    switch ( response->resp_type )
    {
        case RESP_ACK:
            switch ( response->cmd_type )
            {
                case CMD_OB_PAUSE: 
                case CMD_OB_GET_CRT_CMD_ID:           
                    res = internal_sr_ack_cmdID( response->resp.cmdID );
                    break;
                case CMD_OB_STOP:           
                    res = internal_sr_ack_stop( response );
                    break;     
                case CMD_OB_GET_CRT_COORD: 
                    if ( response->resp.getCoord.has_data )
                        res = internal_sr_dump_coordinates( RESP_ACK, response );
                    else
                        res = internal_sr_generic_2_bytes( RESP_ACK, 0 );
                    break;
                case CMD_OB_GET_STATUS: 
                    res = internal_sr_ack_status( response );     
                    break;
                case CMD_OB_GET_PROBE_TOUCH:  
                    res = internal_sr_ack_probe_touch( response );
                    break;
                case CMD_IB_BULK:               
                case CMD_IB_SPINDLE:            
                case CMD_IB_WAIT:               
                case CMD_IB_GOTO:               
                case CMD_IB_DRILL: 
                    res = internal_sr_generic_2_bytes( RESP_ACK, (uint8)response->resp.inband.Qfree ); 
                    break;
                default:
                    res = internal_sr_generic_2_bytes( RESP_ACK, 0 ); 
                    break;
            }
            break;
        case RESP_DMP:
            res = internal_sr_dump_coordinates( RESP_DMP, response );
            break;
        case RESP_INV:
            switch ( response->cmd_type )
            {
                case CMD_IB_BULK:               
                case CMD_IB_SPINDLE:            
                case CMD_IB_WAIT:               
                case CMD_IB_GOTO:               
                case CMD_IB_DRILL: 
                    res = internal_sr_generic_2_bytes( RESP_INV, (uint8)response->resp.cmdID ); 
                    break;
                default:
                    res = internal_sr_generic_1_byte( RESP_INV );
                    break;
            }
            break;
        case RESP_PEN:
            res = internal_sr_generic_1_byte( RESP_PEN );
            break;
        case RESP_REJ:
            switch ( response->cmd_type )
            {
                case CMD_IB_BULK:               
                case CMD_IB_SPINDLE:            
                case CMD_IB_WAIT:               
                case CMD_IB_GOTO:               
                case CMD_IB_DRILL: 
                    res = internal_sr_generic_2_bytes( RESP_REJ, (uint8)response->resp.cmdID ); 
                    break;
                default:
                    res = -3;
                    break;
            }
            break;
    }
}




/* *************************************************
 *
 *  Interface routines
 *
 * *************************************************/

void cmdif_init(void)
{
    memset( &comm, 0, sizeof(comm) );
}


void cmdif_poll( struct SEventStruct *evt )
{
    uint32 in_size;

    do
    {
        if ( comm_cmd_poll() == COMFLAG_OVERFLOW )
        {
            comm_cmd_InFlush();
            internal_command_flush();
            evt->comm_input_overflow = 1;
        }

        if ( comm.state == CSTATE_CMD_TO_GET )
        {
            // if command is prepared and not pulled by application
            evt->comm_command_ready = 1;
            return;
        }

        in_size = comm_cmd_GetInLength();
        if ( in_size )
        {
            switch ( comm.state )
            {
                case CSTATE_WAIT_HEADER:
                    {
                        uint32 byte;
                        byte = comm_rdChar();
                        comm.time_out   = 0;       // no timeout in use when waiting

                        if ( byte == 0xAA )        // check preamb 1010 1010
                        {
                            comm.p_len      = 0;
                            comm.data_ctr   = 0;
                            comm.is_bulk    = 0;

                            comm.state      = CSTATE_GET_ID;
                            comm.cksum      = (uint8)(COMMCKSUMSTART);
                            internal_add_cksum( byte );
                            comm.time_out   = COMMREC_TIMEOUT;
                        }
                    }
                    break;
                case CSTATE_GET_ID:
                    {
                        uint32 byte;
                        byte = comm_rdChar();
                        internal_add_cksum( byte );
                        comm.time_out   = COMMREC_TIMEOUT;
                        comm.cmd_id     = byte & 0x7f;

                        if ( byte & 0x80 )
                        {
                            comm.p_len = (uint16)(-1);
                            comm.state = CSTATE_GET_PAYLOAD;
                        }
                        else
                        {
                            comm.state = CSTATE_GET_CKSUM;
                        }
                    }
                    break;
                case CSTATE_GET_PAYLOAD:
                    {
                        if ( comm.p_len == (uint16)(-1) )       // Get the payload lenght
                        {
                            uint32 byte;
                            byte = comm_rdChar();
                            internal_add_cksum( byte );
                            comm.p_len = byte;
                            comm.time_out   = COMMREC_TIMEOUT;
                        }
                        else                                    // Get the payload
                        {
                            uint32 bytes_to_get = comm.p_len - comm.data_ctr;
                            if ( bytes_to_get > in_size)
                                bytes_to_get = in_size;

                            while ( bytes_to_get )  // get the arrived data bytes
                            {
                                uint32 byte;
                                bytes_to_get--;
                                byte = comm_rdChar();

                                if ( comm.data_ctr >= CMD_INPUT_BUFFER_SIZE )
                                    comm.reject_rest = 1;
                                else
                                {
                                    comm_buffer[comm.data_ctr++] = byte;
                                    internal_add_cksum( byte );
                                }
                            }

                            comm.time_out   = COMMREC_TIMEOUT;
                            if ( comm.data_ctr == comm.p_len )                // reached end of packet
                            {
                                comm.state      = CSTATE_GET_CKSUM;
                            }
                        }
                    }
                    break;
                case CSTATE_GET_CKSUM:
                    {
                        uint32 byte;
                        byte = comm_rdChar();

                        comm.state      = CSTATE_WAIT_HEADER;
                        comm.time_out   = 0;                // no timeout in use when waiting

                        if ( comm.reject_rest )
                        {
                            // command didn't fit in input fifo - notify overflow
                            comm.reject_rest = 0;
                            internal_respond_byte( RESP_OVF );
                            return;
                        }

                        if ( (comm.use_cksum) && (comm.cksum != byte) )
                        {
                            // checksum error:
                            internal_respond_byte( RESP_NAK );
                            return;
                        }
                        else
                        {
                            if ( internal_check_command(comm.cmd_id, comm.p_len, false) )
                            {
                                // if command is invalid
                                internal_respond_byte( RESP_NAK );
                                return;
                            }
                            else
                            {
                                // command was ok, wait for application to get it
                                if ( comm.cmd_id == CMD_IB_BULK )
                                {
                                    comm.is_bulk = 1;
                                    comm.bulk_poz = 0;
                                }

                                comm.state      = CSTATE_CMD_TO_GET;
                                evt->comm_command_ready = 1;
                            }
                            return;
                        }
                    }
                    break;
             }
        }

        if ( in_size == 0 )
        {
            // if no data is received in this iteration then check for timeout and exit
            if ( evt->timer_tick_100us && comm.time_out )
            {
                comm.time_out --;
                if ( comm.time_out == 0 )       // timeout reached
                {
                    comm.state          = CSTATE_WAIT_HEADER;
                    comm.reject_rest    = 0;
                    evt->comm_timeout   = 1;
                    return;
                }
            }
            return;
        }
    } while (1);
}


int cmdif_get_command( struct ScmdIfCommand *command )
{
    int res = 0;
    if ( comm.state != CSTATE_CMD_TO_GET )
        return -1;
    if ( comm.is_bulk )
        return -2;

    // comm buffer - holds the payload only
    res = internal_parse_command( comm.cmd_id, comm_buffer, comm.p_len, command );
    internal_command_flush();           // release the command buffer regardless of result
    return res;
}


int cmdif_confirm_reception( struct ScmdIfResponse *response )
{
    int res;

    // send the response
    res = internal_send_response( response );

    // if sending is successfull, and bulk buffer is still not processed fully - flush the buffer and state
    if ( (res != -1) && ( comm.state == CSTATE_CMD_TO_GET ) )
        internal_command_flush();     

    return res;
}


int cmdif_get_bulk( struct ScmdIfCommand *command )
{
    int res = 0;
    if ( comm.state != CSTATE_CMD_TO_GET )
        return -1;
    if ( comm.is_bulk == false )
        return -2;

    if ( comm.bulk_poz < comm.p_len )
    {
        uint32 cmd_type;
        uint32 cmd_len;

        cmd_type = comm_buffer[ comm.bulk_poz ] & 0x7f;
        cmd_len = comm_buffer[ comm.bulk_poz + 1 ];
        if ( internal_check_command( cmd_type, cmd_len, true ) )
            return -3;
        if ( (cmd_len + comm.bulk_poz) > comm.p_len  )
            return -3;

        res = internal_parse_command( cmd_type, comm_buffer + comm.bulk_poz + 2, cmd_len, command );
        comm.bulk_poz += 2 + cmd_len;
        return res;
    }
    else
    {
        // no more commands in the bulk
        internal_command_flush();
    }
    return -1;
}


