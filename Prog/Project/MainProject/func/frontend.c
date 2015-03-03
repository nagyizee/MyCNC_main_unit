#include "hw_stuff.h"
#include "events_ui.h"
#include "frontend.h"







/* *************************************************
 *
 *  internal routines
 *
 * *************************************************/







/* *************************************************
 *
 *  main routines
 *
 * *************************************************/

    int front_end_init( void )
    {

        return 0;
    }

    void front_end_poll( struct SEventStruct *evt )
    {

         
    }

    bool front_end_chek_op_busy( void )
    {

        return false;
    }


/* *************************************************
 *
 *  Interface routines
 *
 * *************************************************/


    int front_end_spindle_power( bool enable )
    {

        return 0;
    }


    int front_end_spindle_speed( uint32 speed )
    {

        return 0;
    }


    int front_end_check_spindle_ok( void )
    {

        return 0;
    }


    int front_end_get_rpm( uint32 *rpm )
    {

        return 0;
    }


    int front_end_request_coordinates( void )
    {
        
        return 0;
    }


    struct SStepCoordinates *front_end_get_coordinates( void )
    {

        return NULL;
    }


    bool front_end_coordinate_request_sent( void )
    {

        return false;
    }


    int front_end_coordinate_reset_to_max( void )
    {

        return 0;
    }


    int front_end_request_touch_detection( RMuint32 touch_mask )
    {

        return 0;
    }


    int front_end_get_touched_list( RMuint32 *touch_mask )
    {

        return 0;
    }


    int front_end_dbg_get_event_list( uint8 *evt )
    {

        return 0;
    }


    int front_end_dbg_get_signals( uint8 *sig )
    {

        return 0;
    }

