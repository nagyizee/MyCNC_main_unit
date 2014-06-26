#include <stdio.h>
#include "motion_core.h"
#include "motion_core_internals.h"
#include "events_ui.h"


struct SMotionCoreInternals core;


void motion_init( void )
{
    memset( &core, 0, sizeof(core) );

}

void motion_pwr_ctrl( uint32 axis, uint32 power )
{


}

bool motion_pwr_is_set( uint32 axis )
{

}



void motion_set_crt_coord( struct SStepCoordinates *coord )
{
    core.crt_coord = *coord;
    stepper_set_coord( coord );
}


void motion_get_crt_coord( struct SStepCoordinates *coord )
{
    *coord = core.crt_coord;
}


void motion_set_max_travel( struct SStepCoordinates *coord )
{
    core.max_travel = *coord;
}


void motion_step( uint32 axis, uint32 dir )
{
    while ( stepper_getQ() );

    if ( dir )
    {
        if ( (core.crt_coord.coord[axis] < core.max_travel.coord[axis]) || (core.max_travel.coord[axis] == 0) )
        {
            core.crt_coord.coord[axis]++;
            stepper_insert_clock( (1<<axis), (1<<axis) );
        }
    }
    else
    {
        if ( (core.crt_coord.coord[axis] > 0) || (core.max_travel.coord[axis] == 0) )
        {
            if (core.crt_coord.coord[axis] > 0)           // if non-zero coordinate - just decrement it with one step
                core.crt_coord.coord[axis]--;
            else
            {
                struct SStepCoordinates coord;      // for zero coordinate value - increase the hw. axis to avoid uint wraparround
                stepper_get_coord( &coord );
                coord.coord[axis] = 1;
                stepper_set_coord( &coord );
            }

            stepper_insert_clock( (1<<axis), 0 );
        }
    }
}












