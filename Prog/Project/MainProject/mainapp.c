/*********
 *      Main application loop
 *
 *
 *      CNC command pipeline:
 *
 *             [ CommandIf ]        - this is the top command input fifo - commands are received from the host system (controller) and stored here
 *                   |                This interface does the negociation with the host system, comm error checking, handshacking, etc.
 *                   |                Commands are inband and outband - Inbands are stored in fifo, outbands are executed right away.
 *                   |                stand-alone commands can be executed only in stopped or paused state
 *            ( command fifo )          command list:
 *                   |                      - reset         <stand-alone><outband>: resets everything. setup is needed after it.
 *                   |                      - setup         <stand-alone><outband>: set the maximum travels on each axis, maximum allowed speed, fast transition speeds for internally generated sequences
 *                   |                                                              power levels for various movement speeds, etc.
 *                   |                                                              it also sets up the probe points, etc.
 *                   |                                                              May be splitted in more setup commands - TBD.
 *                   |                      - find_origin   <stand-alone><outband>: search the endpoints of each axis
 *                   |                      - go_home       <stand-alone><outband>: go to start position, same as Home/ToolChange
 *                   |                      - find_Z_zero   <stand-alone><outband>: find the tool tip 0 position
 *                   |                      - pause         <outband>:              pauses the current run state - same as Emergency/Pause first press
 *                   |                      - stop          <outband>:              clear up the running state, empty all the fifos (like Emergency/Pause second press)
 *                   |                      - resume        <stand-alone><outband>: will resume paused operation
 *                   |                      - step          <stand-alone><outband>: will execute a step on the given axis. (direction is given also)
 *                   |                      - scalefeed     <outband>:              Scale the feed speed up/down. valid only in running state
 *                   |                      - scalespindle  <outband>:              Scale the spindle speed up/down. valid only in running state
 *                   |                      - spindle       <inband>:               Spindle speed, units TBD
 *                   |                      - goto          <inband>:               go to coordinate. Coordinates and feed speed are given, each param. can be optional, will reuse last value
 *                   |                      - arc           <inband>:               generate arc with helix - same param setup as for goto
 *                   |                      - drill         <inband>:               drill sequence with vertical depth, feed, chip clean quantum, return speed
 *                   |                      - wait          <inband>:               will wait a time in x10 ms till the next command
 *                   |                      - get_coord     <outband>:              starts and stops the coordinate dumping
 *                   |                      - get_crt_cmd   <outband>:              get the ID of the currently executed inband command, if stopped, the last command's ID
 *                   |
 *                   V
 *            [ Seqence gen.]       - generates movement sequences from the commands.
 *                   |                  commands like linear transition are simple sequences with one movement element
 *                   |                           circles, helix - are broken down to sequences of lines
 *                   |                           go-home, or tool change - are auto generated sequences to bring the tool in start-up position
 *                   |                           resume - will do the the inverse, will go to the memorized position and will generate sequences for the remaining op. from the current command
 *                   |                           find_origin - will generate linear sequences on individual coordinates to locate the start points
 *                   |                           find_Z_zero - will generate linear sequences to go to sensing point and search on Z for tool contact - this will work only from home position
 *                   |                  Front panel buttons can create internal commands for the Sequence generator:
 *                   |                      Emergency/Pause - pressed first in run-time:  - will stop the current execution, memorize the progress in the command
 *                   |                                                                      read back the true coordinates from the encoders, stop the spindle
 *                   |                                      - pressed again - will signal application about program interruption and clears all the fifos (TBD)
 *                   |                      Resume - if pressed after Emergency/Pause: - it will start up the spindle, wait for speed, resume the remainings from the current command.
 *                   |                                                                   If readback coordinates doesn't differ then no new sequences are generated, but will accelerate from 0 if needed
 *                   |                                                                   If readback coordinates differ - it will clear the sequence queue and will regenerate the sequence.
 *                   |                             - if pressed after Home/ToolChange: - it will start up the spindle, wait for speed, generate sequences to reach the working point
 *                   |                                                                   After working point reached - it will regenerate the remaining sequence.
 *                   |                      Home/ToolChange - It has effect only in paused mode - it will clear the sequence, and will generate new ones for bring the tool in start position
 *                   |
 *                   |                  This layer contains the run-time coordinate checking for blocked coordinates, or spindle stall checking,
 *                   |                      also dynamic speed adjustment in case of overthrottling the spindle, or user outband command
 *                   |                      Also controls the spindle in inband or outband command mode
 *                   |                      In case of fault - it will retry X times, if no success - it will generate fault signal and enters in Emergency/Pause state
 *           ( sequence fifo )
 *                   |
 *                   v
 *            [ motion core ]       - this is the step signal generator for the motors
 *                   |                it does the acceleration - constant - deceleration, or single step instructions.
 *                   |                Acceleration/deceleration factors are calculated at sequence generator.
 *                   |                It has a global speed up/down scaling value - used by sequence generator / command interface
 *                   |
 *                   |
 *                   V
 *            [ step executor ]     - basically it is a step fifo implemented on system clock
 *              ( step fifo )
 *
 **/

#include <string.h>
#include "hw_stuff.h"
#include "events_ui.h"
#include "motion_core.h"

static uint32 *stack_limit;

static inline void CheckStack(void)
{
#ifndef ON_QT_PLATFORM
    if ( *stack_limit != STACK_CHECK_WORD )
    {
         while (1)
         {
             HW_LED_On();
             asm("NOP");
             asm("NOP");
         }
    }
#endif
}


#define pwr_check( a )  ( sys_st & (a) )

static inline void System_Poll( void )
{
    CheckStack();

}


// Main application routine
static inline void ProcessApplication( struct SEventStruct *evmask )
{

    
    motion_poll( evmask );

}

// Main application entry
void main_entry( uint32 *stack_top )
{
    stack_limit = stack_top;
    InitHW();               // init hardware

    motion_init();

    // init
    struct SStepCoordinates origin;
    origin.coord[ COORD_X ] = 0; //130*400;
    origin.coord[ COORD_Y ] = 0; //46*400;
    origin.coord[ COORD_Z ] = 0; //80*400;
    origin.coord[ COORD_A ] = 0; //0*400;
    motion_set_crt_coord( &origin );

}


// Main application loop
void main_loop(void)
{
    struct SEventStruct event;

#ifndef ON_QT_PLATFORM
    while (1)
#endif
    {
        event = Event_Poll( );

        ProcessApplication( &event );

        Event_Clear( event );
        
        System_Poll( );
    }
}
