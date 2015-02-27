#ifndef CNC_DEFS_H
#define CNC_DEFS_H

#ifdef __cplusplus
    extern "C" {
#endif

#include "typedefs.h"

    #define CNC_MAX_COORDS      4    // 4 coordinates handled, order is: X, Y, Z, A
    #define CNC_MAX_COORDMASK   0x0f
    #define COORD_X             0
    #define COORD_Y             1
    #define COORD_Z             2
    #define COORD_A             3

    #define MAX_ISR_STEPS       8
    #define MAX_ISR_WRAPMASK    0x07

    #define STEP_MM             400
    #define STEP_CLOCK          50000       // step clock: 50KHz step clock (20us)  ( interrupt on each 1440 sysclock cycle - TIM1 from APB2 with 72MHz clock )
    #define STEP_CKOFF          2           // 40us clock pulse duration for the selected driver chip

    #define MM_P_M_NOACC        300
    #define STEP_P_SEC_NOACC    ( (MM_P_M_NOACC * STEP_MM) / 60 )       // maximum step speed without acceleration
    
    #define ACC_FACTOR_FP32     ((uint32)( (uint64)(9000LL << 32) / (STEP_CLOCK * ( STEP_CLOCK / 2 )) ))    // 9000step/sec diff in 1/2 sec   
    #define ACC_FACTOR_MMPM     ( 162039 )              // 162039.66   -> find out how to calculate it from FP32

    #define CNC_MAX_FEED        1500
    #define CNC_MAX_FEED_STEP   ( (uint32)(  (((uint64)(CNC_MAX_FEED)<<32LL) * 400LL)  / (60*STEP_CLOCK) )  )

    #define STEP_X          ( 1 << COORD_X )
    #define STEP_Y          ( 1 << COORD_Y )
    #define STEP_Z          ( 1 << COORD_Z )
    #define STEP_A          ( 1 << COORD_A )

    #define DIR_X_PLUS          STEP_X
    #define DIR_Y_PLUS          STEP_Y
    #define DIR_Z_PLUS          STEP_Z
    #define DIR_A_PLUS          STEP_A

    #define DIR_X_MINUS         0
    #define DIR_Y_MINUS         0
    #define DIR_Z_MINUS         0
    #define DIR_A_MINUS         0


    typedef uint32  TStepCoord;
    typedef uint16  TSpindleSpeed;
    typedef uint16  TFeedSpeed;

    struct SStepCoordinates
    {
        TStepCoord  coord[CNC_MAX_COORDS];

    };




#ifdef __cplusplus
    }
#endif

#endif // CNC_DEFS_H
