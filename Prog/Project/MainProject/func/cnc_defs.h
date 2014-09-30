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
    #define STEP_SEC            10000   // 10kHz step clock - this is the base -> 25rpm -> 25mm/sec -> 1500mm/min
    #define MM_P_M_NOACC        300
    #define STEP_P_SEC_NOACC    ( (MM_P_M_NOACC * STEP_MM) / 60 )       // maximum step speed without acceleration

    #define ACC_FACTOR_FP32     ( (uint64)(1.4 * ( 1 << 32)) )      //  2.5mm/s -> 6mm/s in 0.100s time - this means: 0.1step/sys clock tick  -> 0.24 step/tick in 10000ticks interval:  a = 0.24-0.1 / 1000 = 0.00014
    #define ACC_START_SPEED     ( 1000LL )                          // 150mm/min -> 1000steps/sec


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
