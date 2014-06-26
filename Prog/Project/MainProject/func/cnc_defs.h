#ifndef CNC_DEFS_H
#define CNC_DEFS_H

#ifdef __cplusplus
    extern "C" {
#endif

#include "typedefs.h"

    #define CNC_MAX_COORDS      4    // 4 coordinates handled, order is: X, Y, Z, A
    #define COORD_X             0
    #define COORD_Y             1
    #define COORD_Z             2
    #define COORD_A             3

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

    struct SStepCoordinates
    {
        TStepCoord  coord[CNC_MAX_COORDS];

    };




#ifdef __cplusplus
    }
#endif

#endif // CNC_DEFS_H
