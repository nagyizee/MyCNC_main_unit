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


    typedef uint32  TStepCoord;

    struct SStepCoordinates
    {
        TStepCoord  coord[CNC_MAX_COORDS];

    };




#ifdef __cplusplus
    }
#endif

#endif // CNC_DEFS_H
