#ifndef __TYPEDEFS_H
#define __TYPEDEFS_H

#ifdef __cplusplus
 extern "C" {
#endif

#ifndef NULL
#define NULL    0
#endif


#ifndef ON_QT_PLATFORM

    typedef uint8_t     uint8;
    typedef int8_t      int8;
    typedef uint16_t    uint16;
    typedef int16_t     int16;
    typedef uint32_t    uint32;
    typedef int32_t     int32;
    typedef uint64_t    uint64;
    typedef int64_t     int64;
    typedef uint32_t    bool;


    #define true    1
    #define false   0

#else

 typedef unsigned char          uint8;
 typedef signed char            int8;
 typedef unsigned short         uint16;
 typedef short                  int16;
 typedef unsigned int           uint32;
 typedef int                    int32;
 typedef unsigned long long     uint64;
 typedef long long              int64;
#ifndef __cplusplus
 typedef int                    bool;
#endif



#define true    1
#define false   0

#endif


#ifdef __cplusplus
}
#endif


#endif
