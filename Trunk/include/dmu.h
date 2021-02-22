/** ***************************************************************************
 * @file   dmu.h
 * @Author
 * @date   September, 2008
 * @brief  Copyright (c) 2013, 2014 All Rights Reserved.
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
 * header file for all other files to include, basic information about
 * system configuration
 * associated C file only.
 *
******************************************************************************/
#ifndef DMU_H
#define DMU_H

/// QMATH defines, look in MATH_TYPE == FLOAT_MATH of IQMathCpp.h file
/// to get floating point equivalents
typedef float iq27;
typedef float iq30;
typedef float iq23;
typedef float iq29;

typedef int bool;
typedef int BOOL;

#define IQ27rsmpy(A,B) (A * B)
#define IQ27(x) (x)

typedef uint8_t tBoolean;

#ifndef TRUE
    #define TRUE 1
    #define FALSE 0
#endif

#ifndef NULL
    #define NULL 0
#endif

#define MAXUINT32 4294967295  ///< max unsigned 32 bit int=> ((2^32)-1)
#define MAXUINT16      65535  ///< max unsigned 16 bit int=> ((2^16)-1)
#define MAXINT16       32767  ///< max signed 16 bit int=> ((2^15))
#define MININT16     (-32768) ///< max negative signed 16 bit int=> ((2^15)+1)
#define CONVERT_XBOW_TO_380(x) (((x) >> 16) + (int32_t) MININT16) ///< to signed
#define CONVERT_380_TO_XBOW(x) ((((x) - (uint32_t) MININT16 )) << 16 ) ///< to unsigned

#define D2R        (PI/180.0)
#define DEG2RAD(d) ( (d) * D2R )
#define SIGMA      1.0e-8
#define KNOT2MPSEC 5.144444444e-1
#define SQUARE(x) ((x)*(x))

/// oneKhzTimer.c definitions
extern void SynchStart(void);
extern void SynchStop(void);

#endif /* DMU_H */