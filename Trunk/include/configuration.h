


#include "xbowsp_generaldrivers.h"

// uint32_t calibrationTables[162+99+99][2]; table order: (Temp Tables: xAccel,yAccel,zAccel,xRate,yRate,zRate), Inertial Tables: (same order), mag tables (x,y,z)
#define ACCEL_CAL_TABLE_OFFSET 0
#define ACCEL_CAL_TABLE_SIZE 54

#define RATE_CAL_TABLE_OFFSET (NUM_AXIS * ACCEL_CAL_TABLE_SIZE)
#define RATE_CAL_TABLE_SIZE 33

#define MAG_CAL_TABLE_OFFSET (RATE_CAL_TABLE_OFFSET + (NUM_AXIS * RATE_CAL_TABLE_SIZE))
#define MAG_CAL_TABLE_SIZE 33

#define TEMP_BIAS_OFFSET 0
#define TEMP_SCALE_OFFSET 6

#define TEMP_COUNT 0
#define BIAS_VALUE  1

#define SENSOR_COUNT 0
#define SCALE_VALUE 1


// Data structure specifying how the user sets up the device
struct userBehavior_BITS  {      // bits   description
   uint16_t freeIntegrate:1;       // 0
   uint16_t useMags:1;             // 1
   uint16_t useGPS:1;              // 2
   uint16_t stationaryLockYaw:1;   // 3
   uint16_t restartOnOverRange:1;  // 4
   uint16_t dynamicMotion:1;       // 5
   uint16_t rsvd:10;               // 6:15
};

union UserBehavior
{
   uint16_t          all;
   struct userBehavior_BITS bit;
};


