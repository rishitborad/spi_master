/*****************************************************************************
* File name:  xbowsp_generaldrivers.h
*
* File description: 
*   -Configuration and calibration data structure
*
* File history:
*
* $Rev: 17465 $
* $Date: 2011-02-09 20:20:16 -0800 (Wed, 09 Feb 2011) $
* $Author: by-denglish $
*                                          per the latest design.
***********************************************************************************/
#ifndef XBOWSP_DRIVERS_H
#define XBOWSP_DRIVERS_H

#include "xbowsp_version.h"

/* hard and soft iron resolution (2^16/2) */
#define IRON_SCALE 32768

struct productConfiguration_BITS  {      /* bits   description */
    uint16_t mags:1;             /* 0 */
    uint16_t gps:1;              /* 1 */
    uint16_t algorithm:1;        /* 2 */
    uint16_t extAiding:1;        /* 3 */
    uint16_t architecture:4;     /* 4-7  */
    uint16_t rsvd:7;			    /* 8-14 */
    uint16_t isADAHRS:1;         /* 15 */
};

union ProductConfiguration {
    uint16_t          all;
    struct productConfiguration_BITS bit;
};


struct orientation_BITS  {        /* bits   description  						*/
    uint16_t forwardAxisSign:1;    /* 	  0 is pos, 1 is neg                    */
    uint16_t forwardAxis:2;        /*     0=X,1=Y,2=Z axes forward, 3=do not use */
    uint16_t rightAxisSign:1;      /*  0 is pos, 1 is neg                        */
    uint16_t rightAxis:2;          /*	  0=Y,1=Z,2=X axes forward, 3=do not use*/
    uint16_t downAxisSign:1;       /* 	  0 is pos, 1 is neg                    */
    uint16_t downAxis:2;           /* 	  0=Z,1=X,2=Y axes forward, 3=do not use*/
    uint16_t rsvd:7;               // 9:15
};

union Orientation {
    uint16_t          all;
    struct orientation_BITS bit;
};



/* EEPROM Data Structure: configuration data */ 
typedef struct {
	uint16_t calibrationCRC;  					/* CRC on the calibration structure */
	uint16_t packetRateDivider;  		        /* continuous packet rate divider */     
    uint16_t baudRateUser;                      /* user port */	
	uint16_t packetType;					    /* continuous packet type */ 
	
	union Orientation orientation; 				/* user defined axis orientation */
    
	uint16_t turnSwitchThreshold;               /* turn switch threshold*/
	
    // these are booleans and could be stored in bits
	uint16_t hardwareStatusEnable;  		    /* hardware status enable */
	uint16_t comStatusEnable;                   /* communication status enable */
	uint16_t softwareStatusEnable;              /* software status enable */
	uint16_t sensorStatusEnable;                /* sensor status enable */
    	
	int16_t  baroCorrection;                      /* barometer correction */
	int16_t  OffsetAnglesExtMag[2];               /* external mag roll, pitch offset */
	int16_t  OffsetAnglesAlign[3];                /* roll, pitch, yaw offset alignment  */
	int16_t  hardIronBiasExt[2];                  /* external mag X,Y hard iron bias */

	uint16_t softIronScaleRatioExt; 		      /* external mag soft iron scale ratio */	
	int16_t  softIronAngleExt;                    /* external mag soft iron angle */  
	
	uint16_t port1Usage;						  /*  port 1 usage */
	uint16_t port2Usage;                          /*  port 2 usage */
	uint16_t port3Usage;                          /*  port 3 usage */
	uint16_t port4Usage;                          /*  port 4 usage */
	uint16_t port1BaudRate;                       /*  port 1 baud rate */
	uint16_t port2BaudRate;                       /*  port 2 baud rate */
	uint16_t port3BaudRate;                       /*  port 3 baud rate */
	uint16_t port4BaudRate;                       /*  port 4 baud rate */
	uint16_t remoteMagSerNo[2];                   /*  remote mag serial number, high 16-bits [0x002A] remote mag serial number, low 16-bits */
} ConfigurationStruct;

extern ConfigurationStruct gConfiguration;


#define N_MISALIGN                           18
#define N_RESERVED_CAL                           6

#define N_TABLES_A    		15 		/* 6+6+3   */
#define N_ROWS_TABLE_A   	360 	/* 162+99+99   */
#define N_COLS_TABLE_A        2    

#define N_TABLES_B            7  	/* 5+2      */
#define N_ROWS_TABLE_B   	160
#define N_COLS_TABLE_B        2    

#define N_TABLES_C    		7  		/* 1+1+reserved 5 */
#define N_ROWS_TABLE_C   	72
#define N_COLS_TABLE_C       2    

#define G_SENS_MATRIX_SIZE               9

#define CALIBRATION_OFFSET 0x0100

#define	CAL_SERIAL_NUMBER		0x100
#define CAL_VERSION             0x102
#define CAL_INDEXA				0x142 
#define CAL_TABLEA				0x152 		
#define CAL_MISALIGN			0x6F2
#define CAL_RESERVED            0x716 
#define PROD_CONFIG				0x71C
#define CAL_INDEXB				0x800
#define CAL_TABLEB              0x808
#define CAL_INDEXC              0xA88
#define CAL_TABLEC              0xA90            
#define CAL_GSENS               0xBB0

/* Accel & Gyro senor max */
#define ACCEL_MAX               0xBC2
#define GYRO_MAX                0xBC3
#define MAG_MAX                0x8001 
/* reserve 3 additional sensor range */ 


#define EEPROM_CALIBRATION_END 0xC03 
/*  BIT limit checking */
/* 16 bit values  */

#define EEPROM_BITLIMITS_16bit     0xBCC 
#define N_ROWS_BITLIMITS16 12
#define N_COLS_BITLIMITS16 2
#define N_ROWS_BITLIMITS32  7
#define N_COLS_BITLIMITS32 2

#define N_BITLIMITS16              ( N_ROWS_BITLIMITS16 * N_COLS_BITLIMITS16 ) 
#define EESIZE_BITLIMITS_16bit     ( N_BITLIMITS16 *  sizeof(uint16_t)  ) 
/* 32 bit values */
#define EEPROM_BITLIMITS_32bit     0xBE4
#define N_BITLIMITS32              (N_ROWS_BITLIMITS32 * N_COLS_BITLIMITS32 )
#define EESIZE_BITLIMITS_32bit     (N_BITLIMITS32 * sizeof(uint32_t)  )

/* magnetometer limits */ 
#define EEPROM_MAG_LIMITS				0xC00
#define ROLL_INCIDENCE_LIMIT			EEPROM_MAG_LIMITS
#define SIZE_OF_ROLL_INCIDENCE_LIMIT	sizeof(uint16_t) 
#define PITCH_INCIDENCE_LIMIT           ( EEPROM_MAG_LIMITS + SIZE_OF_ROLL_INCIDENCE_LIMIT )
#define SIZE_OF_PITCH_INCIDENCE_LIMIT	sizeof(uint16_t)
#define HARD_IRON_LIMIT                 0xC02 
#define SIZE_OF_HARD_IRON_LIMIT			 sizeof(uint16_t) 
#define SOFT_IRON_LIMIT                 ( HARD_IRON_LIMIT + SIZE_OF_HARD_IRON_LIMIT )
#define SIZE_OF_SOFT_IRON_LIMIT			sizeof(uint16_t) 


typedef struct {
    uint32_t serialNumber;
    char   versionString[N_VERSION_STR];  
    
    uint16_t calibrationTableIndexA[N_TABLES_A + 1];  /* index into calibrationTables of first element for each sensor (including 7th sensor)  */
    uint32_t calibrationTablesA[N_ROWS_TABLE_A][N_COLS_TABLE_A];   /* table order: (Temp Tables: xAccel,yAccel,zAccel,xRate,yRate,zRate), Inertial Tables: (same order), mag tables (x,y,z)  */
    
    float misalign[N_MISALIGN];
    
    union  ProductConfiguration productConfiguration;
    
    uint16_t calibrationTableIndexB[N_TABLES_B + 1]; 
    uint32_t calibrationTablesB[N_ROWS_TABLE_B][N_COLS_TABLE_B];    
    uint16_t calibrationTableIndexC[N_TABLES_C + 1];  
    uint32_t calibrationTablesC[N_ROWS_TABLE_C][N_COLS_TABLE_C];    
    
    int32_t gSenForGyros[G_SENS_MATRIX_SIZE];  
    
 	float AccelSensorRange;					
	float GyroSensorRange;
    float MagSensorRange;
    
    uint16_t bitLimits[N_BITLIMITS16];
    uint32_t bitLimits32[N_BITLIMITS32];
    
	uint16_t RollIncidenceLimit;              	
	uint16_t PitchIncidenceLimit;             	
    
	uint16_t HardIronLimit;                  
	uint16_t SoftIronLimit;                  
    
} CalibrationStruct;
extern CalibrationStruct gCalibration;



typedef struct {
    /***Input ***/
    unsigned int  TableLength;	             /* table size (starting with 0)                     */
    uint8_t          firstColumnUnsigned;	     /* TRUE: first column should be treated as unsigned */
    /* FALSE, it is signed.                             */
    unsigned int  bIndexLastSearch;         /* index of the beginning row at the last search    */
    /*(where the last search ended)                     */
    unsigned int  numberToBeSearched;	     /* search input--left column                        */
    /***output***/
    unsigned int  bLeftValue, eLeftValue;   /* search result: first row's columns(beginning and end) */
    int           bRightValue, eRightValue; /* search result:  second row's columns (beginning and end) */
    
}LUTinOutStruct;


/* 
* 'VR' and 'VA' only use UINT8 field 
* bootloader reads unspecified UINT (but assumed 2 cells wide)
* and sensor_init() assumes 16 bits
*/
typedef struct {
    unsigned int major;
    unsigned int minor;
    unsigned int patch;
    unsigned int stage;
    unsigned int build;
} softwareVersionStruct;

/* servicing/calling frequency of serial port transmit routine */
#define SERIAL_TX_ROUTINE_FREQUENCY 100 /* Hz */


#endif

