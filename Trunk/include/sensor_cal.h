/**********************************************************************************
* File name:  sensor_cal.h
*
* File description: 
*   -Header file 
*               
* $Rev: 17451 $
* $Date: 2011-02-09 16:52:56 -0800 (Wed, 09 Feb 2011) $
* $Author: by-dan $
*
*********************************************************************************/
#ifndef SENSOR_CAL_H
#define SENSOR_CAL_H 

#include "sensor.h"

extern void calibrateSensors(void); 
extern void gSensComp(void);
extern BOOL magCal(int state, double *hardIron, double *softIron);
extern unsigned int findTable(unsigned int tableGroup,unsigned int sensorType, unsigned int sensorIndex);
extern int fixedPtInterpolation(LUTinOutStruct *myTableInOut);
extern double floatingPtInterpolation(LUTinOutStruct *myTableInOut);
extern void calibrate(sensorStruct *s);
extern void initRPYAlignOffsetMat(void) ;
extern void calcMagScaleParameter(void);
extern void rpyAlignOffsetComp(double *vector);
extern void calcLeveledMags(double *AHRSrp,double *remoteMag) ;
extern void filterRemotemagForCal(double *magV, BOOL *reset) ;
extern void stopMagCal(double *minMag, double *maxMag, unsigned short *numofpts, double *hardIron, double *softIron);
extern void calcMagHeading(double *AHRSrp,double *remoteMag, double*magHeading) ;
extern void filterRemotemag(double *magV, BOOL *reset) ;
extern void magAlign(void); 
extern void initMagAlignResult(void);   
extern BOOL MagCalOutOfBounds (void);
extern void lowPass(uint32_t * out, uint32_t input, int resolution, int shiftCoef) ;
extern void getADAHRSsolution(sensorStruct *sensor);


extern int16_t magRollOffset;
extern int16_t magPitchOffset;
extern int16_t hardIronBias[2];
extern uint16_t softIronScaleRatio; 
extern int16_t softIronAngle; 
extern uint16_t lastLUTRstIndexA[N_TABLES_A + 1]; 

#define METRIC_G 9.8066500	/* Gravity in meters /second^2 */

#define TABLE_GROUP_A_INDEX    				1
#define TABLE_GROUP_B_INDEX    				2
#define TABLE_GROUP_C_INDEX     			3
#define SENSOR_TYPE_INDEX_TEMP              1
#define SENSOR_TYPE_INDEX_INERTIAL          2
#define SENSOR_TYPE_INDEX_PRESSURE        	2
#define SENSOR_TYPE_INDEX_OAT               2

#endif