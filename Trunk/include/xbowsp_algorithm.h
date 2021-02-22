/*****************************************************************************
* File name:  xbowsp_algorithm.h
*
* File description:
*   -Algorithm data structure used in sensor calibration and communication protocols with outside world.
*
* $Rev: 16976 $
* $Date: 2010-11-16 14:59:36 -0800 (Tue, 16 Nov 2010) $
* $Author: by-dan $
*
***********************************************************************************/
#ifndef XBOWSP_ALGORITHM_H
#define XBOWSP_ALGORITHM_H

/* raw sensor order  */
enum rawSensor_e {
     XACCEL, YACCEL, ZACCEL,
     XRATE, YRATE, ZRATE,
     XMAG, YMAG, ZMAG,
     XATEMP, YATEMP, ZATEMP,
     XRTEMP, YRTEMP, ZRTEMP,
     BTEMP,
     N_RAW_SENS
};
//     GTEMP, BTEMP,


extern uint32_t rawInertialSensors[N_RAW_SENS];

/* ADAHRS BIT raw sensor order */
enum rawSensorAdBIT_e {
     TWOVOLIOUP, TWOVOLDUP,TWOFIVEREFBUF, THREEVOL,
     FIVEVOLD, FIVEVOLA, FIVEVOLAG,
     SEVENVOLPWR, VSUPPLY,
     XACUR,YACUR, ZACUR,
     XRCUR,YRCUR,ZRCUR,
     PCBTEMP,
     N_ADAHRS_BITSIGNAL
};

extern uint16_t adahrsBitRawSensors[N_ADAHRS_BITSIGNAL];

/* Global Algorithm structure  */
typedef struct {
	 uint32_t rawSensors[N_RAW_SENS];
     int32_t tempCompBias[6];     		/* the current temp comp inertial sensor bias  */
     double scaledSensors[N_RAW_SENS];  /*g's, rad/s, G, deg C, (rotated to user body frame) */
     double correctedRate[3];     		/*agorithm corrected rate   */
     double tangentRates[3];
     double tangentAccels[3];
     double attitude[3];  				/* from qb2t  */
     double leveledMags[3];  /*tangent x and y mags (body mags rotated to tangent frame w/o user alignment) */
     volatile int calState;  			/* current calibration state */
     volatile uint16_t counter;  			/* inc. with every continous mode output packet */
     volatile uint32_t timer;  			/* timer since power up (ms)                    */

     double downRate;

     double ExtAccels[3];              	/* external accels  */
     double ExtMags[3];              	/* external mag     */
     unsigned int ExtMagSerialNumber;
     double compassHeading;
} AlgorithmStruct;

extern AlgorithmStruct gAlgorithm;

#endif
