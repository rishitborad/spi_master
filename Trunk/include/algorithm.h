//###########################################################################
//
// FILE:   algorithm.h
//
// TITLE:  DMU420 header file for algorithm structures and constants. 
//
// $Id: algorithm.h 8719 2009-04-10 23:20:59Z bt-dliccardo $
//         
//###########################################################################
//  
//
//
//AUTHOR:  Darren Liccardo
//
// 10.23.2006 DRA moved software part numbers to newly created version.h
// 03/01/2007 DA Added "void dropToHighGainAHRS(void);" per BugID224
// 03/12/2007 DA Increased "MAG_VAR_TIMEOUT" to 10 Min.
//###########################################################################
#ifndef _ALGORITHM_H_
#define _ALGORITHM_H_

#include "BITStatus.h"
#include "xbowsp_algorithm.h"


//TODO: GPS startup tuning, algorithm user switch testing, clean up GPS comm, implement heading-track field and dynamic motion switch.


#define FAKE_ZERO_VEL_GPS 0  //fake GPS up zero vel for debug
#define FAKE_CONSTANT_VEL_GPS 0 //fake const vel GPS only when actual GPS is locked
#define GPS_DROP_TIME 3000  //ms of no GPS updates until algorithm drops to AHRS/VG
#define MAG_VAR_TIMEOUT 600000 /// in ms, maximum age of magVar from the last mag var calculation 
							   ///that is accepted for the NAV filter
							   ///10 min. now
#define NO_ENOUGH_GPS_VEL_NAV_TIME 60000  //stay in Nav for XXXX ms 
										   //when no enough GPS Vel 
										   //before algorithm drops to AHRS/VG

#define LONGER_STOP_FREE_HEADING_TIME   20000  //stay in NAV for XXXXms
										  //Heading initialization with GPS heading
										  //needs to be applied when the stoping period is
										  //more than this number.
//#define YAW_TRACK_SPEED_SQ (0.75*0.75)
#define YAW_TRACK_SPEED_SQ (1.0*1.0)


#define TILT_INIT_TIME 200  //the number of (100Hz) algorithm iterations to average tilt data to create initial attitude estimate


//algorithm states
enum { HG_AHRS=0, LG_AHRS=1, HG_NAV=2, LG_NAV=3, HARDWARE_STABILIZE, INIT_ATTITUDE, INIT_NAV };

//*****filter parameters*******
//initial P
static const double INIT_P[4][13]={
								{10,10,10, 3.0e-4,2.0e-4,2.0e-4,2.0e-4, 1.0e-5,1.0e-5,1.0e-5, 2.0e-8,2.0e-8,2.0e-8},  //STANDARD
								{10,10,10, 3.0e-4,2.0e-4,2.0e-4,2.0e-4, 1.0e-5,1.0e-5,1.0e-5, 2.0e-8,2.0e-8,2.0e-8},	 //INTERNATIONAL
								{10,10,10, 4.0e-4,4.0e-4,4.0e-4,4.0e-4, 3.0e-5,3.0e-5,3.0e-5, 2.0e-8,2.0e-8,2.0e-8},  //VG325
								{10,10,10, 4.0e-4,4.0e-4,4.0e-4,4.0e-4, 3.0e-5,3.0e-5,3.0e-5, 2.0e-8,2.0e-8,2.0e-8}}; //VG327

//Q (*dt)
static const double Q_QUAT[4][4] = { //HG_AHRS, LG_AHRS, HG_NAV, LG_NAV
									{ 1.0e-7, 2.5e-8, 2.0e-9, 2.0e-9 },  //STANDARD
									{ 1.0e-7, 2.5e-8, 2.0e-9, 2.0e-9 },  //INTERNATIONAL
									{ 4.0e-7, 2.0e-7, 8.0e-9, 6.0e-9 },  //VG325     
									{ 4.0e-7, 2.0e-7, 8.0e-9, 6.0e-9 }}; //VG327

static const double Q_RATE[4][4] = { //HG_AHRS, LG_AHRS, HG_NAV, LG_NAV
								    { 2.4e-10, 6.0e-11, 4.0e-13, 4.0e-13 },  //STANDARD
								    { 2.4e-10, 6.0e-11, 4.0e-13, 4.0e-13 },  //INTERNATIONAL
								    { 2.0e-9,  6.0e-10, 3.0e-11, 3.0e-11 },  //VG325
								    { 2.0e-9,  6.0e-10, 3.0e-11, 3.0e-11 }}; //VG327

static const double Q_VEL[4][4] = { //HG_AHRS, LG_AHRS, HG_NAV, LG_NAV
								   { 0.0, 0.0, 7.0e-2, 8.0e-4 },  //STANDARD
								   { 0.0, 0.0, 7.0e-2, 8.0e-4 },  //INTERNATIONAL
								   { 0.0, 0.0, 7.0e-2, 8.0e-4 },  //VG325
								   { 0.0, 0.0, 7.0e-2, 8.0e-4 }}; //VG327
/*
static const double Q_VEL[4][4] = { //HG_AHRS, LG_AHRS, HG_NAV, LG_NAV
								   { 0.0, 0.0, 2.0e-3, 8.0e-4 },  //STANDARD
								   { 0.0, 0.0, 2.0e-3, 8.0e-4 },  //INTERNATIONAL
								   { 0.0, 0.0, 2.0e-3, 8.0e-4 },  //VG325
								   { 0.0, 0.0, 2.0e-3, 8.0e-4 }}; //VG327
*/
								
static const double Q_ACCEL[4][4] = { //HG_AHRS, LG_AHRS, HG_NAV, LG_NAV
									 { 0.0, 0.0, 1.2e-16, 6.0e-17 },  //STANDARD
									 { 0.0, 0.0, 1.2e-16, 6.0e-17 },  //INTERNATIONAL
									 { 0.0, 0.0, 1.2e-16, 6.0e-17 },  //VG325
									 { 0.0, 0.0, 1.2e-16, 6.0e-17 }}; //VG327

//R

//mag heading observation noise(GPS down)
static const double R_MAG_AHRS[4] = {0.010,  //STANDARD
									 0.010,  //INTERNATIONAL
									 0.010,  //VG325
									 0.010}; //VG327



//mag heading observation noise (GPS up)
static const double R_MAG_NAV[4] = {0.005,   //STANDARD
									0.005,  //INTERNATIONAL
									0.005,  //VG325
									0.005}; //VG327

//tilt measurment observation noise
static const double R_TILT[4] = {0.015,  //STANDARD
								 0.015,  //INTERNATIONAL
								 0.015,  //VG325
								 0.015};  //VG327
								 
static const double R_GPS[3] = {0.2, 0.2, 0.2}; //diagonal GPS observation noise

static const double R_GPS_STATIC[3] = {0.3, 0.3, 0.3}; //diagonal GPS observation noise static

#define STATIC_VEL 5.0  //static gps vel

//**tilt update:

//tilt error max
static const double TILT_ERROR_CAP[4] = { 0.0174533,  //STANDARD //1.0 degree
										  0.0174533,  //INTERNATIONAL //1.0 degree
										  0.0261799,  //VG325 //1.5 degrees
										  0.0261799};  //VG327 //1.5 degrees
											
//tilt yaw switch (linear ramp down in gain) - OLD SETTINGS
/*static const double TILT_YAW_SWITCH_MAX[4] = { 	0.00006545,  //STANDARD //0.75 degrees/sec  *pi/180 *0.5/ALGORITHM_FREQUENCY
												0.00006545,  //INTERNATIONAL //0.75 degrees/sec *pi/180 *0.5/ALGORITHM_FREQUENCY
												0.00026180,  //VG325 //3.0 degrees/sec *pi/180 *0.5/ALGORITHM_FREQUENCY
												0.00026180}; //VG327 //3.0 degrees/sec *pi/180 *0.5/ALGORITHM_FREQUENCY
															
static const double TILT_YAW_SWITCH_MIN[4]= {   0.00003491,  //STANDARD //0.4 degrees/sec *pi/180 *0.5/ALGORITHM_FREQUENCY
												0.00003491,  //INTERNATIONAL //0.4 degrees/sec *pi/180 *0.5/ALGORITHM_FREQUENCY
												0.00013090,  //VG325 //1.5 degrees/sec *pi/180 *0.5/ALGORITHM_FREQUENCY
												0.00013090}; //VG327 //1.5 degrees/sec *pi/180 *0.5/ALGORITHM_FREQUENCY
*/
//turn switch threshold now controlled by user. 
//linear ramp down in gain beginning at turnSwitchThreshold and ending at turnSwitchThreshold*TILT_YAW_SWITCH_RAMP														
#define TILT_YAW_SWITCH_RAMP 2.0

#define TILT_YAW_SWITCH_GAIN 0.05  //gain at turnSwitchThreshold*TILT_YAW_SWITCH_RAMP or greater yaw rate

//**mag update:

//yaw error cap
static const double YAW_ERROR_CAP[4] = { 0.0436332,  //STANDARD //2.5 deg
										 0.0436332,  //INTERNATIONAL //2.5 deg
										 0.069813,   //VG325 //4.0 deg
										 0.069813 }; //VG327 //4.0 deg
//yaw error cap for GPS heading reinitialization 
//August 20,2007 by Jung
#define YAW_ERROR_CAP_FOR_REINIT	0.17452	// 10.0 deg

//**gps update:

//vtan error max
#define VTAN_ERROR_CAP 3.0 //m/s
//max accel bias
#define MAX_ACCEL_BIAS 0.09  //m/s^2


//rad/s The algorithm will not switch to low gain until all rate bias state are below this bound
static const double RATE_BIAS_STARTUP_BOUND[4] = {  0.01396263, //STANDARD (0.8 deg/sec)
													0.01396263, //INTERNATIONAL (0.8 deg/sec)
													0.02268928,	//VG325 (1.3 deg/sec)
													0.02268928}; //VG327 (1.3 deg/sec)



//Kalman filter variables
typedef struct { 
	double P[13][13];  //state covariance
	iq30 trace;   //P trace
	double temp1[13][13];
	union {
	   double temp2[13][13];
	   int16_t sMagx[338];
	};
	union {
	   double temp3[13][13];
	   int16_t sMagy[338]; 
	};
	double invMat[3][3];
	double accelBias[3];  //filter states (m/s^2)
	double rateBias[3];  //filter states  (rad/s)
	iq30 test;  //for debug
	double test2; //for debug
	double vTanGPS[3];
	signed long LonLatHGPS[3];
	uint32_t velITOW; //last good velocity GPS ITOW
	uint32_t posITOW; //last good position GPS ITOW
	iq29 magVar;
} FilterStruct;


//algorithm functions
void measurementUpdateGPS(iq23 vx, iq23 vy, iq23 vz);
void measurementUpdateTilt(void);
void resetAlgorithm(void);
void initNavFilter(void);
void dropToHighGainAHRS(void);

//recorded packet transmit counter.
extern uint16_t recorded_count;

#endif
