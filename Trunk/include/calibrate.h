/**	\file calibrate.h
 * 	\brief Header file for calibration algorithms.
 *
 *
 */

#include "xbowsp_algorithm.h"

void CalibrateSensors(uint32_t *rawReadings, uint16_t *gain, iq27 *scaledSensors);
void InitCalibration(void);

void orientSensors(  iq27 *scaledSensors );
