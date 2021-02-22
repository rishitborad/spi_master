/*****************************************************************************
** Magnetometer interface 
** This is a generalized magnetometer interface
** possibly implemented using the Honeywell HMC5883L magnetometer 
** 
*****************************************************************************/
#ifndef MAGNETOMETER_H
#define MAGNETOMETER_H
#include <stdint.h>
uint8_t InitMagnetometer(); // this will use boardDefinition.h to set up 
                            // the IO lines returns TRUE if init was successful
#define MAG_RANGE_4000_MILLI_GA 4000
#define MAG_RANGE_4700_MILLI_GA 4700
uint8_t MagnetometerConfig(uint32_t *rangeInMilliGauss);

uint8_t MagnetometerStartReading(); // true if success
uint8_t IsMagnetometerDoneReading(); // true if read is complete
uint8_t MagnetometerGetLastReading(int16_t *readings); // true if success
void MagnetomterDataReadyIRQ(void); // handle mag drdy interrupt


uint8_t  MagnetometerWhoAmI(uint32_t *whoami); // returns true if value is as expected
uint16_t MagnetometerGetGain();
uint8_t  MagnetometerSelfTest();

#endif /* MAGNETOMETER_H */