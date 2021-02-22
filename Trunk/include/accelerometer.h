/**
 * @section DESCRIPTION
 *
 * This is the generic accelerometer interface, it should be implemented
 * by whichever accelerometer is in use
 * 
 */  

#ifndef ACCELEROMETER_H
#define ACCELEROMETER_H

uint8_t InitAccelerometer(); // this will use boardDefinition.h to set up 
                            // the IO lines returns TRUE if init was successful

#define ACCEL_RANGE_8G   8 
#define ACCEL_RANGE_4G   4 
#define ACCEL_RANGE_2G   2 

uint8_t AccelerometerConfig(uint32_t *rangeInGs, uint32_t *outputDataRate);

uint8_t AccelerometerStartReading(); // true if success
uint8_t IsAccelerometerDoneReading(); // true if read is complete
uint8_t AccelerometerGetLastReading(int16_t *readings); // true if success
void AccelerometerDataReadyIRQ(void); // handle data ready interrupt

uint8_t  AccelerometerWhoAmI(uint32_t *whoami); // returns true if value is as expected
uint16_t AccelerometerGetGain();
uint8_t  AccelerometerSelfTest();

#endif /* ACCELEROMETER_H */
