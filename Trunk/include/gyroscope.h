/*****************************************************************************
** Gyroscope interface 
** This is a generalized Gyro interface
** possibly implemented using the Honeywell HMC5883L Gyro 
** 
*****************************************************************************/
#ifndef GYROSCOPE_H
#define GYROSCOPE_H
#include <stdint.h>
uint8_t InitGyro(); // this will use boardDefinition.h to set up 
                            // the IO lines returns TRUE if init was successful

#define GYRO_RANGE_500DPS 500
#define GYRO_RANGE_250DPS 250
#define DEFAULT_GYRO_RANGE GYRO_RANGE_250DPS
uint8_t GyroConfig(uint32_t *rangeInDps, uint32_t *outputDataRate);

void GyroDataReadyIRQ(void);
uint8_t GyroStartReading(); // true if success
uint8_t IsGyroDoneReading(); // true if read is complete
uint8_t GyroGetLastReading(int16_t *readings); // true if success


uint8_t  GyroWhoAmI(uint32_t *whoami); // returns true if value is as expected
uint16_t GyroGetGain();
uint8_t  GyroSelfTest();


// Some gyros have onboard temperature sensing, provide access to it
uint8_t GyroTempExists();
uint8_t GyroTempGetLastReading(int16_t* reading);
uint8_t GyroTempGetTemperature(int16_t reading, float *out);

////////// for second gyro
uint8_t InitGyro2(); // this will use boardDefinition.h to set up 
                            // the IO lines returns TRUE if init was successful
uint8_t Gyro2Config(uint32_t *rangeInDps); // invensense doesn't have sync so it samples at a constant rate (8k)

void Gyro2DataReadyIRQ(void);
uint8_t Gyro2StartReading(); // true if success
uint8_t IsGyro2DoneReading(); // true if read is complete
uint8_t Gyro2GetLastReading(int16_t *readings); // true if success


uint8_t  Gyro2WhoAmI(uint32_t *whoami); // returns true if value is as expected
uint16_t Gyro2GetGain();
uint8_t  Gyro2SelfTest();


// Some gyros have onboard temperature sensing, provide access to it
uint8_t Gyro2TempExists();
uint8_t Gyro2TempGetLastReading(int16_t* reading);
uint8_t Gyro2TempGetTemperature(int16_t reading, float *out);




#endif /* GYROSCOPE_H */