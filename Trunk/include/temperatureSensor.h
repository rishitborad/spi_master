/*****************************************************************************
** Temperature sensor interface 
** This is a generalized thermistor
** possibly implemented using the TI TMP100 temperature sensor
** 
*****************************************************************************/
#ifndef TEMP_SENSOR_H
#define TEMP_SENSOR_H
#include <stdint.h>
uint8_t InitTemperatureSensor(); // this will use boardDefinition.h to set up 
                            // the IO lines returns TRUE if init was successful

uint8_t TemperatureStartReading(); // true if success
uint8_t IsTemperatureDoneReading(); // true if read is complete
uint8_t TemperatureGetLastReading(int16_t *t);

uint16_t TemperatureGetGain();
uint8_t  TemperatureSelfTest();

#endif /* TEMP_SENSOR_H */