/**********************************************************************************
* File name:  s_eeprom_conversion_to_xbow.h
*  This is the conversion between xbow style eeprom and the faux simulated 
*
**********************************************************************************/
#ifndef S_EEPROM_CONV_H
#define S_EEPROM_CONV_H
struct sEepromTranslation {
    const int xbowAddress;
    const void* flashAddress;
    const int maxSize;
}; 
#include "xbowsp_fields.h"
#define NOT_IN_FLASH ((void*)-1)

static const struct sEepromTranslation gEepromTranslation[] =
{
    // address in xbow world            // address in dmu380 sim eeprom
    { CRC_ID, &gEepromInFlash.table.configuration.calibrationCRC, sizeof(gEepromInFlash.table.configuration.calibrationCRC)},
  {  PACKET_RATE_DIVIDER_FIELD_ID,  &gEepromInFlash.table.configuration.packetRateDivider  ,  sizeof(gEepromInFlash.table.configuration.packetRateDivider) },    
  {  BAUD_RATE_USER_ID,  &gEepromInFlash.table.configuration.baudRateUser  ,  sizeof(gEepromInFlash.table.configuration.baudRateUser) },
  {  PACKET_TYPE_FIELD_ID,  &gEepromInFlash.table.configuration.packetType  ,  sizeof(gEepromInFlash.table.configuration.packetType) },
  {  ORIENTATION_FIELD_ID,  &gEepromInFlash.table.configuration.orientation  ,  sizeof(gEepromInFlash.table.configuration.orientation) },
  {  TURN_SWITCH_THRESHOLD_FIELD_ID,  &gEepromInFlash.table.configuration.turnSwitchThreshold  ,  sizeof(gEepromInFlash.table.configuration.turnSwitchThreshold) },
  {  HARDWARE_STATUS_ENABLE_FIELD_ID, &gEepromInFlash.table.configuration.hardwareStatusEnable , sizeof(gEepromInFlash.table.configuration.hardwareStatusEnable) },
  {  COM_STATUS_ENABLE_FIELD_ID,  &gEepromInFlash.table.configuration.comStatusEnable  ,  sizeof(gEepromInFlash.table.configuration.comStatusEnable) },
  {  SOFTWARE_STATUS_ENABLE_FIELD_ID, &gEepromInFlash.table.configuration.softwareStatusEnable , sizeof(gEepromInFlash.table.configuration.softwareStatusEnable) },
  {  SENSOR_STATUS_ENABLE_FIELD_ID,  &gEepromInFlash.table.configuration.sensorStatusEnable  ,  sizeof(gEepromInFlash.table.configuration.sensorStatusEnable) },
  {  BARO_CORRECTION_FIELD_ID,  &gEepromInFlash.table.configuration.baroCorrection  ,  sizeof(gEepromInFlash.table.configuration.baroCorrection) },
  {  OFFSET_ANGLES_EXT_MAG_FIELD_ID,  &gEepromInFlash.table.configuration.OffsetAnglesExtMag[0]  ,  sizeof(gEepromInFlash.table.configuration.OffsetAnglesExtMag[0]) },
  {  OFFSET_ANGLES_EXT_MAG_FIELD_ID+1,  &gEepromInFlash.table.configuration.OffsetAnglesExtMag[1]  ,  sizeof(gEepromInFlash.table.configuration.OffsetAnglesExtMag[1]) },
  {  OFFSET_ANGLES_ALIGN_FIELD_ID,  &gEepromInFlash.table.configuration.OffsetAnglesAlign[0]  ,  sizeof(gEepromInFlash.table.configuration.OffsetAnglesAlign[0]) },
  {  OFFSET_ANGLES_ALIGN_FIELD_ID+1,  &gEepromInFlash.table.configuration.OffsetAnglesAlign[1]  ,  sizeof(gEepromInFlash.table.configuration.OffsetAnglesAlign[1]) },
  {  OFFSET_ANGLES_ALIGN_FIELD_ID+2,  &gEepromInFlash.table.configuration.OffsetAnglesAlign[2]  ,  sizeof(gEepromInFlash.table.configuration.OffsetAnglesAlign[2]) },
  {  HARD_IRON_BIAS_EXT_FIELD_ID,  &gEepromInFlash.table.configuration.hardIronBiasExt [0] ,  sizeof(gEepromInFlash.table.configuration.hardIronBiasExt[0]) },
  {  HARD_IRON_BIAS_EXT_FIELD_ID+1,  &gEepromInFlash.table.configuration.hardIronBiasExt [1] ,  sizeof(gEepromInFlash.table.configuration.hardIronBiasExt[1]) },
  {  SOFT_IRON_SCALE_RATIO_EXT_FIELD_ID,  &gEepromInFlash.table.configuration.softIronScaleRatioExt  ,  sizeof(gEepromInFlash.table.configuration.softIronScaleRatioExt) },
  {  SOFT_IRON_ANGLE_EXT_FIELD_ID,  &gEepromInFlash.table.configuration.softIronAngleExt  ,  sizeof(gEepromInFlash.table.configuration.softIronAngleExt) },
  {  PORT_1_USAGE_FIELD_ID,  &gEepromInFlash.table.configuration.port1Usage  ,  sizeof(gEepromInFlash.table.configuration.port1Usage) },
  {  PORT_2_USAGE_FIELD_ID,  &gEepromInFlash.table.configuration.port2Usage  ,  sizeof(gEepromInFlash.table.configuration.port2Usage) },
  {  PORT_3_USAGE_FIELD_ID,  &gEepromInFlash.table.configuration.port3Usage  ,  sizeof(gEepromInFlash.table.configuration.port3Usage) },
  {  PORT_4_USAGE_FIELD_ID,  &gEepromInFlash.table.configuration.port4Usage  ,  sizeof(gEepromInFlash.table.configuration.port4Usage) },
  {  PORT_1_BAUD_RATE_FIELD_ID,  &gEepromInFlash.table.configuration.port1BaudRate  ,  sizeof(gEepromInFlash.table.configuration.port1BaudRate) },
  {  PORT_2_BAUD_RATE_FIELD_ID,  &gEepromInFlash.table.configuration.port2BaudRate  ,  sizeof(gEepromInFlash.table.configuration.port2BaudRate) },
  {  PORT_3_BAUD_RATE_FIELD_ID,  &gEepromInFlash.table.configuration.port3BaudRate  ,  sizeof(gEepromInFlash.table.configuration.port3BaudRate) },
  {  PORT_4_BAUD_RATE_FIELD_ID,  &gEepromInFlash.table.configuration.port4BaudRate  ,  sizeof(gEepromInFlash.table.configuration.port4BaudRate) },
  {  REMOTE_MAG_SER_NO_FIELD_ID,  &gEepromInFlash.table.configuration.remoteMagSerNo[0]  ,  sizeof(gEepromInFlash.table.configuration.remoteMagSerNo) },
  
  
  {  CAL_SERIAL_NUMBER,  &gEepromInFlash.table.calibration.serialNumber  ,  sizeof(gEepromInFlash.table.calibration.serialNumber) },
  {  CAL_VERSION,  &gEepromInFlash.table.calibration.versionString[0]  ,  sizeof(gEepromInFlash.table.calibration.versionString) },
  {  CAL_INDEXA,  &gEepromInFlash.table.calibration.calibrationTableIndexA[0]  ,  sizeof(gEepromInFlash.table.calibration.calibrationTableIndexA) },
  {  CAL_TABLEA,  &gEepromInFlash.table.calibration.calibrationTablesA[0][0]  ,  sizeof(gEepromInFlash.table.calibration.calibrationTablesA) },
  {  CAL_MISALIGN,  &gEepromInFlash.table.calibration.misalign[0]  ,  sizeof(gEepromInFlash.table.calibration.misalign) },
  {  PROD_CONFIG,  &gEepromInFlash.table.calibration.productConfiguration  ,  sizeof(gEepromInFlash.table.calibration.productConfiguration) },
  {  CAL_INDEXB,  &gEepromInFlash.table.calibration.calibrationTableIndexB[0]  ,  sizeof(gEepromInFlash.table.calibration.calibrationTableIndexB) },
  {  CAL_TABLEB,  &gEepromInFlash.table.calibration.calibrationTablesB[0][0]  ,  sizeof(gEepromInFlash.table.calibration.calibrationTablesB) },
  {  CAL_INDEXC,  &gEepromInFlash.table.calibration.calibrationTableIndexC[0]  ,  sizeof(gEepromInFlash.table.calibration.calibrationTableIndexC) },
  {  CAL_TABLEC,  &gEepromInFlash.table.calibration.calibrationTablesC[0][0]  ,  sizeof(gEepromInFlash.table.calibration.calibrationTablesC) },
  {  CAL_GSENS,  &gEepromInFlash.table.calibration.gSenForGyros[0]  ,  sizeof(gEepromInFlash.table.calibration.gSenForGyros) },
  {  ACCEL_MAX,  &gEepromInFlash.table.calibration.AccelSensorRange  ,  sizeof(gEepromInFlash.table.calibration.AccelSensorRange) },
  {  GYRO_MAX,  &gEepromInFlash.table.calibration.GyroSensorRange  ,  sizeof(gEepromInFlash.table.calibration.GyroSensorRange) },
  {  MAG_MAX,  &gEepromInFlash.table.calibration.MagSensorRange  ,  sizeof(gEepromInFlash.table.calibration.MagSensorRange) },

	{	EEPROM_BITLIMITS_16bit +	0	,	&gEepromInFlash.table.calibration.bitLimits[	0	],	2*sizeof(gEepromInFlash.table.calibration.bitLimits[0])	},
	{	EEPROM_BITLIMITS_16bit +	2	,	&gEepromInFlash.table.calibration.bitLimits[	2	],	2*sizeof(gEepromInFlash.table.calibration.bitLimits[0])	},
	{	EEPROM_BITLIMITS_16bit +	4	,	&gEepromInFlash.table.calibration.bitLimits[	4	],	2*sizeof(gEepromInFlash.table.calibration.bitLimits[0])	},
	{	EEPROM_BITLIMITS_16bit +	6	,	&gEepromInFlash.table.calibration.bitLimits[	6	],	2*sizeof(gEepromInFlash.table.calibration.bitLimits[0])	},
	{	EEPROM_BITLIMITS_16bit +	8	,	&gEepromInFlash.table.calibration.bitLimits[	8	],	2*sizeof(gEepromInFlash.table.calibration.bitLimits[0])	},
	{	EEPROM_BITLIMITS_16bit +	10	,	&gEepromInFlash.table.calibration.bitLimits[	10	],	2*sizeof(gEepromInFlash.table.calibration.bitLimits[0])	},
	{	EEPROM_BITLIMITS_16bit +	12	,	&gEepromInFlash.table.calibration.bitLimits[	12	],	2*sizeof(gEepromInFlash.table.calibration.bitLimits[0])	},
	{	EEPROM_BITLIMITS_16bit +	14	,	&gEepromInFlash.table.calibration.bitLimits[	14	],	2*sizeof(gEepromInFlash.table.calibration.bitLimits[0])	},
	{	EEPROM_BITLIMITS_16bit +	16	,	&gEepromInFlash.table.calibration.bitLimits[	16	],	2*sizeof(gEepromInFlash.table.calibration.bitLimits[0])	},
	{	EEPROM_BITLIMITS_16bit +	18	,	&gEepromInFlash.table.calibration.bitLimits[	18	],	2*sizeof(gEepromInFlash.table.calibration.bitLimits[0])	},
	{	EEPROM_BITLIMITS_16bit +	20	,	&gEepromInFlash.table.calibration.bitLimits[	20	],	2*sizeof(gEepromInFlash.table.calibration.bitLimits[0])	},
	{	EEPROM_BITLIMITS_16bit +	22	,	&gEepromInFlash.table.calibration.bitLimits[	22	],	2*sizeof(gEepromInFlash.table.calibration.bitLimits[0])	},
									
	{	EEPROM_BITLIMITS_32bit +	0	,	&gEepromInFlash.table.calibration.bitLimits32[	0	],	2*sizeof(gEepromInFlash.table.calibration.bitLimits32[0])	},
	{	EEPROM_BITLIMITS_32bit +	4	,	&gEepromInFlash.table.calibration.bitLimits32[	2	],	2*sizeof(gEepromInFlash.table.calibration.bitLimits32[0])	},
	{	EEPROM_BITLIMITS_32bit +	8	,	&gEepromInFlash.table.calibration.bitLimits32[	4	],	2*sizeof(gEepromInFlash.table.calibration.bitLimits32[0])	},
	{	EEPROM_BITLIMITS_32bit +	12	,	&gEepromInFlash.table.calibration.bitLimits32[	6	],	2*sizeof(gEepromInFlash.table.calibration.bitLimits32[0])	},
	{	EEPROM_BITLIMITS_32bit +	16	,	&gEepromInFlash.table.calibration.bitLimits32[	8	],	2*sizeof(gEepromInFlash.table.calibration.bitLimits32[0])	},
	{	EEPROM_BITLIMITS_32bit +	20	,	&gEepromInFlash.table.calibration.bitLimits32[	10	],	2*sizeof(gEepromInFlash.table.calibration.bitLimits32[0])	},
	{	EEPROM_BITLIMITS_32bit +	24	,	&gEepromInFlash.table.calibration.bitLimits32[	12	],	2*sizeof(gEepromInFlash.table.calibration.bitLimits32[0])	},


{  ROLL_INCIDENCE_LIMIT,  &gEepromInFlash.table.calibration.RollIncidenceLimit  ,  2*sizeof(gEepromInFlash.table.calibration.RollIncidenceLimit) },
//  {  PITCH_INCIDENCE_LIMIT,  &gEepromInFlash.table.calibration.PitchIncidenceLimit  ,  sizeof(gEepromInFlash.table.calibration.PitchIncidenceLimit) },
  {  HARD_IRON_LIMIT,  &gEepromInFlash.table.calibration.HardIronLimit  , 2* sizeof(gEepromInFlash.table.calibration.HardIronLimit) },
 // {  SOFT_IRON_LIMIT,  &gEepromInFlash.table.calibration.SoftIronLimit  ,  sizeof(gEepromInFlash.table.calibration.SoftIronLimit) },
  {  PRODUCT_CONFIGURATION_FIELD_ID,  &gEepromInFlash.table.calibration.productConfiguration.all  ,  sizeof(gEepromInFlash.table.calibration.productConfiguration.all) },
};
#define SIZEOF_TRANSLATION_TABLE (sizeof(gEepromTranslation) / sizeof(struct sEepromTranslation))

#endif /* S_EEPROM_CONV_H */ 


