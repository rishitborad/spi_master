/*********************************************************************************
* File name:  xbowsp_fields.h
*
* File description:
*   - functions for handling DMU packets
*
* $Rev: 17469 $
* $Date: 2011-02-09 20:58:18 -0800 (Wed, 09 Feb 2011) $
* $Author: by-denglish $
*
*********************************************************************************/
#if ! defined FIELDS_H
#define FIELDS_H
#include "dmu.h"
#include "xbowsp_generaldrivers.h"
#include "ucb_packet.h"

extern BOOL   	CheckPortUsage 			  (ConfigurationStruct *proposedConfiguration) ;
extern BOOL   	CheckPortBaudRate 		  (uint16_t portBaudRate) ;
extern BOOL   	CheckPacketRateDivider	  (uint16_t packetRateDivider) ;
extern BOOL		CheckContPacketRate (UcbPacketTypeEnum outputPacket, uint16_t baudRate, uint16_t packetRateDivider) ;
extern uint8_t	CheckRamFieldData 		  (uint8_t numFields, uint16_t fieldId [], uint16_t fieldData [], uint16_t validFields []) ;
extern uint8_t	CheckEepromFieldData 	  (uint8_t numFields, uint16_t fieldId [], uint16_t fieldData [], uint16_t validFields []) ;
extern void		SetFieldData			  (void) ;

#define CONFIGURATION_START                 0x0000
#define CRC_ID                              0x0000
#define LOWER_CONFIG_ADDR_BOUND				0x0001  /* lower configuration address boundary */
#define PACKET_RATE_DIVIDER_FIELD_ID  		0x0001	/* continuous packet rate divider */
#define BAUD_RATE_USER_ID             		0x0002	/* continuous packet rate divider */
#define PACKET_TYPE_FIELD_ID  				0x0003	/* continuous packet type */
#define ORIENTATION_FIELD_ID  				0x0007	/* user defined axis orientation */
#define USER_BEHAVIOR_FIELD_ID				0x0008	/* user behavior switches */
#define TURN_SWITCH_THRESHOLD_FIELD_ID		0x000D	/* turn switch threshold  */
#define HARDWARE_STATUS_ENABLE_FIELD_ID 	0x0010	/* hardware status enable */
#define COM_STATUS_ENABLE_FIELD_ID  		0x0011	/* communication status enable */
#define SOFTWARE_STATUS_ENABLE_FIELD_ID	 	0x0012	/* software status enable */
#define SENSOR_STATUS_ENABLE_FIELD_ID  		0x0013	/* sensor status enable */
#define BARO_CORRECTION_FIELD_ID  			0x0016	/* barometer correction */
#define OFFSET_ANGLES_EXT_MAG_FIELD_ID		0x0017	/* external mag roll offset */
#define OFFSET_ANGLES_ALIGN_FIELD_ID  		0x0019	/* roll offset alignment */
#define HARD_IRON_BIAS_EXT_FIELD_ID  		0x001C	/* external mag X hard iron bias */
#define SOFT_IRON_SCALE_RATIO_EXT_FIELD_ID  0x001E	/* external mag soft iron scale ratio */
#define SOFT_IRON_ANGLE_EXT_FIELD_ID  		0x001F	/* external mag soft iron angle */
#define PORT_1_USAGE_FIELD_ID  		    	0x0021
#define PORT_2_USAGE_FIELD_ID               0x0022
#define PORT_3_USAGE_FIELD_ID               0x0023
#define PORT_4_USAGE_FIELD_ID               0x0024
#define PORT_1_BAUD_RATE_FIELD_ID           0x0025
#define PORT_2_BAUD_RATE_FIELD_ID           0x0026
#define PORT_3_BAUD_RATE_FIELD_ID           0x0027
#define PORT_4_BAUD_RATE_FIELD_ID           0x0028
#define REMOTE_MAG_SER_NO_FIELD_ID  		0x0029
#define REMOTE_MAG_SER_NO_FIELD_ID_1  		0x002A
#define UPPER_CONFIG_ADDR_BOUND				0x002B	/* upper coniguration address boundary */

#define PRODUCT_CONFIGURATION_FIELD_ID		0x071C	/* outside of configuration, but needs to be read as a field */

#define NUM_CONFIG_FIELDS					(UPPER_CONFIG_ADDR_BOUND - LOWER_CONFIG_ADDR_BOUND - 2)
#define BYTES_PER_CONFIG_FIELD				2

extern BOOL CheckOrientation (uint16_t orientation) ;
extern void DefaultPortConfiguration (void);
extern BOOL CheckBaroCorrection(int32_t baroCorrection) ;
extern BOOL WriteFieldData (void);

extern uint16_t appendAttitudeTrue (char *response, uint16_t index) ;
extern uint16_t appendCorrectedRates (char *response, uint16_t index)  ;
extern uint16_t appendAccels (char *response, uint16_t index) ;
extern uint16_t appendTangentRates (char *response, uint16_t index) ;
extern uint16_t appendTangentAccels (char *response, uint16_t index) ;
extern uint16_t appendRates (char *response, uint16_t index);
extern uint16_t appendMagReadings (char *response, uint16_t index);
extern uint16_t appendTemps (char *response, uint16_t index) ;
extern uint16_t appendInertialCounts (char *response, uint16_t index) ;
extern uint16_t appendMagnetometerCounts (char *response, uint16_t index);
extern uint16_t appendAllTempCounts (char *response, uint16_t index) ;

#endif