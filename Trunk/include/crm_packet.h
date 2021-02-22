/*********************************************************************************
* File name:  crm_packet.h
*
* File description: 
*   - CRM packet structure
*               
*********************************************************************************/            

#ifndef CRM_PACKET_H
#define CRM_PACKET_H      

#define CRM_SYNC_LENGTH			2     
#define CRM_X_ACCEL_LENGTH		2
#define CRM_Y_ACCEL_LENGTH		2
#define CRM_Z_ACCEL_LENGTH		2
#define CRM_X_MAG_LENGTH		2
#define CRM_Y_MAG_LENGTH		2
#define CRM_Z_MAG_LENGTH		2	
#define CRM_MODEL_NO_LENGTH		2
#define CRM_BIT_LENGTH			2
#define CRM_CHECKSUM_LENGTH		2  	     
							
#define CRM_SYNC_INDEX			0
#define CRM_X_ACCEL_INDEX		(CRM_SYNC_INDEX + CRM_SYNC_LENGTH)
#define CRM_Y_ACCEL_INDEX  		(CRM_X_ACCEL_INDEX + CRM_X_ACCEL_LENGTH)
#define CRM_Z_ACCEL_INDEX		(CRM_Y_ACCEL_INDEX + CRM_Y_ACCEL_LENGTH)
#define CRM_X_MAG_INDEX			(CRM_Z_ACCEL_INDEX + CRM_Z_ACCEL_LENGTH)
#define CRM_Y_MAG_INDEX  		(CRM_X_MAG_INDEX + CRM_X_MAG_LENGTH)
#define CRM_Z_MAG_INDEX			(CRM_Y_MAG_INDEX + CRM_Y_MAG_LENGTH)
#define CRM_MODEL_NO_INDEX		(CRM_Z_MAG_INDEX + CRM_Z_MAG_LENGTH)
#define CRM_BIT_INDEX			(CRM_MODEL_NO_INDEX + CRM_MODEL_NO_LENGTH)
#define CRM_CHECKSUM_INDEX		(CRM_BIT_INDEX + CRM_BIT_LENGTH)
  
/* preamble sync bytes */ 
static const uint8_t CRM_SYNC [CRM_SYNC_LENGTH] = { 0xAA, 0x55 };  

typedef enum {  
#	define PACKET_TYPE(constName, varName)	constName,
#	include "crm_packet_types.def" 
#	undef PACKET_TYPE   
	
	NUM_CRM_PACKET_TYPE_ENUM
} CrmPacketTypeEnum;
  
/* packet field type definitions */
typedef int16_t		CrmPacketAccelType;
typedef int16_t		CrmPacketMagType;
typedef uint16_t	CrmPacketModelNoType;
typedef uint16_t	CrmPacketBitType;
typedef uint16_t	CrmPacketChecksumType;
 
typedef struct {
	CrmPacketTypeEnum		packetType;
	CrmPacketAccelType		xAccel; 
	CrmPacketAccelType		yAccel;
	CrmPacketAccelType		zAccel;
	CrmPacketMagType		xMag; 
	CrmPacketMagType		yMag;    
	CrmPacketMagType		zMag;	
	CrmPacketModelNoType	modelNo; 
	CrmPacketBitType		bit;
} CrmPacketStruct;
 
/* CRM packet-specific utility functions */   
CrmPacketAccelType 		CrmPacketBytesToAccel      (const uint8_t bytes []);
CrmPacketMagType 		CrmPacketBytesToMag        (const uint8_t bytes []);
CrmPacketModelNoType 	CrmPacketBytesToModelNo    (const uint8_t bytes []);
CrmPacketBitType 		CrmPacketBytesToBit        (const uint8_t bytes []);
CrmPacketChecksumType 	CrmPacketBytesToChecksum   (const uint8_t bytes []);
CrmPacketChecksumType 	CrmPacketCalculateChecksum (const uint8_t data [], uint16_t length, const CrmPacketChecksumType seed);

#endif

