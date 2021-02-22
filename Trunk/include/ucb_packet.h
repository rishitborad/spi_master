/*********************************************************************************
* File name:  ucb_packet.h
*
* File description: 
*   - UCB packet structure
*               
* $Rev: 15924 $
* $Date: 2010-08-03 10:20:52 -0700 (Tue, 03 Aug 2010) $
* $Author: by-rhilles $
*
*********************************************************************************/            

#ifndef UCB_PACKET_H
#define UCB_PACKET_H  
#include "crc.h"

#define UCB_SYNC_LENGTH				2     
#define UCB_PACKET_TYPE_LENGTH		2
#define UCB_PAYLOAD_LENGTH_LENGTH	1
#define UCB_MAX_PAYLOAD_LENGTH		255
#define UCB_CRC_LENGTH				CRC_CCITT_LENGTH  	

#define UCB_SYNC_INDEX				0
#define UCB_PACKET_TYPE_INDEX		(UCB_SYNC_INDEX + UCB_SYNC_LENGTH)
#define UCB_PAYLOAD_LENGTH_INDEX    (UCB_PACKET_TYPE_INDEX + UCB_PACKET_TYPE_LENGTH)  
  
/* preamble sync bytes */ 
static const uint8_t UCB_SYNC [UCB_SYNC_LENGTH] = { 0x55, 0x55 };  
  
/* packet field type definitions */
typedef uint16_t			UcbPacketCodeType; 
typedef uint8_t  			UcbPacketPayloadLengthType;
typedef CrcCcittType 	UcbPacketCrcType; 
 
typedef enum {
#	define ALL_PACKET_TYPES 
#	define PACKET_TYPE(constName, varName, code)	constName,
#	include "ucb_packet_types.def" 
#	undef ALL_PACKET_TYPES
#	undef PACKET_TYPE  
	
	NUM_UCB_PACKET_TYPE_ENUM
} UcbPacketTypeEnum;        
 
typedef struct {
     UcbPacketTypeEnum			packetType;                      
     UcbPacketPayloadLengthType	payloadLength;
     uint8_t               		payload [UCB_MAX_PAYLOAD_LENGTH];
} UcbPacketStruct;
 
/* UCB packet-specific utility functions */ 
extern UcbPacketTypeEnum   			UcbPacketBytesToPacketType      (const uint8_t bytes []); 
extern void				UcbPacketPacketTypeToBytes	(UcbPacketTypeEnum type, uint8_t bytes []);  
extern UcbPacketPayloadLengthType  	UcbPacketBytesToPayloadLength   (const uint8_t bytes []); 
extern void				UcbPacketPayloadLengthToBytes   (UcbPacketPayloadLengthType type, uint8_t bytes []);
extern UcbPacketCrcType			UcbPacketBytesToCrc		(const uint8_t bytes []);
extern void				UcbPacketCrcToBytes		(const UcbPacketCrcType crc, uint8_t bytes []);
extern UcbPacketCrcType			UcbPacketCalculateCrc		(const uint8_t data [], uint16_t length, const UcbPacketCrcType seed); 
extern BOOL					UcbPacketIsAnInputPacket        (UcbPacketTypeEnum type); 
extern BOOL					UcbPacketIsAnOutputPacket       (UcbPacketTypeEnum type);

#endif
