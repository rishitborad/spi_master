/*********************************************************************************
* File name:  packet.h
*
* File description: 
*   - packet structure
*               
*********************************************************************************/            

#ifndef PACKET_H
#define PACKET_H             

typedef union {
	UcbPacketStruct *ucbPacketPtr;
	CrmPacketStruct *crmPacketPtr;
} PacketPtrType;  

#endif
