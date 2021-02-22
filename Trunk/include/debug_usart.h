/*******************************************************************************
* File name: 
*		debug_usart.h
*
* Description:
*       usart specific routines to deal with debug port
*
* $Revision: 15935 $
*
*******************************************************************************/
#ifndef DEBUG_USART_H
#define DEBUG_USART_H

#include <stdint.h>

extern int DebugSerialPutChar (int c);
extern int IsDebugSerialIdle(); 

extern int DebugSerialReadLine(uint8_t *buf, uint32_t *index, uint32_t len);


extern void InitDebugSerialCommunication(void);

#endif /*DEBUG_USART_H */

