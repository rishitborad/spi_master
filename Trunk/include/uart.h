/******************************************************************************
* File name:	
*	uart.h
*
* File description:
*	uart head file
*
* $Revision: 15150 $
*
* $Date: 2010-12-07 10:43:16 +0800 (Tue, 07 Dec 2010) $
*
* $Author: jsun $
 ******************************************************************************/

#ifndef __UART_H
#define __UART_H

extern void uart_init(unsigned int uartChannel,uart_hw *port_hw);
extern void uart_read(unsigned int channel, port_struct *port);
extern void uart_write(unsigned int channel, port_struct *port);
extern unsigned int bytes_remaining(unsigned int channel, port_struct *port);

#endif
