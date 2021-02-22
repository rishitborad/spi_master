/**********************************************************************************
* File name:  circ_buffers.h
*
* File description: - for the uart driver, for use by the system
*               
* $Rev: 16773 $
* $Date: 2010-10-08 11:08:05 -0700 (Fri, 08 Oct 2010) $
* $Author: by-tdelong $
***********************************************************************************/

#ifndef CIRC_BUFFERS_H
#define CIRC_BUFFERS_H   
#include "stdint.h"
#include "port_def.h"

extern unsigned int circ_buffer_bytes (cir_buf *buf_struc);
extern unsigned int circ_buf_space (cir_buf *buf_struc);
extern unsigned int circ_buf_pop (cir_buf *buf_struc, unsigned int pop_cnt);       
extern bool circ_buf_read (cir_buf *buf_struc,unsigned int buf_index,unsigned int cnt,unsigned char *buf_out);
extern bool circ_buf_in(cir_buf *buf_struc,unsigned char *buf,unsigned int cnt);
extern bool circ_buf_out(cir_buf *buf_struc,unsigned char *buf,unsigned int cnt);
extern void circ_buf_init(port_struct *port,uint8_t *tx_buf,uint8_t *rx_buf,uint32_t rx_size,uint32_t tx_size);
 
#endif
