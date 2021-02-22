/**********************************************************************************
* File name:  comm_buffers.h
*
* File description: - for the uart driver, for use by the system
*               
* $Rev: 16069 $
* $Date: 2010-08-18 10:05:16 -0700 (Wed, 18 Aug 2010) $
* $Author: by-lfera $
***********************************************************************************/

#ifndef COMM_BUFFERS_H
#define COMM_BUFFERS_H   

extern unsigned int COM_buffer_bytes (cir_buf *buf_struc);
extern unsigned int COM_buf_space (cir_buf *buf_struc);
extern unsigned int COM_buf_pop (cir_buf *buf_struc, unsigned int pop_cnt);       
extern BOOL COM_buf_read (cir_buf *buf_struc,unsigned int buf_index,unsigned int cnt,unsigned char *buf_out);
extern BOOL COM_buf_in(cir_buf *buf_struc,unsigned char *buf,unsigned int cnt);
extern BOOL COM_buf_out(cir_buf *buf_struc,unsigned char *buf,unsigned int cnt);
extern void COM_buf_init(port_struct *port,unsigned char *tx_buf,unsigned char *rx_buf,unsigned int rx_size,unsigned int tx_size);
 
#endif
