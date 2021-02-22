/** ***************************************************************************
 * @file   circbuf.c
 * @Author
 * @date   September, 2008
 * @brief  Copyright (c) 2013, 2014 All Rights Reserved.
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
 * software array based cicular buffer
 ******************************************************************************/
#include <stdint.h>
#include "circbuf.h"

/** ****************************************************************************
 * @name: CircBufInit
 * @brief: Initializes a circular buffer structure
 * @param [in] cb - pointer to circular buffer structure
 * @param [in] buf - a pointer to a pre-allocated buffer used to store the data
 * @param [in] buf_size - size of the pre-allocated buffer
 * @retval  TRUE if successful, FALSE otherwise
 ******************************************************************************/
tBoolean CircBufInit(tCircularBuffer  *cb,
                     uint8_t volatile *buf,
                     uint32_t         buf_size) {
	if (!cb || !buf || (0 == buf_size)) {
		return FALSE;
	}
	cb->data   = buf;
	cb->size   = buf_size;
	cb->rd_idx = 0;
	cb->wr_idx = 0;
	return TRUE;
}

/** ****************************************************************************
 * @name CircBufPut
 * @brief Puts a byte in the circular buffer, this only
 *        accesses rd_idx once and it only modifies wr_idx
 * @param [in] cb - pointer to circular buffer structure
 * @param [in] c - byte to add
 * Returned value:  None
 ******************************************************************************/
void CircBufPut(tCircularBuffer *cb,
                uint8_t         c)
{
    int32_t len = cb->rd_idx; /// atomic read to eliminate chance of
                              /// interrupts messing this up
    len -= cb->wr_idx;
    if (len < 0)
    { len += cb->size; }
	if (len < cb->size) {
		cb->data[cb->wr_idx++] = c;
		if (cb->wr_idx >= cb->size) {
			cb->wr_idx = 0;
		}
	}
}

/** ****************************************************************************
 * @name CircBufGet
 * @brief Gets a byte from the circular buffer,, this only
 *        accesses wr_idx once and it only modifies rd_idx
 * @param [in] cb - pointer to circular buffer structure
 * @retval c - Byte removed from circular buffer
 ******************************************************************************/
uint8_t CircBufGet(tCircularBuffer *cb)
{
	uint8_t c = 0;
    int32_t len = cb->wr_idx; /// atomic read to eliminate chance of
                              /// interrupts messing this up
    len -= cb->rd_idx;
    if (len < 0) { len += cb->size; }

	if (len > 0) {
		c = cb->data[cb->rd_idx++];
		if (cb->rd_idx >= cb->size) {
			cb->rd_idx = 0;
		}
	}
	return c;
}

/** ****************************************************************************
 * @name CircBufEmpty
 * @brief Returns TRUE if the circular buffer is empty
 * @param [in] cb - pointer to circular buffer structure
 * @retval  TRUE if the circular buffer is empty, FALSE otherwise
 ******************************************************************************/
tBoolean CircBufEmpty(tCircularBuffer *cb)
{
    uint32_t wr_idx = cb->wr_idx;
    uint32_t rd_idx =  cb->rd_idx;
	return (wr_idx == rd_idx);
}


