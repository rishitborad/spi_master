/*
 * circbuf.h
 * Circular buffer implementatinos
 */

#ifndef CIRCBUF_H_
#define CIRCBUF_H_

#include <stdint.h>
#include "dmu.h" // for tBoolean, TRUE and FALSE

typedef struct {
	uint32_t volatile rd_idx;
	uint32_t volatile wr_idx;
	uint8_t volatile *data;
	uint32_t size;
} tCircularBuffer;

tBoolean CircBufInit(tCircularBuffer *cb, uint8_t volatile *buf, uint32_t buf_size);

void CircBufPut(tCircularBuffer *cb, uint8_t c);
uint8_t CircBufGet(tCircularBuffer *cb);
tBoolean CircBufEmpty(tCircularBuffer *cb);

#endif /* CIRCBUF_H_ */
