
#ifndef 	_UTIL_H_
#define		_UTIL_H_

#include <stdint.h>
#include <stdlib.h>
#include <math.h>

typedef struct
{
	uint16_t head;
	uint16_t tail;
	uint16_t size;
	uint16_t capacity;
	int16_t	 *buf;
} CircleBuffer;

// bit circle buffer

typedef struct 
{
	uint8_t bit_head;
	uint8_t bit_tail;
	uint8_t bit_size;
	uint8_t bit_capacity;
	uint8_t	 *bit_buf;
} BitCircleBuffer;
//-------------------------------------------------------------------

CircleBuffer * CB_Create(uint16_t capacity);
uint16_t CB_GetSize(CircleBuffer * c);
void CB_Flush(CircleBuffer *c);
void CB_PlaceTail(CircleBuffer * c , int16_t value);
int16_t CB_GetHead(CircleBuffer *c);
int16_t CB_GetRMS(CircleBuffer * c);
int16_t CB_GetAVG(CircleBuffer * c);
//-------------------------------------------------------------------
// bit operation
BitCircleBuffer *  BitCB_Create(uint8_t bit_capacity);
void BitCB_Flush(BitCircleBuffer *c);
void BitCB_PlaceTail(BitCircleBuffer* c, uint8_t bit_value);
uint8_t BitCB_GetOnes(BitCircleBuffer* c);



#endif
