#ifndef QUEUEBUFFER_H
#define QUEUEBUFFER_H

#include <stdbool.h>

typedef struct
{
	uint8_t* buffer;
	uint32_t buffer_size;
	uint32_t count;
	uint32_t head;
	uint32_t tail;
} QueueBuffer_t;

void QueueBuffer_Init(QueueBuffer_t* const q, uint8_t* buffer, const uint32_t buffer_size);
uint32_t QueueBuffer_Count(const QueueBuffer_t* q);
void QueueBuffer_Append(QueueBuffer_t* const q, const uint8_t x);
bool QueueBuffer_AppendBuffer(QueueBuffer_t* const q, const uint8_t* x, const uint32_t append_size);
bool QueueBuffer_IsEmpty(const QueueBuffer_t* q);
bool QueueBuffer_Get(QueueBuffer_t* const q, uint8_t* const x);
void QueueBuffer_Dequeue(QueueBuffer_t* const q, const uint32_t n);
bool QueueBuffer_Peek(const QueueBuffer_t* q, const uint32_t index, uint8_t* const x);
bool QueueBuffer_PeekBuffer(const QueueBuffer_t* q, const uint32_t index, uint8_t* buffer, const uint32_t size);

#endif