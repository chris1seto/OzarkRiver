#ifndef TICKS_H
#define TICKS_H

#include "FreeRTOS.h"
#include "task.h"

bool Ticks_IsExpired(const TickType_t ticksStart, const TickType_t timeout);
bool Ticks_OccuredWithin(const TickType_t x, const TickType_t y, const TickType_t timeout);
void Ticks_Reset(TickType_t* const timer);
TickType_t Ticks_Diff(const TickType_t x, const TickType_t y);
TickType_t Ticks_Now(void);

#endif
