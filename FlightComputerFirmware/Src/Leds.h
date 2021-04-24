#ifndef LEDS_H
#define LEDS_H

enum LED
{
  LED_RED = (1 << 0),
  LED_BLUE = (1 << 1),
};

void Leds_Init(void);
void Leds_On(const uint32_t leds);
void Leds_Off(const uint32_t leds);
void Leds_Toggle(const uint32_t leds, const bool enabled);

#endif