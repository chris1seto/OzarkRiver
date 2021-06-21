#ifndef SPI2_H
#define SPI2_H

enum SPI2_TARGET
{
  SPI2_TARGET_WINBOND,
};

void Spi2_Init(void);
bool Spi2_Transaction(const enum SPI2_TARGET target, const uint8_t* tx, uint8_t* const rx, const uint32_t size);

#endif
