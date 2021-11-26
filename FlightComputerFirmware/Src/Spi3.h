#ifndef SPI3_H
#define SPI3_H

enum SPI3_TARGET
{
  SPI3_TARGET_SD,
};

void Spi3_Init(void);
bool Spi3_Transaction(const enum SPI3_TARGET target, const uint8_t* tx, uint8_t* const rx, const uint32_t size);

#endif
