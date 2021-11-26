#ifndef SPI1_H
#define SPI1_H

enum SPI1_TARGET
{
  SPI1_TARGET_MPU6000,
};

void Spi1_Init(void);
bool Spi1_Transaction(const enum SPI1_TARGET target, const uint8_t* tx, uint8_t* const rx, const uint32_t size);

#endif
