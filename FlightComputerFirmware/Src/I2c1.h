#ifndef I2C1_H
#define I2C1_H

void I2c1_Init(void);
bool I2c1_WriteMemory8(const uint8_t dev_addr, const uint8_t reg_addr, const uint8_t* data, const uint32_t size);
bool I2c1_ReadMemory8(const uint8_t dev_addr, const uint8_t reg_addr, uint8_t* const data, const uint32_t size);
bool I2c1_Write(const uint8_t dev_addr, const uint8_t* data, const uint32_t size);
bool I2c1_Read(const uint8_t dev_addr, uint8_t* const data, const uint32_t size);

#endif
