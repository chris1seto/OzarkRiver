#ifndef ANALOG_H
#define ANALOG_H

typedef struct
{
  float bus_voltage;
  float bus_current;
  float mcu_temperature;
} AnalogStatus_t;

void Analog_Init(void);
bool Analog_GetStatus(AnalogStatus_t* const status);


#endif
