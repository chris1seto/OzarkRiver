#ifndef SERVOOUT_H
#define SERVOOUT_H

enum SERVOOUT_CHANNEL
{
  SERVOOUT_CHANNEL_1,
  SERVOOUT_CHANNEL_2,
  SERVOOUT_CHANNEL_3,
  SERVOOUT_CHANNEL_4,
  SERVOOUT_CHANNEL_5,
  SERVOOUT_CHANNEL_6,
  SERVOOUT_CHANNEL_7,
  SERVOOUT_CHANNEL_8,
};

void ServoOut_Init(void);
void ServoOut_Set(const enum SERVOOUT_CHANNEL channel, const float value);
void ServoOut_Stop(const enum SERVOOUT_CHANNEL channel);

#endif
