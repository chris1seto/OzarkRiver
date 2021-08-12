#ifndef PID_H
#define PID_H

#include <math.h>
#include <stdint.h>
#include <stdbool.h>
#include "FreeRTOS.h"
#include "task.h"

typedef struct
{
  float kp;
  float ki;
  float kd;
  float integrator_limit_min;
  float integrator_limit_max;
  float output_limit_min;
  float output_limit_max;
  float last_process;
  float integrator;
  TickType_t last_run_time;
  TickType_t loop_period;
  float max_loop_period_factor;
} Pid_t;

void Pid_Reset(Pid_t* const inst);
void Pid_SetTuning(Pid_t* const inst, const TickType_t loop_period, const float kp, const float ki, const float kd);
void Pid_SetOutputLimit(Pid_t* const inst, const float min, const float max);
void Pid_SetIntegratorError(Pid_t* const inst, const float min, const float max);
float Pid_Calculate(Pid_t* const inst, const float setpoint, const float process);

#endif
