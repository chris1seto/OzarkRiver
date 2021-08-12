#include <math.h>
#include <stdint.h>
#include <stdbool.h>
#include "Pid.h"
#include "Ticks.h"
#include "MathX.h"

void Pid_Reset(Pid_t* const inst)
{
  inst->integrator = 0;
  inst->last_process = 0;
  inst->last_run_time = 0;
}

void Pid_SetTuning(Pid_t* const inst, const TickType_t loop_period, const float kp, const float ki, const float kd)
{
  inst->loop_period = loop_period;
  inst->kp = kp;
  inst->ki = ki;
  inst->kd = kd;
}

void Pid_SetOutputLimit(Pid_t* const inst, const float min, const float max)
{
  inst->output_limit_min = min;
  inst->output_limit_max = max;
}

void Pid_SetIntegratorError(Pid_t* const inst, const float min, const float max)
{
  inst->integrator_limit_min = min;
  inst->integrator_limit_max = max;
}

float Pid_Calculate(Pid_t* const inst, const float setpoint, const float process)
{
  float p;
  float i;
  float d;
  float error;
  float dt_correction;
  float output;

  // Calculate dT in mS
  dt_correction = (Ticks_Diff(Ticks_Now(), inst->last_run_time) / inst->loop_period);
  
  if (dt_correction > inst->max_loop_period_factor)
  {
    dt_correction = 1;
  }

  // Error
  error = (setpoint - process);

  // Integrator
  inst->integrator += (error * dt_correction);

  // Constrain integrator
  inst->integrator = MathX_ConstrainF(inst->integrator,
    inst->integrator_limit_min,
    inst->integrator_limit_max);

  // Proportional
  p = (inst->kp * error);

  // Integral
  i = (inst->ki * inst->integrator);

  // Derivative
  d = inst->kd * ((process - inst->last_process) / dt_correction);
  inst->last_process = process;

  // Sum terms
  output = p + i + d;

  // Constrain output
  output = MathX_ConstrainF(output,
    inst->output_limit_min,
    inst->output_limit_max);

  inst->last_run_time = Ticks_Now();

  return output;
}
