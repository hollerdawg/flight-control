/*
 * pid_control.c - Flight control PID controller module
 */

#include "pid_control.h"

#if !IS_ENABLED(CONFIG_CMSIS_DSP)
void pid_init(pid_controller_t *pid, float kp, float ki, float kd, float dt)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->integrator = 0.0f;
    pid->last_error = 0.0f;
}

float pid_update(pid_controller_t *pid, float target, float current, float dt)
{
    float error = target - current;
    pid->integrator += error * dt;
    float derivative = (error - pid->last_error) / dt;
    pid->last_error = error;
    return (pid->kp * error) + (pid->ki * pid->integrator) + (pid->kd * derivative);
}
#endif
