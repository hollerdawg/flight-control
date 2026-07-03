/*
 * pid_control.c - Flight control PID controller module
 */

#include "pid_control.h"

#if !IS_ENABLED(CONFIG_CMSIS_DSP)
float pid_update(pid_controller_t *pid, float target, float current, float dt)
{
    float error = target - current;
    pid->integrator += error * dt;
    float derivative = (error - pid->last_error) / dt;
    pid->last_error = error;
    return (pid->kp * error) + (pid->ki * pid->integrator) + (pid->kd * derivative);
}
#endif
