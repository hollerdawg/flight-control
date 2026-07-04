/*
 * pid_control.c - Flight control PID controller module
 */

#include "pid_control.h"

#if !IS_ENABLED(CONFIG_CMSIS_DSP)
void pid_init(pid_controller_t *pid, float kp, float ki, float kd, float dt, float output_limit)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->integrator = 0.0f;
    pid->last_error = 0.0f;
    pid->output_limit = output_limit;
}

float pid_update(pid_controller_t *pid, float target, float current, float dt)
{
    float error = target - current;
    pid->integrator += error * dt;

    if (pid->output_limit > 0.0f) {
        if (pid->integrator > pid->output_limit) {
            pid->integrator = pid->output_limit;
        } else if (pid->integrator < -pid->output_limit) {
            pid->integrator = -pid->output_limit;
        }
    }

    float derivative = (error - pid->last_error) / dt;
    pid->last_error = error;

    float output = (pid->kp * error) + (pid->ki * pid->integrator) + (pid->kd * derivative);

    if (pid->output_limit > 0.0f) {
        if (output > pid->output_limit) {
            output = pid->output_limit;
        } else if (output < -pid->output_limit) {
            output = -pid->output_limit;
        }
    }

    return output;
}
#endif
