/*
 * pid_control.h - Flight control PID controller module
 */

#ifndef PID_CONTROL_H_
#define PID_CONTROL_H_

#include <zephyr/kernel.h>

#if IS_ENABLED(CONFIG_CMSIS_DSP)
#include <arm_math.h>
typedef arm_pid_instance_f32 pid_controller_t;

static inline float pid_update(pid_controller_t *pid, float target, float current, float dt)
{
    return arm_pid_f32(pid, target - current);
}

static inline void pid_init(pid_controller_t *pid, float kp, float ki, float kd, float dt, float output_limit)
{
    pid->Kp = kp;
    pid->Ki = ki * dt;
    pid->Kd = kd / dt;
    arm_pid_init_f32(pid, 1);
}
#else
typedef struct {
    float kp, ki, kd;
    float integrator;
    float last_error;
    float output_limit;
} pid_controller_t;

void pid_init(pid_controller_t *pid, float kp, float ki, float kd, float dt, float output_limit);
float pid_update(pid_controller_t *pid, float target, float current, float dt);
#endif

#endif /* PID_CONTROL_H_ */
