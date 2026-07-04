/*
 * mahony.h - Flight control Mahony 9-axis AHRS filter
 */

#ifndef MAHONY_H_
#define MAHONY_H_

#include <zephyr/kernel.h>

#define MAHONY_KP 2.0f
#define MAHONY_KI 0.005f

extern float q0, q1, q2, q3;
extern float eIntX, eIntY, eIntZ;

void mahony_update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float dt);
void mahony_get_euler(float *roll, float *pitch);
void mahony_reset(void);

#endif /* MAHONY_H_ */
