/*
 * mahony.c - Flight control Mahony 9-axis AHRS filter
 */

#include "mahony.h"
#include <math.h>

#if IS_ENABLED(CONFIG_CMSIS_DSP)
#include <arm_math.h>
#endif

float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
float eIntX = 0.0f, eIntY = 0.0f, eIntZ = 0.0f;

static inline float my_sqrt(float val)
{
#if IS_ENABLED(CONFIG_CMSIS_DSP)
    float res;
    arm_sqrt_f32(val, &res);
    return res;
#else
    return sqrtf(val);
#endif
}

void mahony_reset(void)
{
    q0 = 1.0f; q1 = 0.0f; q2 = 0.0f; q3 = 0.0f;
    eIntX = 0.0f; eIntY = 0.0f; eIntZ = 0.0f;
}

void mahony_update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float dt)
{
    float recipNorm;
    float vx, vy, vz;
    float ex = 0.0f, ey = 0.0f, ez = 0.0f;
    float qa, qb, qc;

    // Normalise accelerometer measurement
    float norm = my_sqrt(ax * ax + ay * ay + az * az);
    if (norm > 0.0f) {
        recipNorm = 1.0f / norm;
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // Estimated direction of gravity
        vx = 2.0f * (q1 * q3 - q0 * q2);
        vy = 2.0f * (q0 * q1 + q2 * q3);
        vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

        // Error is cross product between estimated direction and measured direction of gravity
        ex = (ay * vz - az * vy);
        ey = (az * vx - ax * vz);
        ez = (ax * vy - ay * vx);
    }

#if IS_ENABLED(CONFIG_MAGNETOMETER_ENABLED)
    // Normalise magnetometer measurement
    float mag_norm = my_sqrt(mx * mx + my * my + mz * mz);
    if (mag_norm > 0.0f) {
        recipNorm = 1.0f / mag_norm;
        mx *= recipNorm;
        my *= recipNorm;
        mz *= recipNorm;

        // Reference direction of Earth's magnetic field in body frame
        float hx = mx * (q0*q0 + q1*q1 - q2*q2 - q3*q3) + 2.0f * my * (q1*q2 - q0*q3) + 2.0f * mz * (q1*q3 + q0*q2);
        float hy = 2.0f * mx * (q1*q2 + q0*q3) + my * (q0*q0 - q1*q1 + q2*q2 - q3*q3) + 2.0f * mz * (q2*q3 - q0*q1);
        float bx = my_sqrt(hx * hx + hy * hy);
        float bz = 2.0f * mx * (q1*q3 - q0*q2) + 2.0f * my * (q2*q3 + q0*q1) + mz * (q0*q0 - q1*q1 - q2*q2 + q3*q3);

        // Estimated direction of magnetic field in body frame
        float wx = bx * (q0*q0 + q1*q1 - q2*q2 - q3*q3) + bz * 2.0f * (q1*q3 - q0*q2);
        float wy = bx * 2.0f * (q1*q2 - q0*q3) + bz * 2.0f * (q2*q3 + q0*q1);
        float wz = bx * 2.0f * (q1*q3 + q0*q2) + bz * (q0*q0 - q1*q1 - q2*q2 + q3*q3);

        // Error is cross product between estimated direction and measured direction of magnetic field
        ex += (my * wz - mz * wy);
        ey += (mz * wx - mx * wz);
        ez += (mx * wy - my * wx);
    }
#endif

    // Compute and apply integral feedback if enabled
    if (ex != 0.0f || ey != 0.0f || ez != 0.0f) {
        if (MAHONY_KI > 0.0f) {
            eIntX += ex * dt;
            eIntY += ey * dt;
            eIntZ += ez * dt;
            gx += MAHONY_KI * eIntX;
            gy += MAHONY_KI * eIntY;
            gz += MAHONY_KI * eIntZ;
        } else {
            eIntX = 0.0f;
            eIntY = 0.0f;
            eIntZ = 0.0f;
        }

        // Apply proportional feedback
        gx += MAHONY_KP * ex;
        gy += MAHONY_KP * ey;
        gz += MAHONY_KP * ez;
    }

    // Integrate rate of change of quaternion
    qa = q0;
    qb = q1;
    qc = q2;
    q0 += (-qb * gx - qc * gy - q3 * gz) * (0.5f * dt);
    q1 += (qa * gx + qc * gz - q3 * gy) * (0.5f * dt);
    q2 += (qa * gy - qb * gz + q3 * gx) * (0.5f * dt);
    q3 += (qa * gz + qb * gy - qc * gx) * (0.5f * dt);

    // Normalise quaternion
    norm = my_sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    if (norm > 0.0f) {
        recipNorm = 1.0f / norm;
        q0 *= recipNorm;
        q1 *= recipNorm;
        q2 *= recipNorm;
        q3 *= recipNorm;
    }
}

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif



void mahony_get_euler(float *roll, float *pitch)
{
    *roll  = atan2f(2.0f * (q0 * q1 + q2 * q3), q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3) * (180.0f / M_PI);
    float sinp = 2.0f * (q0 * q2 - q1 * q3);
    sinp = CLAMP(sinp, -1.0f, 1.0f);
    *pitch = asinf(sinp) * (180.0f / M_PI);
}
