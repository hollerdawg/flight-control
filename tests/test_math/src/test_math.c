/*
 * test_math.c - Unit tests for PID control and Mahony filter
 */

#include <zephyr/ztest.h>
#include <math.h>

#include "pid_control.h"
#include "mahony.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/* Test Suite definition */
ZTEST_SUITE(math_tests, NULL, NULL, NULL, NULL, NULL);

ZTEST(math_tests, test_pid_control)
{
    pid_controller_t pid;
    pid_init(&pid, 1.0f, 0.5f, 0.1f, 1.0f, 100.0f);

    /* Step 1:
     * target = 10.0, current = 0.0 => error = 10.0, dt = 1.0s
     * Proportional = 10.0 * 1.0 = 10.0
     * Integral     = (0.0 + 10.0 * 1.0) * 0.5 = 5.0 (integrator = 10.0)
     * Derivative   = ((10.0 - 0.0) / 1.0) * 0.1 = 1.0 (last_error = 10.0)
     * Expected Output = 16.0
     */
    float out = pid_update(&pid, 10.0f, 0.0f, 1.0f);
    zassert_true(fabsf(out - 16.0f) < 1e-4f, "First PID step output calculation incorrect (got %f, expected 16.0)", (double)out);

    /* Step 2:
     * target = 10.0, current = 5.0 => error = 5.0, dt = 1.0s
     * Proportional = 5.0 * 1.0 = 5.0
     * Integral     = (10.0 + 5.0 * 1.0) * 0.5 = 7.5 (integrator = 15.0)
     * Derivative   = ((5.0 - 10.0) / 1.0) * 0.1 = -0.5 (last_error = 5.0)
     * Expected Output = 5.0 + 7.5 - 0.5 = 12.0
     */
    out = pid_update(&pid, 10.0f, 5.0f, 1.0f);
    zassert_true(fabsf(out - 12.0f) < 1e-4f, "Second PID step output calculation incorrect (got %f, expected 12.0)", (double)out);
}

/* 1b. PID Anti-Windup Unit Test */
ZTEST(math_tests, test_pid_anti_windup)
{
#if (defined(CONFIG_PID_RATE_MOTOR_LIMIT) && CONFIG_PID_RATE_MOTOR_LIMIT > 0) || \
    (defined(CONFIG_PID_ANGLE_RATE_LIMIT) && CONFIG_PID_ANGLE_RATE_LIMIT > 0) || \
    (defined(CONFIG_PID_ALTITUDE_THROTTLE_LIMIT) && CONFIG_PID_ALTITUDE_THROTTLE_LIMIT > 0)
    pid_controller_t pid;
    pid_init(&pid, 1.0f, 1.0f, 0.0f, 1.0f, 10.0f);

    /* Step 1: error = 20.0
     * Integrator accumulates 20.0 * 1.0 = 20.0.
     * It should be clamped to 10.0.
     * Expected Output: Proportional (20.0) + Integral (10.0) = 30.0.
     * Output should be clamped to 10.0.
     */
    float out = pid_update(&pid, 20.0f, 0.0f, 1.0f);
#if !IS_ENABLED(CONFIG_CMSIS_DSP)
    zassert_true(fabsf(pid.integrator - 10.0f) < 1e-4f, 
                 "Integrator not clamped (got %f, expected 10.0)", (double)pid.integrator);
#endif
    zassert_true(fabsf(out - 10.0f) < 1e-4f, 
                 "Output not clamped (got %f, expected 10.0)", (double)out);
#else
    ztest_test_skip();
#endif
}

/* 2. Mahony Stationary Test */
ZTEST(math_tests, test_mahony_stationary)
{
    mahony_reset();

    /* Gravity pointing straight down, no rotation */
    float gx = 0.0f, gy = 0.0f, gz = 0.0f;
    float ax = 0.0f, ay = 0.0f, az = 1.0f;

    /* Update multiple times */
    for (int i = 0; i < 50; i++) {
        mahony_update(gx, gy, gz, ax, ay, az, 0.0f, 0.0f, 0.0f, 0.01f);
    }

    float roll, pitch;
    mahony_get_euler(&roll, &pitch);

    zassert_true(fabsf(roll - 0.0f) < 1e-4f, "Stationary roll must be 0 (got %f)", (double)roll);
    zassert_true(fabsf(pitch - 0.0f) < 1e-4f, "Stationary pitch must be 0 (got %f)", (double)pitch);
}

/* 3. Mahony Tilt Test */
ZTEST(math_tests, test_mahony_tilt)
{
    mahony_reset();

    /* Accelerometer reads gravity vector tilted by 45 degrees around X-axis (Roll)
     * Vector: [0.0, sin(45), cos(45)] = [0.0, 0.707107, 0.707107]
     */
    float gx = 0.0f, gy = 0.0f, gz = 0.0f;
    float ax = 0.0f;
    float ay = sinf(45.0f * M_PI / 180.0f);
    float az = cosf(45.0f * M_PI / 180.0f);

    /* Run the update multiple times to let the filter converge to the tilt vector */
    for (int i = 0; i < 200; i++) {
        mahony_update(gx, gy, gz, ax, ay, az, 0.0f, 0.0f, 0.0f, 0.01f);
    }

    float roll, pitch;
    mahony_get_euler(&roll, &pitch);

    /* Roll should be 45 degrees (absolute value), Pitch should be 0 */
    printk("DEBUG: roll = %d, pitch = %d\n", (int)roll, (int)pitch);
    zassert_true(fabsf(fabsf(roll) - 45.0f) < 1.0f, "Tilted roll calculation incorrect (got %d, expected ±45)", (int)roll);
    zassert_true(fabsf(pitch - 0.0f) < 1.0f, "Tilted pitch must be 0 (got %d)", (int)pitch);
}

/* 4. Mahony Gyro Bias Correction (Integral Term) Test */
ZTEST(math_tests, test_mahony_gyro_bias)
{
    mahony_reset();

    /* Gravity straight down, but with a constant gyroscope drift bias on X axis (roll rate) */
    float gx_bias = 0.1f; /* rad/s gyro bias */
    float ax = 0.0f, ay = 0.0f, az = 1.0f;

    /* First check: without integral gain, the bias causes continuous angle drift */
    /* (We check integration behavior by simulating steps) */
    for (int i = 0; i < 100; i++) {
        mahony_update(gx_bias, 0.0f, 0.0f, ax, ay, az, 0.0f, 0.0f, 0.0f, 0.01f);
    }

    float roll, pitch;
    mahony_get_euler(&roll, &pitch);

    /* Due to Ki, eIntX should build up to counteract the bias and restore stability.
     * With Ki = 0.005 and dt = 0.01, convergence is slow but eIntX must be non-zero.
     */
    zassert_not_equal(eIntX, 0.0f, "Mahony integral error accumulator did not build up");
}

/* 5. Mahony 9-Axis Test */
ZTEST(math_tests, test_mahony_9axis)
{
    mahony_reset();

    /* Gravity pointing straight down, magnetic field pointing North-ish */
    float gx = 0.0f, gy = 0.0f, gz = 0.0f;
    float ax = 0.0f, ay = 0.0f, az = 1.0f;
    float mx = 0.5f, my = 0.0f, mz = -0.5f;

    for (int i = 0; i < 100; i++) {
        mahony_update(gx, gy, gz, ax, ay, az, mx, my, mz, 0.01f);
    }

    float roll, pitch;
    mahony_get_euler(&roll, &pitch);

    zassert_true(fabsf(roll - 0.0f) < 1e-3f, "9-Axis roll must be 0 (got %f)", (double)roll);
    zassert_true(fabsf(pitch - 0.0f) < 1e-3f, "9-Axis pitch must be 0 (got %f)", (double)pitch);
}
