#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/logging/log.h>
#include <math.h>

#include "mahony.h"
#include "pid_control.h"

#if IS_ENABLED(CONFIG_CMSIS_DSP)
#include <arm_math.h>
#endif



#if IS_ENABLED(CONFIG_FLIGHT_CTL_RC_IBUS)
#include "ibus.h"
#endif

LOG_MODULE_REGISTER(flight_blackbox, CONFIG_LOG_DEFAULT_LEVEL);

#define FLIGHT_LOOP_PRIORITY   -1  /* High-priority cooperative thread */
#define FLIGHT_LOOP_PERIOD_MS  2.5f
#define DT_SEC                 0.0025f

#define MOTOR_MIN_PULSE_US     1000
#define MOTOR_MAX_PULSE_US     2000
#define BASE_HOVER_THROTTLE    1450

#define DESCENT_RATE_MPS       0.2f
#define LANDING_THRESHOLD_M    0.1f  

typedef enum {
    STATE_DISARMED,
    STATE_MANUAL_FLYING,
    STATE_ALT_HOLD,
    STATE_LANDING
} flight_state_t;

static flight_state_t current_state = STATE_MANUAL_FLYING;

#if IS_ENABLED(CONFIG_CMSIS_DSP)
static pid_controller_t roll_pid;
static pid_controller_t pitch_pid;
static pid_controller_t yaw_pid;
static pid_controller_t angle_pid_roll;
static pid_controller_t angle_pid_pitch;
#else
static pid_controller_t roll_pid  = { .kp = 1.2f, .ki = 0.05f, .kd = 0.12f };
static pid_controller_t pitch_pid = { .kp = 1.2f, .ki = 0.05f, .kd = 0.12f };
static pid_controller_t yaw_pid   = { .kp = 2.0f, .ki = 0.10f, .kd = 0.05f };

static pid_controller_t angle_pid_roll  = { .kp = 4.0f, .ki = 0.05f, .kd = 0.1f };
static pid_controller_t angle_pid_pitch = { .kp = 4.0f, .ki = 0.05f, .kd = 0.1f };
#endif

typedef enum {
    FLIGHT_MODE_ACRO,
    FLIGHT_MODE_ANGLE,
    FLIGHT_MODE_HORIZON
} flight_mode_t;

static flight_mode_t current_flight_mode = FLIGHT_MODE_ACRO;
static float est_roll  = 0.0f;
static float est_pitch = 0.0f;

#if IS_ENABLED(CONFIG_FLIGHT_CTL_HAS_BARO)
#if IS_ENABLED(CONFIG_CMSIS_DSP)
static pid_controller_t alt_pid;
#else
static pid_controller_t alt_pid   = { .kp = 2.5f, .ki = 0.15f, .kd = 0.45f };
#endif
static float target_altitude      = 1.5f; 
static float current_alt          = 0.0f;
#endif

static float target_throttle = (float)MOTOR_MIN_PULSE_US;
static float target_roll     = 0.0f;
static float target_pitch    = 0.0f;
static float target_yaw      = 0.0f;

static const struct pwm_dt_spec motor_fl = PWM_DT_SPEC_GET(DT_ALIAS(motor_front_left));
static const struct pwm_dt_spec motor_fr = PWM_DT_SPEC_GET(DT_ALIAS(motor_front_right));
static const struct pwm_dt_spec motor_rl = PWM_DT_SPEC_GET(DT_ALIAS(motor_rear_left));
static const struct pwm_dt_spec motor_rr = PWM_DT_SPEC_GET(DT_ALIAS(motor_rear_right));

#if IS_ENABLED(CONFIG_FLIGHT_CTL_RC_IBUS)
static const struct device *const ibus_uart = DEVICE_DT_GET(DT_CHOSEN(quad_ibus_uart));
#else
static const struct pwm_dt_spec rc_throttle = PWM_DT_SPEC_GET(DT_ALIAS(rc_throttle));
static const struct pwm_dt_spec rc_roll     = PWM_DT_SPEC_GET(DT_ALIAS(rc_roll));
static const struct pwm_dt_spec rc_pitch    = PWM_DT_SPEC_GET(DT_ALIAS(rc_pitch));
static const struct pwm_dt_spec rc_yaw      = PWM_DT_SPEC_GET(DT_ALIAS(rc_yaw));
static const struct pwm_dt_spec rc_mode     = PWM_DT_SPEC_GET(DT_ALIAS(rc_mode));
static const struct pwm_dt_spec rc_landing  = PWM_DT_SPEC_GET(DT_ALIAS(rc_landing));
#endif

static const struct device *const gyro_dev  = DEVICE_DT_GET(DT_CHOSEN(flt_ctl_gyro));
static const struct device *const accel_dev = DEVICE_DT_GET(DT_CHOSEN(flt_ctl_accel));

#if IS_ENABLED(CONFIG_FLIGHT_CTL_HAS_BARO)
static const struct device *const baro_dev  = DEVICE_DT_GET(DT_CHOSEN(flt_ctl_baro));
#endif


#if !IS_ENABLED(CONFIG_FLIGHT_CTL_RC_IBUS)
static volatile int64_t last_rc_update_time = 0;

static void rc_capture_cb(const struct device *dev, uint32_t channel,
                          uint32_t period_cycles, uint32_t pulse_cycles,
                          int status, void *user_data)
{
    if (status == 0) {
        volatile uint16_t *target_val = (volatile uint16_t *)user_data;
        uint64_t pulse_us;
        if (pwm_cycles_to_usec(dev, channel, pulse_cycles, &pulse_us) == 0) {
            if (pulse_us >= MOTOR_MIN_PULSE_US && pulse_us <= MOTOR_MAX_PULSE_US) {
                *target_val = (uint16_t)pulse_us;
                last_rc_update_time = k_uptime_get();
            }
        }
    }
}
#endif

void flight_control_thread(void *p1, void *p2, void *p3)
{
    struct sensor_value gyro_data[3];
    struct sensor_value accel_data[3];
    float roll_rate, pitch_rate, yaw_rate;
    float accel_x, accel_y, accel_z;
    uint32_t log_divider = 0;

#if !IS_ENABLED(CONFIG_FLIGHT_CTL_RC_IBUS)
    static volatile uint16_t rc_throttle_val = 1000;
    static volatile uint16_t rc_roll_val     = 1500;
    static volatile uint16_t rc_pitch_val    = 1500;
    static volatile uint16_t rc_yaw_val      = 1500;
    static volatile uint16_t rc_mode_val     = 1500;
    static volatile uint16_t rc_landing_val  = 1000;
#endif

    if (!device_is_ready(motor_fl.dev) || !device_is_ready(motor_fr.dev) ||
        !device_is_ready(motor_rl.dev) || !device_is_ready(motor_rr.dev)) {
        LOG_ERR("PWM motor outputs are not ready!");
        return;
    }
    if (!device_is_ready(gyro_dev) || !device_is_ready(accel_dev)) {
        LOG_ERR("NXP Motion Sensor Shield IMU units are not ready!");
        return;
    }
#if IS_ENABLED(CONFIG_FLIGHT_CTL_HAS_BARO)
    if (!device_is_ready(baro_dev)) {
        LOG_ERR("DPS310 Barometer sensor node is not ready!");
        return;
    }
#endif
#if IS_ENABLED(CONFIG_FLIGHT_CTL_RC_IBUS)
    if (ibus_init(ibus_uart) != 0) {
        LOG_ERR("iBUS receiver failed to initialize!");
        return;
    }
#else
    pwm_flags_t cap_flags = PWM_POLARITY_NORMAL | PWM_CAPTURE_TYPE_PULSE | PWM_CAPTURE_MODE_CONTINUOUS;

    pwm_configure_capture(rc_throttle.dev, rc_throttle.channel, cap_flags, rc_capture_cb, (void*)&rc_throttle_val);
    pwm_enable_capture(rc_throttle.dev, rc_throttle.channel);

    pwm_configure_capture(rc_roll.dev, rc_roll.channel, cap_flags, rc_capture_cb, (void*)&rc_roll_val);
    pwm_enable_capture(rc_roll.dev, rc_roll.channel);

    pwm_configure_capture(rc_pitch.dev, rc_pitch.channel, cap_flags, rc_capture_cb, (void*)&rc_pitch_val);
    pwm_enable_capture(rc_pitch.dev, rc_pitch.channel);

    pwm_configure_capture(rc_yaw.dev, rc_yaw.channel, cap_flags, rc_capture_cb, (void*)&rc_yaw_val);
    pwm_enable_capture(rc_yaw.dev, rc_yaw.channel);

    pwm_configure_capture(rc_mode.dev, rc_mode.channel, cap_flags, rc_capture_cb, (void*)&rc_mode_val);
    pwm_enable_capture(rc_mode.dev, rc_mode.channel);

    pwm_configure_capture(rc_landing.dev, rc_landing.channel, cap_flags, rc_capture_cb, (void*)&rc_landing_val);
    pwm_enable_capture(rc_landing.dev, rc_landing.channel);
#endif

#if IS_ENABLED(CONFIG_CMSIS_DSP)
    roll_pid.Kp = 1.2f;  roll_pid.Ki = 0.05f;  roll_pid.Kd = 0.12f;
    arm_pid_init_f32(&roll_pid, 1);
    pitch_pid.Kp = 1.2f; pitch_pid.Ki = 0.05f; pitch_pid.Kd = 0.12f;
    arm_pid_init_f32(&pitch_pid, 1);
    yaw_pid.Kp = 2.0f;   yaw_pid.Ki = 0.10f;   yaw_pid.Kd = 0.05f;
    arm_pid_init_f32(&yaw_pid, 1);

    angle_pid_roll.Kp = 4.0f;  angle_pid_roll.Ki = 0.05f;  angle_pid_roll.Kd = 0.1f;
    arm_pid_init_f32(&angle_pid_roll, 1);
    angle_pid_pitch.Kp = 4.0f; angle_pid_pitch.Ki = 0.05f; angle_pid_pitch.Kd = 0.1f;
    arm_pid_init_f32(&angle_pid_pitch, 1);

#if IS_ENABLED(CONFIG_FLIGHT_CTL_HAS_BARO)
    alt_pid.Kp = 2.5f;   alt_pid.Ki = 0.15f;   alt_pid.Kd = 0.45f;
    arm_pid_init_f32(&alt_pid, 1);
#endif
#endif

    LOG_INF("Starting IMU Gyro Calibration. Please keep the drone perfectly still...");
    float gyro_bias_roll = 0.0f, gyro_bias_pitch = 0.0f, gyro_bias_yaw = 0.0f;
    const int calib_samples = 200; // 2 seconds at 10ms per loop
    for (int i = 0; i < calib_samples; i++) {
        sensor_sample_fetch(gyro_dev);
        sensor_channel_get(gyro_dev, SENSOR_CHAN_GYRO_XYZ, gyro_data);
        gyro_bias_roll  += (float)sensor_value_to_double(&gyro_data[0]);
        gyro_bias_pitch += (float)sensor_value_to_double(&gyro_data[1]);
        gyro_bias_yaw   += (float)sensor_value_to_double(&gyro_data[2]);

        // Also fetch accelerometer to allow the Mahony filter to align with gravity
        sensor_sample_fetch(accel_dev);
        sensor_channel_get(accel_dev, SENSOR_CHAN_ACCEL_XYZ, accel_data);
        float ax = (float)sensor_value_to_double(&accel_data[0]);
        float ay = (float)sensor_value_to_double(&accel_data[1]);
        float az = (float)sensor_value_to_double(&accel_data[2]);
        mahony_update(0.0f, 0.0f, 0.0f, ax, ay, az, 0.01f);

        k_sleep(K_MSEC(10));
    }
    gyro_bias_roll  /= (float)calib_samples;
    gyro_bias_pitch /= (float)calib_samples;
    gyro_bias_yaw   /= (float)calib_samples;
    LOG_INF("Calibration Complete. Biases - Roll: %f, Pitch: %f, Yaw: %f", (double)gyro_bias_roll, (double)gyro_bias_pitch, (double)gyro_bias_yaw);

    LOG_INF("Quadcopter Flight Control Workspace Online. Starting loop...");

    while (1) {
        int64_t start_time = k_uptime_get();

        uint16_t mode_switch_val = 1500;
        uint16_t landing_switch_val = 1000;

        /* 1. PILOT RADIO RECEIVER INPUT */
#if IS_ENABLED(CONFIG_FLIGHT_CTL_RC_IBUS)
        /* iBUS: All channels from a single UART stream */
        target_throttle = (float)ibus_channel(2);                    /* CH3 = Throttle */
        target_roll     = ((float)ibus_channel(0) - 1500.0f) * 0.1f; /* CH1 = Roll     */
        target_pitch    = ((float)ibus_channel(1) - 1500.0f) * 0.1f; /* CH2 = Pitch    */
        target_yaw      = ((float)ibus_channel(3) - 1500.0f) * 0.1f; /* CH4 = Yaw      */
        mode_switch_val = ibus_channel(4);                           /* CH5 = Mode     */
        landing_switch_val = ibus_channel(5);                        /* CH6 = Landing  */
#else
        /* PWM Capture (Continuous Interrupt-Driven): Read from volatile memory */
        target_throttle = (float)rc_throttle_val;
        target_roll     = ((float)rc_roll_val - 1500.0f) * 0.1f; 
        target_pitch    = ((float)rc_pitch_val - 1500.0f) * 0.1f;
        target_yaw      = ((float)rc_yaw_val - 1500.0f) * 0.1f;
        mode_switch_val = rc_mode_val;
        landing_switch_val = rc_landing_val;

        /* Failsafe Check */
        if (k_uptime_get() - last_rc_update_time > 500) {
            if (current_state == STATE_MANUAL_FLYING || current_state == STATE_ALT_HOLD) {
                LOG_WRN("Failsafe triggered! RC signal lost.");
                current_state = STATE_LANDING;
            }
        }
#endif

        if (landing_switch_val > 1700) {
            if (current_state == STATE_MANUAL_FLYING || current_state == STATE_ALT_HOLD) {
                LOG_INF("Landing switch triggered.");
                current_state = STATE_LANDING;
            }
        }

        /* 2. IMU SENSOR FETCH (FXAS21002 GYRO & FXOS8700 ACCEL) */
        sensor_sample_fetch(gyro_dev);
        sensor_channel_get(gyro_dev, SENSOR_CHAN_GYRO_XYZ, gyro_data);
        roll_rate  = (float)sensor_value_to_double(&gyro_data[0]) - gyro_bias_roll;
        pitch_rate = (float)sensor_value_to_double(&gyro_data[1]) - gyro_bias_pitch;
        yaw_rate   = (float)sensor_value_to_double(&gyro_data[2]) - gyro_bias_yaw;

        sensor_sample_fetch(accel_dev);
        sensor_channel_get(accel_dev, SENSOR_CHAN_ACCEL_XYZ, accel_data);
        accel_x = (float)sensor_value_to_double(&accel_data[0]);
        accel_y = (float)sensor_value_to_double(&accel_data[1]);
        accel_z = (float)sensor_value_to_double(&accel_data[2]);

        /* Mahony Filter for Attitude Estimation */
        mahony_update(roll_rate, pitch_rate, yaw_rate, accel_x, accel_y, accel_z, DT_SEC);
        mahony_get_euler(&est_roll, &est_pitch);

        /* 3. CONDITIONAL ALTITUDE CONTROL / LANDING ASSIST STATE MACHINE */
        float dynamic_throttle = target_throttle;

#if IS_ENABLED(CONFIG_FLIGHT_CTL_HAS_BARO)
        struct sensor_value press_val;
        sensor_sample_fetch(baro_dev);
        sensor_channel_get(baro_dev, SENSOR_CHAN_PRESS, &press_val);
        float pressure_pa = (float)sensor_value_to_double(&press_val);

        current_alt = (44330.77f * (1.0f - powf((pressure_pa / 101325.0f), 0.190263f)));

        if (current_state == STATE_LANDING) {
            target_altitude -= DESCENT_RATE_MPS * DT_SEC;
            if (current_alt <= LANDING_THRESHOLD_M) {
                current_state = STATE_DISARMED;
            }
        }

        if (current_state == STATE_ALT_HOLD || current_state == STATE_LANDING) {
            float alt_output = pid_update(&alt_pid, target_altitude, current_alt, DT_SEC);
            dynamic_throttle = (float)BASE_HOVER_THROTTLE + alt_output;
        }
#endif

        if (current_state == STATE_DISARMED) {
            dynamic_throttle = (float)MOTOR_MIN_PULSE_US;
#if IS_ENABLED(CONFIG_CMSIS_DSP)
            arm_pid_reset_f32(&roll_pid);
            arm_pid_reset_f32(&pitch_pid);
            arm_pid_reset_f32(&yaw_pid);
#else
            roll_pid.integrator = 0.0f;
            pitch_pid.integrator = 0.0f;
            yaw_pid.integrator = 0.0f;
#endif
        }

        /* 4. FLIGHT MODE LOGIC */
        if (mode_switch_val < 1300) {
            current_flight_mode = FLIGHT_MODE_ACRO;
        } else if (mode_switch_val < 1700) {
            current_flight_mode = FLIGHT_MODE_ANGLE;
        } else {
            current_flight_mode = FLIGHT_MODE_HORIZON;
        }

        float target_roll_rate  = 0.0f;
        float target_pitch_rate = 0.0f;

        if (current_flight_mode == FLIGHT_MODE_ACRO) {
            /* In Acro, stick input is directly the target rate */
            target_roll_rate  = target_roll;
            target_pitch_rate = target_pitch;
        } else if (current_flight_mode == FLIGHT_MODE_ANGLE) {
            /* In Angle, stick input is the target angle (e.g. max 45 degrees) */
            /* stick target_roll is scaled -50 to +50 approx. Scale to ±45 deg */
            float target_roll_angle  = target_roll * 0.9f;
            float target_pitch_angle = target_pitch * 0.9f;

            target_roll_rate  = pid_update(&angle_pid_roll, target_roll_angle, est_roll, DT_SEC);
            target_pitch_rate = pid_update(&angle_pid_pitch, target_pitch_angle, est_pitch, DT_SEC);
        } else if (current_flight_mode == FLIGHT_MODE_HORIZON) {
            float target_roll_angle  = target_roll * 0.9f;
            float target_pitch_angle = target_pitch * 0.9f;

            float angle_roll_rate  = pid_update(&angle_pid_roll, target_roll_angle, est_roll, DT_SEC);
            float angle_pitch_rate = pid_update(&angle_pid_pitch, target_pitch_angle, est_pitch, DT_SEC);

            /* Blend based on stick deflection. target_roll is -50 to 50 approx. */
            float blend_roll  = fabsf(target_roll) / 50.0f;
            if (blend_roll > 1.0f) blend_roll = 1.0f;
            
            float blend_pitch = fabsf(target_pitch) / 50.0f;
            if (blend_pitch > 1.0f) blend_pitch = 1.0f;

            target_roll_rate  = (1.0f - blend_roll) * angle_roll_rate + blend_roll * target_roll;
            target_pitch_rate = (1.0f - blend_pitch) * angle_pitch_rate + blend_pitch * target_pitch;
        }

        /* 5. RATE CORES AND QUADCOPTER MOTOR MIXER (X-CONFIGURATION) */
        float roll_output  = pid_update(&roll_pid, target_roll_rate, roll_rate, DT_SEC);
        float pitch_output = pid_update(&pitch_pid, target_pitch_rate, pitch_rate, DT_SEC);
        float yaw_output   = pid_update(&yaw_pid, target_yaw, yaw_rate, DT_SEC);

        float m1_fl = dynamic_throttle + roll_output - pitch_output - yaw_output;
        float m2_fr = dynamic_throttle - roll_output - pitch_output + yaw_output;
        float m3_rl = dynamic_throttle + roll_output + pitch_output + yaw_output;
        float m4_rr = dynamic_throttle - roll_output + pitch_output - yaw_output;

        uint32_t fl_pulse = (uint32_t)CLAMP(m1_fl, MOTOR_MIN_PULSE_US, MOTOR_MAX_PULSE_US);
        uint32_t fr_pulse = (uint32_t)CLAMP(m2_fr, MOTOR_MIN_PULSE_US, MOTOR_MAX_PULSE_US);
        uint32_t rl_pulse = (uint32_t)CLAMP(m3_rl, MOTOR_MIN_PULSE_US, MOTOR_MAX_PULSE_US);
        uint32_t rr_pulse = (uint32_t)CLAMP(m4_rr, MOTOR_MIN_PULSE_US, MOTOR_MAX_PULSE_US);

        pwm_set_pulse_dt(&motor_fl, PWM_USEC(fl_pulse));
        pwm_set_pulse_dt(&motor_fr, PWM_USEC(fr_pulse));
        pwm_set_pulse_dt(&motor_rl, PWM_USEC(rl_pulse));
        pwm_set_pulse_dt(&motor_rr, PWM_USEC(rr_pulse));

        /* 6. BLACKBOX LOGGING DOWN-SAMPLING (400Hz -> 40Hz) */
        if (++log_divider >= 10) {
            log_divider = 0;
            const char* mode_str = (current_flight_mode == FLIGHT_MODE_ACRO) ? "ACRO" :
                                   (current_flight_mode == FLIGHT_MODE_ANGLE) ? "ANGLE" : "HORIZON";
            const char* state_str = (current_state == STATE_DISARMED) ? "DISARMED" :
                                    (current_state == STATE_MANUAL_FLYING) ? "FLYING" :
                                    (current_state == STATE_ALT_HOLD) ? "ALTHOLD" :
                                    (current_state == STATE_LANDING) ? "LANDING" : "UNKNOWN";
#if IS_ENABLED(CONFIG_FLIGHT_CTL_HAS_BARO)
            LOG_INF("ST:%s %s | Alt:%.2fm (Tgt:%.2fm) | EstAng:(%.1f, %.1f) | Gyro:(%.1f, %.1f, %.1f) | Thr:%.0f",
                    state_str, mode_str, (double)current_alt, (double)target_altitude,
                    (double)est_roll, (double)est_pitch,
                    (double)roll_rate, (double)pitch_rate, (double)yaw_rate,
                    (double)dynamic_throttle);
#else
            LOG_INF("ST:%s %s | EstAng:(R:%.1f, P:%.1f) | Gyro:(R:%.1f, P:%.1f, Y:%.1f) | Thr:%.0f",
                    state_str, mode_str, (double)est_roll, (double)est_pitch,
                    (double)roll_rate, (double)pitch_rate,
                    (double)yaw_rate, (double)dynamic_throttle);
#endif
        }

        int64_t elapsed = k_uptime_get() - start_time;
        if (elapsed < (int64_t)FLIGHT_LOOP_PERIOD_MS) {
            k_msleep((int32_t)(FLIGHT_LOOP_PERIOD_MS - (float)elapsed));
        }
    }
}

K_THREAD_DEFINE(flight_ctl_id, 2048, flight_control_thread, NULL, NULL, NULL,
                FLIGHT_LOOP_PRIORITY, 0, 0);