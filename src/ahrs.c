#include "ahrs.h"
#include <devicetree.h>
#include <zephyr.h>
#include <drivers/sensor.h>
#include <stdio.h>
#include "MadgwickAHRS.h"
#include <logging/log.h>
#include <stdbool.h>

#define NUM_ACCEL 2
#define NUM_MAG 2
#define NUM_GYRO 1

LOG_MODULE_REGISTER(AHRS, CONFIG_SENSOR_LOG_LEVEL);

K_SEM_DEFINE(sem, 0, 1);	/* starts off "not available" */

#if !defined(CONFIG_SENSORFUSION_POLLING)
static void get_sample_trigger_handler(const struct device *dev,
			    struct sensor_trigger *trigger)
{
	if (sensor_sample_fetch(dev)) {
		printf("sensor_sample_fetch failed\n");
		return;
	}
}
#endif

void RunAHRS(void)
{
	// TODO: Ensure value doesn't become stale, somehow?
	// TODO: Move array initialization to preprocessor magic
	bool sensor_success = false;
	struct sensor_value accel[3];
	struct sensor_value mag[3];
	struct sensor_value gyro[3];
	const struct device *accels[NUM_ACCEL];
	const struct device *mags[NUM_MAG];
	const struct device *gyros[NUM_GYRO];
	accels[0] = device_get_binding(DT_LABEL(DT_INST(0, nxp_fxos8700)));
	accels[1] = device_get_binding(DT_LABEL(DT_INST(1, nxp_fxos8700)));
	mags[0] = accels[0];
	mags[1] = accels[1];
	gyros[0] = device_get_binding(DT_LABEL(DT_INST(0, nxp_fxas21002)));

	uint8_t dev_num;
	struct sensor_trigger trig;

	trig.type = SENSOR_TRIG_DATA_READY;
	trig.chan = SENSOR_CHAN_ACCEL_XYZ;
	for ( dev_num = 0; dev_num < NUM_ACCEL; dev_num++)  {
		LOG_DBG("Initializing Accelerometer %i", dev_num);
		if (accels[dev_num] == NULL) {
			printf("Could not get accelerometer device %i\n", dev_num);
			return;
		}

#if !defined(CONFIG_SENSORFUSION_POLLING)
		if (sensor_trigger_set(accels[dev_num], &trig, get_sample_trigger_handler)) {
			printf("Could not set trigger\n");
			return;
		}
#endif
	}

#ifdef CONFIG_SENSORFUSION_SEPARATE_MAG_ACCEL
	trig.type = SENSOR_TRIG_DATA_READY;
	trig.chan = SENSOR_CHAN_MAGN_XYZ;
#endif
	for ( dev_num = 0; dev_num < NUM_MAG; dev_num++)  {
		LOG_DBG("Initializing Magnetometer %i", dev_num);
		if (mags[dev_num] == NULL) {
			printf("Could not get magnetometer device %i\n", dev_num);
			return;
		}

#if defined(CONFIG_SENSORFUSION_SEPARATE_MAG_ACCEL) && !defined(CONFIG_SENSORFUSION_POLLING)
		if (sensor_trigger_set(mags[dev_num], &trig, get_sample_trigger_handler)) {
			printf("Could not set trigger\n");
			return;
		}
#endif
	}

	trig.type = SENSOR_TRIG_DATA_READY;
	trig.chan = SENSOR_CHAN_GYRO_XYZ;
	for ( dev_num = 0; dev_num < NUM_GYRO; dev_num++)  {
		LOG_DBG("Initializing Gyroscope %i", dev_num);
		if (gyros[dev_num] == NULL) {
			printf("Could not get gyroscope device %i\n", dev_num);
			return;
		}

#if !defined(CONFIG_SENSORFUSION_POLLING)
		if (sensor_trigger_set(gyros[dev_num], &trig, get_sample_trigger_handler)) {
			printf("Could not set trigger\n");
			return;
		}
#endif
	}

	LOG_DBG("Starting AHRS");

	while (1) {
		// TODO: Clear data ready if polling?
		// TODO: Synchronizing hardware data sampling may be very important to reduce drift?

		k_usleep(19530U); // 512 Hz

#ifdef CONFIG_SENSORFUSION_POLLING
		for ( dev_num = 0; dev_num < NUM_ACCEL; dev_num++)  {
			sensor_sample_fetch(accels[dev_num]);
		}
		for ( dev_num = 0; dev_num < NUM_MAG; dev_num++)  {
			sensor_sample_fetch(mags[dev_num]);
		}
		for ( dev_num = 0; dev_num < NUM_GYRO; dev_num++)  {
			sensor_sample_fetch(gyros[dev_num]);
		}
#endif
		sensor_success = false;
		for ( dev_num = 0; dev_num < NUM_ACCEL; dev_num++)  {
			if (sensor_channel_get(accels[dev_num], SENSOR_CHAN_ACCEL_XYZ, accel)) {
				LOG_ERR("Could not read from accelerometer device %i", dev_num);
			} else {
				sensor_success = true;
				break;
			}
			if (!sensor_success) {
				LOG_ERR("No accelorometer could be read");
			}
		}

		sensor_success = false;
		for ( dev_num = 0; dev_num < NUM_MAG; dev_num++)  {
			if (sensor_channel_get(mags[dev_num], SENSOR_CHAN_MAGN_XYZ, mag)) {
				LOG_ERR("Could not read from magnetometer device %i", dev_num);
			} else {
				sensor_success = true;
				break;
			}
			if (!sensor_success) {
				LOG_ERR("No magnetometer could be read");
			}
		}

		sensor_success = false;
		for ( dev_num = 0; dev_num < NUM_GYRO; dev_num++)  {
			if (sensor_channel_get(mags[dev_num], SENSOR_CHAN_MAGN_XYZ, gyro)) {
				LOG_ERR("Could not read from gyrometer device %i", dev_num);
			} else {
				sensor_success = true;
				break;
			}
			if (!sensor_success) {
				LOG_ERR("No gyrometer could be read");
			}
		}

		printf("AX=%10.6f AY=%10.6f AZ=%10.6f ",
			sensor_value_to_double(&accel[0]),
			sensor_value_to_double(&accel[1]),
			sensor_value_to_double(&accel[2]));
		printf("MX=%10.6f MY=%10.6f MZ=%10.6f ",
			sensor_value_to_double(&mag[0]),
			sensor_value_to_double(&mag[1]),
			sensor_value_to_double(&mag[2]));
		printf("GX=%10.3f GY=%10.3f GZ=%10.3f\n",
			sensor_value_to_double(&gyro[0]),
			sensor_value_to_double(&gyro[1]),
			sensor_value_to_double(&gyro[2]));
	}
}
