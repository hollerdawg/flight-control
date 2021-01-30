#ifndef ahrs_h
#define ahrs_h
#include <zephyr.h>
#include <drivers/sensor.h>
#include <stdio.h>
#include "MadgwickAHRS.h"

void RunAHRS(void);

#endif
