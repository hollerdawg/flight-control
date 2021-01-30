#ifndef motorcontrol_h
#define motorcontrol_h
#include <zephyr.h>
#include <sys/printk.h>
#include <device.h>
#include <drivers/pwm.h>

void RunMotorControl(void);

#endif
