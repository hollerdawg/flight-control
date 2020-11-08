#include <zephyr.h>
#include <autoconf.h>
#include <sys/printk.h>
#include <kernel.h>
#include "ahrs.h"

/* size of stack area used by each thread */
#define STACKSIZE 1024

/* scheduling priority used by each thread */
#define PRIORITY 7

K_SEM_DEFINE(imu_sem, 0, 1);	/* starts off "not available" */

void RunFaultReporting(void);
void grootiam(char *greeting);

void grootiam(char *greeting)
{
	printk("I am Groot! %s\n", greeting);
}

void RunFaultReporting(void)
{
	char *greeting = "iamgroot";
	while(1)
	{
		grootiam(greeting);
		k_yield();
	}
}

K_THREAD_DEFINE(RunAHRS_id, STACKSIZE, RunAHRS, NULL, NULL, NULL,
		PRIORITY, 0, 0);
// K_THREAD_DEFINE(RunFaultReporting_id, STACKSIZE, RunFaultReporting, NULL, NULL, NULL,
// 		PRIORITY, 0, 0);
