#include <zephyr.h>
#include <sys/printk.h>

void main(void)
{
	printk("I am Groot! %s\n", CONFIG_BOARD);
}
