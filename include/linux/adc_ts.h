#ifndef __LINUX_ADCTS_H
#define __LINUX_ADCTS_H

struct adc_ts_platform_data {
	int irq;
	u16 x_plate_ohms;
	void (*service)(int cmd);
};

#endif
