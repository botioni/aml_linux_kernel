#ifndef __LINUX_ADC_KEYPAD_H
#define __LINUX_ADC_KEYPAD_H

struct adc_key{
	int code;	/* input key code */
	unsigned char *name;
	int chan;
	int value;	/* voltage/3.3v * 1023 */
	int tolerance;
};

struct adc_kp_platform_data{
	struct adc_key *key;
	int key_num;
};

#endif