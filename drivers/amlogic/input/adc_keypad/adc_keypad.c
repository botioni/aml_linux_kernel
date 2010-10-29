/*
 * linux/drivers/input/adc_kbd/adc_keypad.c
 *
 * ADC Keypad Driver
 *
 * Copyright (C) 2010 Amlogic Corporation
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 * author :   Robin Zhu
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/types.h>
#include <linux/input.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/errno.h>
#include <asm/irq.h>
#include <asm/io.h>

#include <mach/am_regs.h>
#include <mach/pinmux.h>
#include <linux/major.h>
#include <linux/slab.h>
#include <asm/uaccess.h>

struct kp {
	struct input_dev *input;
	struct timer_list timer;
	unsigned int cur_keycode;
	int			config_major;
	char 		config_name[20];
	struct class *config_class;
	struct device *config_dev;
};

static struct kp *gp_kp=NULL;

static void init_adc(void)
{
    WRITE_CBUS_REG(SAR_ADC_REG3, (READ_CBUS_REG(SAR_ADC_REG3) & ~(0x3f << 10)) | ((1 << 30) | (20 << 10)));
    // Enable ADC
    WRITE_CBUS_REG(SAR_ADC_REG3, READ_CBUS_REG(SAR_ADC_REG3) | (1<<21));
    // Setup ADC_CTRL[4:0]
    WRITE_CBUS_REG(SAR_ADC_REG3, (READ_CBUS_REG(SAR_ADC_REG3) & ~(0x1F << 23)) | (0 << 23));
    // Setup MSR_A[2:0]
    WRITE_CBUS_REG(SAR_ADC_DETECT_IDLE_SW, (READ_CBUS_REG(SAR_ADC_DETECT_IDLE_SW) & ~(0x03FF << 0)) | (4 << 7));
    WRITE_CBUS_REG(SAR_ADC_CHAN_10_SW, (READ_CBUS_REG(SAR_ADC_CHAN_10_SW) & ~(0x03FF << 0)) | (4 << 7));
    // TEMPSEN_PD12, TEMPSEN_MODE
    WRITE_CBUS_REG(SAR_ADC_REG3, (READ_CBUS_REG(SAR_ADC_REG3) & ~(0x3 << 28)) | ((0 << 29) | (0 << 28)));
    printk("reg3=%x,detect_idle=%x,chan_10_sw=%x", READ_CBUS_REG(SAR_ADC_REG3), READ_CBUS_REG(SAR_ADC_DETECT_IDLE_SW), READ_CBUS_REG(SAR_ADC_CHAN_10_SW));
}

static unsigned int get_adc_sample(void)
{
	unsigned long data[7] = {0};
	int count = 0;
	int sum = 0;
	int i = 0;
	int res = 0x3ff;

    // Disable the sampling engine
        WRITE_CBUS_REG(SAR_ADC_REG0, READ_CBUS_REG(SAR_ADC_REG0) & ~(1 << 0));
	
    // Enable the sampling engine
        WRITE_CBUS_REG(SAR_ADC_REG0, READ_CBUS_REG(SAR_ADC_REG0) | (1 << 0));

	// Start Sample
        WRITE_CBUS_REG(SAR_ADC_REG0, READ_CBUS_REG(SAR_ADC_REG0) | (1 << 2));

	//wait fifo full
        while (((READ_CBUS_REG(SAR_ADC_REG0) >> 21) & 0x1f) < 7) {;}

	//Stop sample and wait all engine stop
	WRITE_CBUS_REG(SAR_ADC_REG0, READ_CBUS_REG(SAR_ADC_REG0) | (1 << 14));
	while (((READ_CBUS_REG(SAR_ADC_REG0) >> 28) & 0x7)) {;}

	for( i = 0; i < 7; i++){
		data[i] = (READ_CBUS_REG(SAR_ADC_FIFO_RD) & 0x3ff);
	}

	//printk("get_adc_sample = %x,%x,%x,%x,%x,%x.\n", data[0],data[1],data[2],data[3],data[4],data[5]);
	//Ignore the first value
	for( i = 1; i < 7; i++){
		//Ignore invalidate value
		if(data[i] < 0x3e0 && data[i] != 0x1fe ){
			sum += data[i];
			count++;
		}
	}

	if(count != 0)
		res = (int)sum/count;
	
    return res;	
}

static void adckp_timer_sr(unsigned long data)
{
	  unsigned int result;
    struct kp *kp_data=(struct kp *)data;
    result = get_adc_sample();
    if (result>=0x3e0){
        if (kp_data->cur_keycode != 0){
            input_report_key(kp_data->input,kp_data->cur_keycode, 0);	
            kp_data->cur_keycode = 0;
		        printk("adc ch4 sample = %x, keypad released.\n", result);
        }
    }
	else if (result>=0x0 && result<0x60 ) {
		if (kp_data->cur_keycode!=KEY_HOME){
    	  kp_data->cur_keycode = KEY_HOME;
    	  input_report_key(kp_data->input,kp_data->cur_keycode, 1);	
    	  printk("adc ch4 sample = %x, keypad pressed.\n", result);
		}
    }
	else if (result>=0x110 && result<0x170 ) {
		if (kp_data->cur_keycode!=KEY_ENTER){
    	  kp_data->cur_keycode = KEY_ENTER;
    	  input_report_key(kp_data->input,kp_data->cur_keycode, 1);	
    	  printk("adc ch4 sample = %x, keypad pressed.\n", result);
		}
    }
	else if (result>=0x240 && result<0x290 ) {
		if (kp_data->cur_keycode!= KEY_TAB ){
    	  kp_data->cur_keycode = KEY_TAB;
    	  input_report_key(kp_data->input,kp_data->cur_keycode, 1);	
    	  printk("adc ch4 sample = %x, keypad pressed.\n", result);
		}
    }
	else if (result>=0x290 && result<0x380 ) {
		if (kp_data->cur_keycode!= KEY_LEFTMETA ){
    	  kp_data->cur_keycode = KEY_LEFTMETA;
    	  input_report_key(kp_data->input,kp_data->cur_keycode, 1);	
    	  printk("adc ch4 sample = %x, keypad pressed.\n", result);
    }
    }
    else{
		printk("adc ch4 sample = unknown key %x, pressed.\n", result);
    }
    mod_timer(&kp_data->timer,jiffies+msecs_to_jiffies(200));
}

static int
adckpd_config_open(struct inode *inode, struct file *file)
{
    file->private_data = gp_kp;
    return 0;
}

static int
adckpd_config_release(struct inode *inode, struct file *file)
{
    file->private_data=NULL;
    return 0;
}

static const struct file_operations keypad_fops = {
    .owner      = THIS_MODULE,
    .open       = adckpd_config_open,
    .ioctl      = NULL,
    .release    = adckpd_config_release,
};

static int register_keypad_dev(struct kp  *kp)
{
    int ret=0;
    strcpy(kp->config_name,"am_adc_kpd");
    ret=register_chrdev(0, kp->config_name, &keypad_fops);
    if(ret<=0)
    {
        printk("register char device error\r\n");
        return  ret ;
    }
    kp->config_major=ret;
    printk("adc keypad major:%d\r\n",ret);
    kp->config_class=class_create(THIS_MODULE,kp->config_name);
    kp->config_dev=device_create(kp->config_class,NULL,MKDEV(kp->config_major,0),NULL,kp->config_name);
    return ret;
}

static int __init kp_probe(struct platform_device *pdev)
{
    struct kp *kp;
    struct input_dev *input_dev;
    int ret;

    kp = kzalloc(sizeof(struct kp), GFP_KERNEL);
    input_dev = input_allocate_device();
    if (!kp || !input_dev) {
        kfree(kp);
        input_free_device(input_dev);
        return -ENOMEM;
    }
    gp_kp=kp;

    platform_set_drvdata(pdev, kp);
    kp->input = input_dev;

	  kp->cur_keycode = 0;
    setup_timer(&kp->timer, adckp_timer_sr, kp) ;
    mod_timer(&kp->timer, jiffies+msecs_to_jiffies(100));

    /* setup input device */
    set_bit(EV_KEY, input_dev->evbit);
    set_bit(EV_REP, input_dev->evbit);
    set_bit(KEY_TAB, input_dev->keybit);
	set_bit(KEY_HOME, input_dev->keybit);
	set_bit(KEY_LEFTMETA, input_dev->keybit);
	set_bit(KEY_ENTER, input_dev->keybit);
    
    input_dev->name = "adc_keypad";
    input_dev->phys = "adc_keypad/input0";
    input_dev->dev.parent = &pdev->dev;

    input_dev->id.bustype = BUS_ISA;
    input_dev->id.vendor = 0x0001;
    input_dev->id.product = 0x0001;
    input_dev->id.version = 0x0100;

    input_dev->rep[REP_DELAY]=0xffffffff;
    input_dev->rep[REP_PERIOD]=0xffffffff;

    input_dev->keycodesize = sizeof(unsigned short);
    input_dev->keycodemax = 0x1ff;

    ret = input_register_device(kp->input);
    if (ret < 0) {
        printk(KERN_ERR "Unable to register keypad input device.\n");
		    kfree(kp);
		    input_free_device(input_dev);
		    return -EINVAL;
    }
    printk("adc keypad register input device completed.\r\n");
		init_adc();
    register_keypad_dev(gp_kp);
    return 0;
}

static int kp_remove(struct platform_device *pdev)
{
    struct kp *kp = platform_get_drvdata(pdev);

    input_unregister_device(kp->input);
    input_free_device(kp->input);
    unregister_chrdev(kp->config_major,kp->config_name);
    if(kp->config_class)
    {
        if(kp->config_dev)
        device_destroy(kp->config_class,MKDEV(kp->config_major,0));
        class_destroy(kp->config_class);
    }
    kfree(kp);
    gp_kp=NULL ;
    return 0;
}

static struct platform_driver kp_driver = {
    .probe      = kp_probe,
    .remove     = kp_remove,
    .suspend    = NULL,
    .resume     = NULL,
    .driver     = {
        .name   = "m1-adckp",
    },
};

static int __devinit kp_init(void)
{
    printk(KERN_INFO "ADC Keypad Driver init.\n");

    return platform_driver_register(&kp_driver);
}

static void __exit kp_exit(void)
{
    printk(KERN_INFO "ADC Keypad Driver exit.\n");
    platform_driver_unregister(&kp_driver);
}

module_init(kp_init);
module_exit(kp_exit);

MODULE_AUTHOR("Robin Zhu");
MODULE_DESCRIPTION("ADC Keypad Driver");
MODULE_LICENSE("GPL");




