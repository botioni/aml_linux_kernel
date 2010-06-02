/*
 * linux/drivers/input/irremote/apollo_remote_kbd.c
 *
 * apollo Keypad Driver
 *
 * Copyright (C) 2009 Amlogic Corporation
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
 * author :   jianfeng_wang
 */
 /*
 * !!caution: if you use remote ,you should disable card1 used for  ata_enable pin.
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
#include <asm/arch/am_regs.h>
#include <linux/major.h>

#undef NEW_BOARD_LEARNING_MODE


 #define IR_STATUS_REPEAT_KEY       		(1<<0)
 #define IR_STATUS_CUSTOM_ERROR     	(1<<1)
 #define IR_STATUS_DATA_CODE_ERROR  	(1<<2)
 #define IR_STATUS_FRAME_DATA_VALID 	(1<<3)

 #define IR_CONTROL_HOLD_LAST_KEY 		(1<<6)
 #define IR_CONTROL_DECODER_MODE		(3<<7)
 #define IR_CONTROL_SKIP_HEADER		(1<<7)
 #define IR_CONTROL_RESET         			(1<<0)

#define  	KEY_PRESS		0
#define	KEY_REPEAT		1
#define	KEY_RELEASE	2

#define   KEY_RELEASE_DELAY    200

 
#ifdef   DEBUG
#define pr_dbg(fmt, args...) printk(KERN_DEBUG "apollo kpads: " fmt, ## args)
//#define pr_err(fmt, args...) printk(KERN_ERR "apollo kpads: " fmt, ## args)
#else   
#define    pr_dbg(fmt, args...)
//#define 	  pr_err(fmt, args...)
#endif

static void apollo_kp_tasklet(unsigned long);



static DEFINE_MUTEX(kp_enable_mutex);
static DEFINE_MUTEX(kp_file_mutex);
static int kp_enable = 1;

static struct timer_list kp_timer;

static  int NEC_REMOTE_IRQ_NO=AM_ISA_GEN_IRQ(15);

typedef  struct {
	char		     *platform_name;
	unsigned int  pin_mux;
	unsigned int  bit;
}pin_config_t;

static  pin_config_t  pin_config[]={
		{
			.platform_name="8626_64x2",
#if defined(CONFIG_AMLOGIC_BOARD_APOLLO)||defined(CONFIG_AMLOGIC_BOARD_APOLLO_H)			
			.pin_mux=PREG_PIN_MUX_REG2,
			.bit=PINMUX2_CARD1_REMOTE,
#endif			
		},
		{
			.platform_name="8328_32x2",
#if  defined(CONFIG_AMLOGIC_BOARD_NIKE)				
			.pin_mux=PERIPHS_PIN_MUX_3,
#endif			
			.bit=(1<<25),
		},

};

struct apollo_kp {
	struct input_dev *input;
	struct timer_list timer;
	int irq;

	unsigned long cur_keycode;
	unsigned int 	repeate_flag;
	unsigned long delay;
	unsigned int debounce;
	unsigned int custom_code;
	unsigned int release_delay;
};
//our keyboard is 9 rows and 4 cols 
#define    KEY_MAX_NUM		36

//this array store scancodes of every key.

static unsigned long    apollo_scan_code[KEY_MAX_NUM]={
	0xeb14ff00,0xffffffff,0xffffffff,0xb946ff00,
	0xf609ff00,0xe21dff00,0xe01fff00,0xf20dff00,
	0xe619ff00,0xe41bff00,0xee11ff00,0xea15ff00,
	0xe817ff00,0xed12ff00,0xffffffff,0xf708ff00,
	0xaf50ff00,0xffffffff,0xb748ff00,0xffffffff,
	0xffffffff,0xf906ff00,0xffffffff,0xfc03ff00,
	0xb847ff00,0xf807ff00,0xbf40ff00,0xffffffff,
	0xe718ff00,0xbb44ff00,0xffffffff,0xae51ff00,
	0xf50aff00,0xe11eff00,0xf10eff00,0xe51aff00
		
};

static unsigned short  apollo_keycode[KEY_MAX_NUM]={
	KEY_POWER,0,0,KEY_MUTE,
	KEY_F1,KEY_F2,KEY_F3,KEY_TEXT,
	KEY_F4,KEY_F5,KEY_ZOOM,KEY_F6,
	KEY_F7,KEY_MENU,0,KEY_BACK,
	KEY_F8,0,KEY_F9,0,
	0,KEY_UP,0,KEY_MINUS,
	KEY_LEFT,KEY_ENTER,KEY_RIGHT,0,
	KEY_EXIT,KEY_DOWN,0,KEY_MINUS,
	KEY_PLAYPAUSE,KEY_STOP,KEY_REWIND,KEY_FORWARD
};
/*
static unsigned short   apollo_repeat_keycode[KEY_MAX_NUM]={
	KEY_POWER,0,0,KEY_MUTE,
	KEY_A,KEY_B,KEY_C,KEY_D,
	KEY_E,KEY_F,KEY_G,KEY_H,
	KEY_I,KEY_G,0,KEY_K,
	KEY_L,0,KEY_M,0,
	0,KEY_UP,0,KEY_MINUS,
	KEY_LEFT,KEY_ENTER,KEY_RIGHT,0,
	KEY_ESC,KEY_DOWN,0,KEY_MINUS,
	KEY_TAB,KEY_HOME,KEY_PAGEUP,KEY_PAGEDOWN
};*/

DECLARE_TASKLET_DISABLED(kp_tasklet, apollo_kp_tasklet, 0);

static  inline int apollo_find_key_index(unsigned int  scan_code)
{
	int  i ;
	for (i=0;i<KEY_MAX_NUM;i++)
	{
		if(apollo_scan_code[i]==scan_code)
		{
			return i;
		}
	}
	return -1;
}

void	kp_timer_sr(unsigned long data)
{
	struct apollo_kp *apollo_kp_data=(struct apollo_kp *)data;
#if defined(CONFIG_KEYPADS_APOLLO_REPORT_SCANCODE) //only report scan code.
	input_report_key(apollo_kp_data->input,(apollo_kp_data->cur_keycode>>16)&0xff ,0);
	pr_dbg("key release\r\n");
#else
	int  key_index=apollo_find_key_index (apollo_kp_data->cur_keycode);
	pr_dbg("key ==>0x%x  released\r\n",apollo_keycode[key_index]);
	input_report_key(apollo_kp_data->input,apollo_keycode[key_index] ,0);
#endif
}


static irqreturn_t apollo_kp_interrupt(int irq, void *dev_id)
{
	/* disable keyboard interrupt and schedule for handling */
//	pr_dbg("===trigger one  kpads interupt \r\n");
	tasklet_schedule(&kp_tasklet);

	return IRQ_HANDLED;
}


static inline int apollo_kp_find_key(struct apollo_kp *apollo_kp_data )
{
	int  key_index;
	unsigned  int  status,scan_code;
	static  int last_scan_code,key_hold;
	
	 
	// 1		get  scan code
	scan_code=READ_PERIPHS_REG(PREG_IR_DEC_FRAME_DATA);
	status=READ_PERIPHS_REG(PREG_IR_DEC_FRAME_STATUS);
	
#if defined(CONFIG_KEYPADS_APOLLO_REPORT_SCANCODE) //only report scan code.
	key_index=0 ;
	key_hold=-1 ;
	if(scan_code)  //key first press
	{
		input_report_key(apollo_kp_data->input,(scan_code>>16)&0xff,1);
		pr_dbg("key pressed,scan code :0x%x\r\n",scan_code);
	}
	else if(scan_code==0 && status&0x1) //repeate key
	{
		scan_code=last_scan_code;
		input_report_key(apollo_kp_data->input,(scan_code>>16)&0xff,2);
		pr_dbg("key repeate,scan code :0x%x\r\n",scan_code);
	}
	apollo_kp_data->cur_keycode=last_scan_code;
	last_scan_code=scan_code;
	kp_timer.data=(unsigned long)apollo_kp_data;	
	mod_timer(&kp_timer,jiffies+msecs_to_jiffies(apollo_kp_data->release_delay));
	return -1 ;
#else
	//pr_dbg("current scan code is:0x%x\r\n",scan_code);
	if(scan_code) key_hold=1;
	else	 key_hold++;
	if(scan_code==0 && status&0x1 && key_hold != 2)//repeat key
	{
		scan_code=last_scan_code;
	}
	key_index=apollo_find_key_index(scan_code);
	//dont change this sentence, it's ok .
	if(key_index<0) return -1;
	last_scan_code=scan_code;
	apollo_kp_data->cur_keycode=last_scan_code;
	kp_timer.data=(unsigned long)apollo_kp_data;	
	mod_timer(&kp_timer,jiffies+msecs_to_jiffies(apollo_kp_data->release_delay));
	// 2		return map key code.
	if(status&0x01) //repeate key 
	{
		
		return apollo_keycode[key_index]|1<<15;
	}
	else
	{
		return apollo_keycode[key_index] ;
	}
#endif	
}

static void apollo_kp_tasklet(unsigned long data)
{
	struct apollo_kp *apollo_kp_data = (struct apollo_kp *) data;
	int  key_code ;     	
	key_code = apollo_kp_find_key(apollo_kp_data);
	if(key_code != -1 && key_code!=0 )
	{
		input_report_key(apollo_kp_data->input, key_code&~(1<<15),key_code&(1<<15)?2:1);
		pr_dbg("key ==> 0x%x - %s\r\n",key_code&~(1<<15),key_code&(1<<15)?"repeat":"pressed");
	}
	
	
}

static ssize_t apollo_kp_enable_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", kp_enable);
}
//control var by sysfs .
static ssize_t apollo_kp_enable_store(struct device *dev, struct device_attribute *attr,
				    const char *buf, size_t count)
{
	int state;

	if (sscanf(buf, "%u", &state) != 1)
		return -EINVAL;

	if ((state != 1) && (state != 0))
		return -EINVAL;

	mutex_lock(&kp_enable_mutex);
	if (state != kp_enable) {
		if (state)
			enable_irq(NEC_REMOTE_IRQ_NO);
		else
			disable_irq(NEC_REMOTE_IRQ_NO);
		kp_enable = state;
	}
	mutex_unlock(&kp_enable_mutex);

	return strnlen(buf, count);
}

static DEVICE_ATTR(enable, S_IRUGO | S_IWUSR, apollo_kp_enable_show, apollo_kp_enable_store);



static int    hardware_init(struct platform_device *pdev)
{
	struct resource *mem; 
	unsigned int  control_value,status,data_value;
	int i;
	pin_config_t  *config=NULL;
	
	//step 0: set mutx to remote
	if (!(mem = platform_get_resource(pdev, IORESOURCE_IO, 0))) {
		pr_err("not define ioresource for remote keyboard.\n");
		return -1;
	}
	for (i=0;i<ARRAY_SIZE(pin_config);i++)
	{
		if(strcmp(pin_config[i].platform_name,mem->name)==0)
		{
			config=&pin_config[i] ;
			pr_dbg("got resource :%d\r\n",i);
			break;
		}
	}
	if(NULL==config)  return -1;
	SET_PERIPHS_REG_BITS(config->pin_mux,config->bit); //apollo
	
	//step 1 :set reg IR_DEC_CONTROL
#if defined(AML_A1H)	//for 24Mhz oscillator.
	control_value = (0xFA0 << 12) |0x13;
#else				//for 27Mhz oscillator.
	control_value = (0xFA0 << 12) |0x13; 
#endif
	WRITE_PERIPHS_REG(PREG_IR_DEC_BASE_GEN , control_value) ;	
      	control_value=READ_PERIPHS_REG(PREG_IR_DEC_CONTROL);
	WRITE_PERIPHS_REG(PREG_IR_DEC_CONTROL,control_value|IR_CONTROL_HOLD_LAST_KEY);
	
	status=READ_PERIPHS_REG(PREG_IR_DEC_FRAME_STATUS);       	
	data_value=READ_PERIPHS_REG(PREG_IR_DEC_FRAME_DATA);
	
       //step 2 : request nec_remote irq  & enable it             		
       return request_irq(NEC_REMOTE_IRQ_NO, apollo_kp_interrupt, IRQF_SHARED,"apollo-keypad", (void *)apollo_kp_interrupt);
	
	


}

static int __init apollo_kp_probe(struct platform_device *pdev)
{
	struct apollo_kp *apollo_kp;
	struct input_dev *input_dev;
	
	int i,ret;

	
	apollo_kp = kzalloc(sizeof(struct apollo_kp), GFP_KERNEL);
	input_dev = input_allocate_device();
	if (!apollo_kp || !input_dev) {
		kfree(apollo_kp);
		input_free_device(input_dev);
		return -ENOMEM;
	}
       
	platform_set_drvdata(pdev, apollo_kp);

	apollo_kp->input = input_dev;
	apollo_kp->release_delay=KEY_RELEASE_DELAY;
	apollo_kp->custom_code=0xff00;

	/* Disable the interrupt for the MPUIO keyboard */
	
	 

	/* get the irq and init timer*/
	pr_dbg("set drvdata completed\r\n");
	tasklet_enable(&kp_tasklet);
	kp_tasklet.data = (unsigned long) apollo_kp;
	setup_timer(&kp_timer, kp_timer_sr, 0) ;

	ret = device_create_file(&pdev->dev, &dev_attr_enable);
	if (ret < 0)
		goto err1;

	pr_dbg("device_create_file completed \r\n");
	/* setup input device */
	set_bit(EV_KEY, input_dev->evbit);
	set_bit(EV_REP, input_dev->evbit);

	
	for (i = 0; i<KEY_MAX; i++)
	{
		set_bit( i, input_dev->keybit);
	}
	clear_bit(0,input_dev->keybit);
	input_dev->name = "apollo-keypad";
	input_dev->phys = "apollo-keypad/input0";
	//input_dev->cdev.dev = &pdev->dev;
	//input_dev->private = apollo_kp;
	input_dev->dev.parent = &pdev->dev;
	
	input_dev->id.bustype = BUS_ISA;
	input_dev->id.vendor = 0x0001;
	input_dev->id.product = 0x0001;
	input_dev->id.version = 0x0100;
#ifdef  CONFIG_KEYPADS_APOLLO_REPEAT_ENABLE	
	input_dev->rep[REP_DELAY]=200;
	input_dev->rep[REP_PERIOD]=apollo_kp->release_delay-240;
#else
	input_dev->rep[REP_DELAY]=0xffffffff;
	input_dev->rep[REP_PERIOD]=0xffffffff;
#endif
	
	
	
	input_dev->keycode = apollo_keycode;
	input_dev->keycodesize = sizeof(unsigned short);
	input_dev->keycodemax = 0x1ff;
	
	ret = input_register_device(apollo_kp->input);
	if (ret < 0) {
		printk(KERN_ERR "Unable to register apollo-keypad input device\n");
		goto err2;
	}
	pr_dbg("input_register_device completed \r\n");
	if(hardware_init(pdev))  goto err3;


	return 0;
err3:
	 free_irq(NEC_REMOTE_IRQ_NO,apollo_kp_interrupt);
 	input_unregister_device(apollo_kp->input);
	input_dev = NULL;
err2:
	device_remove_file(&pdev->dev, &dev_attr_enable);
err1:
	
	kfree(apollo_kp);
	input_free_device(input_dev);

	return -EINVAL;
}

static int apollo_kp_remove(struct platform_device *pdev)
{
	struct apollo_kp *apollo_kp = platform_get_drvdata(pdev);

	/* disable keypad interrupt handling */
	pr_dbg("remove apollo kpads \r\n");
	tasklet_disable(&kp_tasklet);
	tasklet_kill(&kp_tasklet);
	
	/* unregister everything */
	input_unregister_device(apollo_kp->input);
      device_remove_file(&pdev->dev, &dev_attr_enable);
	free_irq(NEC_REMOTE_IRQ_NO,apollo_kp_interrupt);
	input_free_device(apollo_kp->input); 
	kfree(apollo_kp);

	return 0;
}

static struct platform_driver apollo_kp_driver = {
	.probe		= apollo_kp_probe,
	.remove		= apollo_kp_remove,
	.suspend	= NULL,
	.resume		= NULL,
	.driver		= {
		.name	= "apollo-kp",
	},
};

static int __devinit apollo_kp_init(void)
{
	printk(KERN_INFO "apollo Keypad Driver\n");
	
	return platform_driver_register(&apollo_kp_driver);
}

static void __exit apollo_kp_exit(void)
{
	printk(KERN_INFO "apollo Keypad exit \n");
	platform_driver_unregister(&apollo_kp_driver);
}

module_init(apollo_kp_init);
module_exit(apollo_kp_exit);

MODULE_AUTHOR("jianfeng_wang");
MODULE_DESCRIPTION("apollo Keypad Driver");
MODULE_LICENSE("GPL");

