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

#define REMOTE_STATUS_WAIT       0
#define REMOTE_STATUS_LEADER     1
#define REMOTE_STATUS_DATA       2
#define REMOTE_STATUS_SYNC       3

#if  defined(CONFIG_KEYPADS_APOLLO_DECODER_SW_SKYWORTH)
#define  LEADER_ACTIVE_TIME   	0x02bc012c	//300-700
#define	BIT0_TIME			  	0x00460028	//40-70
#define	BIT1_TIME			  	0x00ff0064	//100-130
#define	REPEAT_LEADER_TIME	0x02bc012c 	//300-700
#endif
//add other manufacture config data here.


			

#ifdef   DEBUG
#define pr_dbg(fmt, args...) printk(KERN_ALERT "apollo kpads: " fmt, ##args)
//#define pr_err(fmt, args...) printk(KERN_ERR "apollo kpads: " fmt, ## args)
#else   
#define    pr_dbg(fmt, args...)
//#define 	  pr_err(fmt, args...)
#endif

static void apollo_kp_tasklet(unsigned long);

DECLARE_TASKLET_DISABLED(kp_tasklet, apollo_kp_tasklet, 0);
static DEFINE_MUTEX(kp_enable_mutex);
static DEFINE_MUTEX(kp_flag_mutex);
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

	unsigned int cur_keycode;
	unsigned int delay;
	unsigned int   step;
	unsigned int   bit_num;
	unsigned int	last_jiffies;
	unsigned int   repeate_flag;
	unsigned int 	time_window[8];
	int			last_pulse_width;
	int			repeat_time_count;
};


static int kp_enable = 1;
static struct timer_list kp_timer;

static  int NEC_REMOTE_IRQ_NO=AM_ISA_GEN_IRQ(15);

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
void reset_kpd(unsigned long data)
{
	struct apollo_kp *apollo_kp_data = (struct apollo_kp *) data;

	SET_PERIPHS_REG_BITS(PREG_IR_DEC_CONTROL,1);
	CLEAR_PERIPHS_REG_BITS(PREG_IR_DEC_CONTROL,1);
	apollo_kp_data->step   = REMOTE_STATUS_WAIT ;
	
}
void	kp_timer_sr(unsigned long data)
{
	struct apollo_kp *apollo_kp_data = (struct apollo_kp *) data;

	pr_dbg("key 0x%x release\r\n",apollo_kp_data->cur_keycode);
	input_report_key(apollo_kp_data->input, (apollo_kp_data->cur_keycode>>16)&0xff, 0);
	apollo_kp_data->step   = REMOTE_STATUS_WAIT ;
	//reset_kpd(data);
}
static int  get_pulse_width(unsigned long data)
{
	struct apollo_kp *apollo_kp_data = (struct apollo_kp *) data;
	int  pulse_width,tmp;
	pulse_width     = ( (READ_PERIPHS_REG( PREG_IR_DEC_CONTROL)) & 0x1FFF0000 ) >> 16 ;
	tmp=pulse_width;
	pulse_width= pulse_width < apollo_kp_data->last_pulse_width ? (0x1fff - apollo_kp_data->last_pulse_width) +  pulse_width : \
															pulse_width - apollo_kp_data->last_pulse_width ;			
	apollo_kp_data->last_pulse_width=tmp;
	return pulse_width;
}
	
static inline void kbd_software_mode_remote_wait(unsigned long data)
{
	unsigned short pulse_width;
	struct apollo_kp *apollo_kp_data = (struct apollo_kp *) data;
	
    	pulse_width     = get_pulse_width(data) ;
	apollo_kp_data->step   = REMOTE_STATUS_LEADER;
	apollo_kp_data->cur_keycode = 0 ;
	apollo_kp_data->bit_num = 32 ;
	//pr_dbg("wait,pulsewidth:%d\r\n",pulse_width);
}
static inline void kbd_software_mode_remote_leader(unsigned long data)
{
	unsigned short pulse_width;
	struct apollo_kp *apollo_kp_data = (struct apollo_kp *) data;
	
   	pulse_width     = get_pulse_width(data) ;
	//pr_dbg("leader,pulse width %d\r\n",pulse_width);
	if((pulse_width > apollo_kp_data->time_window[0]) && (pulse_width <apollo_kp_data->time_window[1])) {
	    	apollo_kp_data->step   = REMOTE_STATUS_DATA;
		//pr_dbg("fit in leader window\r\n");	
	}
    	else {
      		apollo_kp_data->step    = REMOTE_STATUS_WAIT ;
		//pr_dbg("pulse width 0x%x,leader return wait\r\n",pulse_width);		
		//reset_kpd(data);
    	}

	//RemoteStatus   = REMOTE_STATUS_DATA;
	apollo_kp_data->cur_keycode = 0 ;
	apollo_kp_data->bit_num = 32 ;
}
static inline void kbd_software_mode_remote_send_key(unsigned long data)
{
  	struct apollo_kp *apollo_kp_data = (struct apollo_kp *) data;
    	if(apollo_kp_data->repeate_flag)
    	{
    		pr_dbg("report key :0x%x ====repeat\r\n",apollo_kp_data->cur_keycode);
		input_report_key(apollo_kp_data->input, (apollo_kp_data->cur_keycode>>16)&0xff, 2);
    	}else{
    		pr_dbg("report key :0x%x ====down\r\n",apollo_kp_data->cur_keycode);
		input_report_key(apollo_kp_data->input, (apollo_kp_data->cur_keycode>>16)&0xff, 1);	
    	}
		
    	apollo_kp_data->step   = REMOTE_STATUS_SYNC ;
}
static inline void kbd_software_mode_remote_data(unsigned long data)
{
	unsigned short pulse_width;
	struct apollo_kp *apollo_kp_data = (struct apollo_kp *) data;
       
    	pulse_width     = get_pulse_width(data) ;
	//pr_dbg("data,pulse width %d\r\n",pulse_width);	
	apollo_kp_data->step   = REMOTE_STATUS_DATA ;
	
	if((pulse_width > apollo_kp_data->time_window[2]) && (pulse_width < apollo_kp_data->time_window[3])) {
            	apollo_kp_data->bit_num--;
	}
	else if((pulse_width > apollo_kp_data->time_window[4]) && (pulse_width < apollo_kp_data->time_window[5])) {
        	apollo_kp_data->cur_keycode |= 1<<(32-apollo_kp_data->bit_num) ;       //1
        	apollo_kp_data->bit_num--;
		//pr_dbg("current data:0x%x\r\n",apollo_kp_data->cur_keycode);		
	}
    	else {
		pr_dbg("data return wait:0x%x \r\n",apollo_kp_data->bit_num );		
      		apollo_kp_data->step   = REMOTE_STATUS_WAIT ;
    	}
    	if(apollo_kp_data->bit_num == 0)
    	{
     	 	apollo_kp_data->repeate_flag= 0;
		apollo_kp_data->repeat_time_count=0;
        	kbd_software_mode_remote_send_key(data);
		pr_dbg("send data:0x%x\r\n",apollo_kp_data->cur_keycode );	
    	}
	
}
static inline void kbd_software_mode_remote_sync(unsigned long data)
{
	unsigned short pulse_width;
	struct apollo_kp *apollo_kp_data = (struct apollo_kp *) data;
    
   	pulse_width     = get_pulse_width(data) ;
	
	if((pulse_width > apollo_kp_data->time_window[6]) && (pulse_width < apollo_kp_data->time_window[7])) {
        	apollo_kp_data->repeate_flag=1;
		apollo_kp_data->repeat_time_count++;
		if(1!=apollo_kp_data->repeat_time_count)
        	kbd_software_mode_remote_send_key(data);
	}
    	apollo_kp_data->step  = REMOTE_STATUS_SYNC ;
	kp_timer.data=(unsigned long)apollo_kp_data;
	mod_timer(&kp_timer,jiffies+HZ/5);//6
	
}
static void apollo_kp_tasklet(unsigned long data)
{
	struct apollo_kp *apollo_kp_data = (struct apollo_kp *) data;
	int	   current_jiffies=jiffies;
    	if((current_jiffies -apollo_kp_data->last_jiffies > 20) && (apollo_kp_data->step  <  REMOTE_STATUS_SYNC)) {
      		  apollo_kp_data->step = REMOTE_STATUS_WAIT ;
    	}
    	apollo_kp_data->last_jiffies = current_jiffies ;  //ignore a little nsecs.
	//pr_dbg("current step:%d\r\n",apollo_kp_data->step);	
    	switch( apollo_kp_data->step)
    	{
        case REMOTE_STATUS_WAIT:
            kbd_software_mode_remote_wait(data) ;
            break;
        case REMOTE_STATUS_LEADER:
            kbd_software_mode_remote_leader(data);
            break;
        case REMOTE_STATUS_DATA:
            kbd_software_mode_remote_data(data) ;
            break;
        case REMOTE_STATUS_SYNC:
            kbd_software_mode_remote_sync(data) ;
            break;
        default:
            break;
    	}
}
static irqreturn_t apollo_kp_interrupt(int irq, void *dev_id)
{
	/* disable keyboard interrupt and schedule for handling*/
	//int pulse_width;
	pr_dbg("===trigger one sw  kpads interupt \r\n");
	tasklet_schedule(&kp_tasklet); 
	/*unsigned long data;
	struct apollo_kp *apollo_kp_data = (struct apollo_kp *) kp_tasklet.data;
	int	   current_jiffies=jiffies;

	data=kp_tasklet.data;
    	if((current_jiffies -apollo_kp_data->last_jiffies > 20) && (apollo_kp_data->step  <  REMOTE_STATUS_SYNC)) {
      		  apollo_kp_data->step = REMOTE_STATUS_WAIT ;
    	}
    	apollo_kp_data->last_jiffies = current_jiffies ;  //ignore a little nsecs.
	//pr_dbg("current step:%d\r\n",apollo_kp_data->step);	
    	switch( apollo_kp_data->step)
    	{
        case REMOTE_STATUS_WAIT:
            kbd_software_mode_remote_wait(data) ;
            break;
        case REMOTE_STATUS_LEADER:
            kbd_software_mode_remote_leader(data);
            break;
        case REMOTE_STATUS_DATA:
            kbd_software_mode_remote_data(data) ;
            break;
        case REMOTE_STATUS_SYNC:
            kbd_software_mode_remote_sync(data) ;
            break;
        default:
            break;
    	}*/

	return IRQ_HANDLED;
}

static int    hardware_init(struct platform_device *pdev)
{
	struct resource *mem; 
	unsigned int  control_value;
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
	control_value = 	0<<28		 | //filter enable
					(0xFA0 << 12) |
					0x13;
	WRITE_PERIPHS_REG(PREG_IR_DEC_BASE_GEN , control_value) ;	
	control_value=0x578;
	WRITE_PERIPHS_REG(PREG_IR_DEC_CONTROL,control_value);
	pr_dbg("read back from PREG_IR_DEC_CONTROL:0x%x\r\n",READ_PERIPHS_REG(PREG_IR_DEC_CONTROL));
	return request_irq(NEC_REMOTE_IRQ_NO, apollo_kp_interrupt, IRQF_SHARED,"apollo-keypad", (void *)apollo_kp_interrupt);
	
}

static DEVICE_ATTR(enable, S_IRUGO | S_IWUSR, apollo_kp_enable_show, apollo_kp_enable_store);


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
	apollo_kp->step = REMOTE_STATUS_WAIT;
	apollo_kp->time_window[0]=LEADER_ACTIVE_TIME&0xffff;
	apollo_kp->time_window[1]=(LEADER_ACTIVE_TIME>>16)&0xffff;
	apollo_kp->time_window[2]=BIT0_TIME&0xffff;
	apollo_kp->time_window[3]=(BIT0_TIME>>16)&0xffff;
	apollo_kp->time_window[4]=BIT1_TIME&0xffff;
	apollo_kp->time_window[5]=(BIT1_TIME>>16)&0xffff;
	apollo_kp->time_window[6]=REPEAT_LEADER_TIME&0xffff;
	apollo_kp->time_window[7]=(REPEAT_LEADER_TIME>>16)&0xffff;
	apollo_kp->last_pulse_width=0;
	/* Disable the interrupt for the MPUIO keyboard */
	
	 
	apollo_kp->last_jiffies=0xffffffff;
	
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
#ifdef CONFIG_KEYPADS_APOLLO_REPEAT_ENABLE	
	input_dev->rep[REP_DELAY]=100;
	input_dev->rep[REP_PERIOD]=50;
#else
	input_dev->rep[REP_DELAY]=0xffffffff;
	input_dev->rep[REP_PERIOD]=0xffffffff;
#endif
	//input_dev->keycode = NULL;
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

