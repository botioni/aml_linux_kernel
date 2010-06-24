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
#include <mach/am_regs.h>
#include <linux/major.h>
#include <linux/slab.h>
#include <asm/uaccess.h>
#include "amkbd_remote.h"
#undef NEW_BOARD_LEARNING_MODE

 #define IR_CONTROL_HOLD_LAST_KEY 		(1<<6)
 #define IR_CONTROL_DECODER_MODE		(3<<7)
 #define IR_CONTROL_SKIP_HEADER		(1<<7)
 #define IR_CONTROL_RESET         			(1<<0)


#define   KEY_RELEASE_DELAY    200


type_printk   input_dbg ;
static DEFINE_MUTEX(kp_enable_mutex);
static DEFINE_MUTEX(kp_file_mutex);
static void apollo_kp_tasklet(unsigned long);
static int kp_enable ;
static  int NEC_REMOTE_IRQ_NO=INT_REMOTE;


DECLARE_TASKLET_DISABLED(kp_tasklet, apollo_kp_tasklet, 0);

typedef  struct {
	char		     *platform_name;
	unsigned int  pin_mux;
	unsigned int  bit;
}pin_config_t;

static  pin_config_t  pin_config={
			.platform_name="meson-1",
			.pin_mux=1,  //need fix 
			.bit= 2,
};





static struct apollo_kp   *gp_apollo_kp=NULL;
char    *remote_log_buf;
int  remote_printk(const char *fmt, ...)
{
	va_list args;
	int r;
	
	if (gp_apollo_kp->debug_enable==0)  return 0;
	va_start(args, fmt);
	r = vprintk(fmt, args);
	va_end(args);
	return r;
}

void	kp_timer_sr(unsigned long data)
{
	struct apollo_kp *apollo_kp_data=(struct apollo_kp *)data;
	input_report_key(apollo_kp_data->input,(apollo_kp_data->cur_keycode>>16)&0xff ,0);
	input_dbg("key release 0x%02x\r\n", (apollo_kp_data->cur_keycode>>16)&0xff);
	if(apollo_kp_data->work_mode==REMOTE_WORK_MODE_SW)
		apollo_kp_data->step   = REMOTE_STATUS_WAIT ;
}


static irqreturn_t apollo_kp_interrupt(int irq, void *dev_id)
{
	/* disable keyboard interrupt and schedule for handling */
//	input_dbg("===trigger one  kpads interupt \r\n");
	tasklet_schedule(&kp_tasklet);

	return IRQ_HANDLED;
}


static inline int apollo_kp_hw_reprot_key(struct apollo_kp *apollo_kp_data )
{
	int  key_index;
	unsigned  int  status,scan_code;
	static  int last_scan_code,key_hold;
	static  int last_custom_code;
	
	 
	// 1		get  scan code
	scan_code=READ_MPEG_REG(IR_DEC_FRAME);
	status=READ_MPEG_REG(IR_DEC_STATUS);
	
	key_index=0 ;
	key_hold=-1 ;
	if(scan_code)  //key first press
	{
		last_custom_code=scan_code&0xffff;
		if(apollo_kp_data->custom_code != last_custom_code )
		{
		       input_dbg("Wrong custom code is 0x%04x\n", last_custom_code);
			return -1;
		}
		if(apollo_kp_data->timer.expires > jiffies){
			input_report_key(apollo_kp_data->input,(apollo_kp_data->cur_keycode>>16)&0xff ,0);
			input_dbg("key release 0x%02x\r\n", (apollo_kp_data->cur_keycode>>16)&0xff);			
			}
		input_report_key(apollo_kp_data->input,(scan_code>>16)&0xff,1);
		input_dbg("key pressed,scan code :0x%x\r\n",scan_code);
	}
	else if(scan_code==0 && status&0x1) //repeate key
	{
		scan_code=last_scan_code;
		if(apollo_kp_data->custom_code != last_custom_code )
		{
			return -1;
		}
              if(apollo_kp_data->repeat_enable){
		    input_report_key(apollo_kp_data->input,(scan_code>>16)&0xff,2);
		    input_dbg("key repeate,scan code :0x%x\r\n",scan_code);
                }
              else{
                  if(apollo_kp_data->timer.expires > jiffies)
                  	mod_timer(&apollo_kp_data->timer,jiffies+msecs_to_jiffies(apollo_kp_data->release_delay));
                  return -1;
                }
	}
	last_scan_code=scan_code;
	apollo_kp_data->cur_keycode=last_scan_code;
	apollo_kp_data->timer.data=(unsigned long)apollo_kp_data;	
	mod_timer(&apollo_kp_data->timer,jiffies+msecs_to_jiffies(apollo_kp_data->release_delay));
	return 0 ;

}

static void apollo_kp_tasklet(unsigned long data)
{
	struct apollo_kp *apollo_kp_data = (struct apollo_kp *) data;

	if(apollo_kp_data->work_mode==REMOTE_WORK_MODE_HW)
	{
		apollo_kp_hw_reprot_key(apollo_kp_data);
	}else{
		apollo_kp_sw_reprot_key(data);
	}
}
static ssize_t apollo_kp_log_buffer_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	int ret =0;
	ret=sprintf(buf, "%s\n", remote_log_buf);
	//printk(remote_log_buf);
	remote_log_buf[0]='\0';
	return ret ;
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
static DEVICE_ATTR(log_buffer, S_IRUGO | S_IWUSR, apollo_kp_log_buffer_show, NULL);

/*****************************************************************
**
** func : hardware init 
**		 in this function will do pin configuration and and initialize for hardware 
**		 decoder mode .
**
********************************************************************/
static int    hardware_init(struct platform_device *pdev)
{
	struct resource *mem; 
	unsigned int  control_value,status,data_value;
	int i;
	pin_config_t  *config=NULL;
	
	//step 0: set mutx to remote
	
	
	config=&pin_config ;
	
	if(NULL==config)  return -1;
	WRITE_MPEG_REG_BITS(config->pin_mux,1,config->bit,1); //apollo
	
	//step 1 :set reg IR_DEC_CONTROL
									//for 27Mhz oscillator.
	control_value = 3<<28|(0xFA0 << 12) |0x13; 

	WRITE_MPEG_REG(IR_DEC_REG0 , control_value) ;	
      	control_value=READ_MPEG_REG(IR_DEC_REG1);
	WRITE_MPEG_REG(IR_DEC_REG1,control_value|IR_CONTROL_HOLD_LAST_KEY);
	
	status=READ_MPEG_REG(IR_DEC_STATUS);       	
	data_value=READ_MPEG_REG(IR_DEC_FRAME);
	
       //step 2 : request nec_remote irq  & enable it             		
       return request_irq(NEC_REMOTE_IRQ_NO, apollo_kp_interrupt, IRQF_SHARED,"apollo-keypad", (void *)apollo_kp_interrupt);
	
	


}
static   int   
work_mode_config(int  work_mode)
{
	unsigned int  control_value;
		
	if(work_mode==REMOTE_WORK_MODE_HW)
	{
		control_value=0xbe40; //ignore  custom code .
		WRITE_MPEG_REG(IR_DEC_REG1,control_value|IR_CONTROL_HOLD_LAST_KEY);
	}else{
		control_value=0x8578;
		WRITE_MPEG_REG(IR_DEC_REG1,control_value);
	}
	return 0;
}
static int
remote_config_open(struct inode *inode, struct file *file)
{
	file->private_data = gp_apollo_kp;
	return 0;
}
static int
remote_config_ioctl(struct inode *inode, struct file *filp,
                 unsigned int cmd, unsigned long args)
{

	struct apollo_kp   *kp=(struct apollo_kp*)filp->private_data;
	void  __user* argp =(void __user*)args;
	unsigned int   val;

	disable_irq(NEC_REMOTE_IRQ_NO);
	if(args)
	{
		copy_from_user(&val,argp,sizeof(unsigned long));
	}
	mutex_lock(&kp_file_mutex);
	//cmd input 
	switch(cmd)
	{
		// 1 set part
		case  REMOTE_IOC_SET_REPEAT_ENABLE:
		copy_from_user(&kp->repeat_enable,argp,sizeof(long));	
		break;	
		case  REMOTE_IOC_SET_DEBUG_ENABLE:
		copy_from_user(&kp->debug_enable,argp,sizeof(long));	
		break;	
		case  REMOTE_IOC_SET_MODE:
		copy_from_user(&kp->work_mode,argp,sizeof(long));	
		break;
		case  REMOTE_IOC_SET_BIT_COUNT:
		copy_from_user(&kp->bit_count,argp,sizeof(long));		
		break;	
		case  REMOTE_IOC_SET_CUSTOMCODE:
		copy_from_user(&kp->custom_code,argp,sizeof(long));
		break;
		case  REMOTE_IOC_SET_REG_BASE_GEN:
		WRITE_MPEG_REG(IR_DEC_REG0,val);
		break;
		case REMOTE_IOC_SET_REG_CONTROL:
		WRITE_MPEG_REG(IR_DEC_REG1,val);
		break;
		case REMOTE_IOC_SET_REG_LEADER_ACT:
		WRITE_MPEG_REG(IR_DEC_LDR_ACTIVE,val);
		break;
		case REMOTE_IOC_SET_REG_LEADER_IDLE:
		WRITE_MPEG_REG(IR_DEC_LDR_IDLE,val);	
		break;
		case REMOTE_IOC_SET_REG_REPEAT_LEADER:
		WRITE_MPEG_REG(IR_DEC_LDR_REPEAT,val);		
		break;	
		case REMOTE_IOC_SET_REG_BIT0_TIME:
		WRITE_MPEG_REG(IR_DEC_BIT_0,val);		
		break;
		case REMOTE_IOC_SET_RELEASE_DELAY:
		copy_from_user(&kp->release_delay,argp,sizeof(long));	
		break;
		//SW
		case REMOTE_IOC_SET_TW_LEADER_ACT:
		kp->time_window[0]=val&0xffff;
		kp->time_window[1]=(val>>16)&0xffff;	
		break;
		case REMOTE_IOC_SET_TW_BIT0_TIME:
		kp->time_window[2]=val&0xffff;
		kp->time_window[3]=(val>>16)&0xffff;	
		break;	
		case REMOTE_IOC_SET_TW_BIT1_TIME:
		kp->time_window[4]=val&0xffff;
		kp->time_window[5]=(val>>16)&0xffff;	
		break;	
		case REMOTE_IOC_SET_TW_REPEATE_LEADER:
		kp->time_window[6]=val&0xffff;
		kp->time_window[7]=(val>>16)&0xffff;	
		break;	
		// 2 get  part
		case REMOTE_IOC_GET_REG_BASE_GEN:
		val=READ_MPEG_REG(IR_DEC_REG0);
		break;
		case REMOTE_IOC_GET_REG_CONTROL:
		val=READ_MPEG_REG(IR_DEC_REG1);	
		break;
		case REMOTE_IOC_GET_REG_LEADER_ACT:
		val=READ_MPEG_REG(IR_DEC_LDR_ACTIVE);
		break;	
		case REMOTE_IOC_GET_REG_LEADER_IDLE:
		val=READ_MPEG_REG(IR_DEC_LDR_IDLE);
		break;
		case REMOTE_IOC_GET_REG_REPEAT_LEADER:
		val=READ_MPEG_REG(IR_DEC_LDR_REPEAT);
		break;
		case REMOTE_IOC_GET_REG_BIT0_TIME:
		val=READ_MPEG_REG(IR_DEC_BIT_0);	
		break;	
		case REMOTE_IOC_GET_REG_FRAME_DATA:
		val=READ_MPEG_REG(IR_DEC_FRAME);		
		break;
		case REMOTE_IOC_GET_REG_FRAME_STATUS:
		val=READ_MPEG_REG(IR_DEC_STATUS);	
		break;	
		//sw
		case REMOTE_IOC_GET_TW_LEADER_ACT:
		val=kp->time_window[0]|(kp->time_window[1]<<16);	
		break;
		case REMOTE_IOC_GET_TW_BIT0_TIME:
		val=kp->time_window[2]|(kp->time_window[3]<<16);	
		break;
		case REMOTE_IOC_GET_TW_BIT1_TIME:
		val=kp->time_window[4]|(kp->time_window[5]<<16);	
		break;	
		case REMOTE_IOC_GET_TW_REPEATE_LEADER:
		val=kp->time_window[6]|(kp->time_window[7]<<16);		
		break;	
	}
	//output result 
	switch(cmd)
	{
		case REMOTE_IOC_SET_REPEAT_ENABLE:
		if (kp->repeat_enable)
		{
			kp->input->rep[REP_DELAY]=1000;
			kp->input->rep[REP_PERIOD]=250;
		}else{
			kp->input->rep[REP_DELAY]=0xffffffff;
			kp->input->rep[REP_PERIOD]=0xffffffff;
		}
		break;
		case REMOTE_IOC_SET_MODE:
		work_mode_config(kp->work_mode);	
		break;	
		case REMOTE_IOC_GET_REG_BASE_GEN:
		case REMOTE_IOC_GET_REG_CONTROL:
		case REMOTE_IOC_GET_REG_LEADER_ACT	:
		case REMOTE_IOC_GET_REG_LEADER_IDLE:
		case REMOTE_IOC_GET_REG_REPEAT_LEADER:
		case REMOTE_IOC_GET_REG_BIT0_TIME:
		case REMOTE_IOC_GET_REG_FRAME_DATA:
		case REMOTE_IOC_GET_REG_FRAME_STATUS:	
		case REMOTE_IOC_GET_TW_LEADER_ACT	:
		case REMOTE_IOC_GET_TW_BIT0_TIME:
		case REMOTE_IOC_GET_TW_BIT1_TIME:
		case REMOTE_IOC_GET_TW_REPEATE_LEADER:	
		copy_to_user(argp,&val,sizeof(long));
		break;
	}
	mutex_unlock(&kp_file_mutex);
	enable_irq(NEC_REMOTE_IRQ_NO);
	return  0;
}
static int 
remote_config_release(struct inode *inode, struct file *file)
{
	file->private_data=NULL;
	return 0;
	
}
static const struct file_operations remote_fops = {
	.owner		= THIS_MODULE,
	.open		=remote_config_open,  
	.ioctl		= remote_config_ioctl,
	.release		= remote_config_release, 	
};
static int  register_remote_dev(struct apollo_kp  *kp)
{
	int ret=0;
	strcpy(kp->config_name,"amremote");
	ret=register_chrdev(0,kp->config_name,&remote_fops);
	if(ret <=0) 
	{
		printk("register char dev tv error\r\n");
		return  ret ;
	}
	kp->config_major=ret;
	printk("remote config major:%d\r\n",ret);
	kp->config_class=class_create(THIS_MODULE,kp->config_name);
	kp->config_dev=device_create(kp->config_class,NULL,MKDEV(kp->config_major,0),NULL,kp->config_name);
	return ret;
}
static int __init apollo_kp_probe(struct platform_device *pdev)
{
	struct apollo_kp *apollo_kp;
	struct input_dev *input_dev;
	
	int i,ret;

	kp_enable=1;
	apollo_kp = kzalloc(sizeof(struct apollo_kp), GFP_KERNEL);
	input_dev = input_allocate_device();
	if (!apollo_kp || !input_dev) {
		kfree(apollo_kp);
		input_free_device(input_dev);
		return -ENOMEM;
	}
	gp_apollo_kp=apollo_kp;
	apollo_kp->debug_enable=0;

   	input_dbg=remote_printk;
	platform_set_drvdata(pdev, apollo_kp);
	apollo_kp->work_mode=REMOTE_WORK_MODE_HW;
	apollo_kp->input = input_dev;
	apollo_kp->release_delay=KEY_RELEASE_DELAY;
	apollo_kp->custom_code=0xff00;
	apollo_kp->bit_count=32;  //default 32bit for sw mode.
	apollo_kp->last_jiffies=0xffffffff;
	apollo_kp->last_pulse_width=0;
	

	apollo_kp->step = REMOTE_STATUS_WAIT;
	apollo_kp->time_window[0]=0x1;
	apollo_kp->time_window[1]=0x1;
	apollo_kp->time_window[2]=0x1;
	apollo_kp->time_window[3]=0x1;
	apollo_kp->time_window[4]=0x1;
	apollo_kp->time_window[5]=0x1;
	apollo_kp->time_window[6]=0x1;
	apollo_kp->time_window[7]=0x1;
	/* Disable the interrupt for the MPUIO keyboard */
	
	 

	/* get the irq and init timer*/
	input_dbg("set drvdata completed\r\n");
	tasklet_enable(&kp_tasklet);
	kp_tasklet.data = (unsigned long) apollo_kp;
	setup_timer(&apollo_kp->timer, kp_timer_sr, 0) ;

	ret = device_create_file(&pdev->dev, &dev_attr_enable);
	if (ret < 0)
		goto err1;
       ret=device_create_file(&pdev->dev, &dev_attr_log_buffer);
	if(ret<0)
	{
		device_remove_file(&pdev->dev, &dev_attr_enable);
		goto err1;
	}
	
	input_dbg("device_create_file completed \r\n");
	/* setup input device */
	set_bit(EV_KEY, input_dev->evbit);
	set_bit(EV_REP, input_dev->evbit);

	
	for (i = 0; i<KEY_MAX; i++)
	{
		set_bit( i, input_dev->keybit);
	}
	//clear_bit(0,input_dev->keybit);
	input_dev->name = "apollo-keypad";
	input_dev->phys = "apollo-keypad/input0";
	//input_dev->cdev.dev = &pdev->dev;
	//input_dev->private = apollo_kp;
	input_dev->dev.parent = &pdev->dev;
	
	input_dev->id.bustype = BUS_ISA;
	input_dev->id.vendor = 0x0001;
	input_dev->id.product = 0x0001;
	input_dev->id.version = 0x0100;
	apollo_kp->repeat_enable=0;
	
	
	input_dev->rep[REP_DELAY]=0xffffffff;
	input_dev->rep[REP_PERIOD]=0xffffffff;

	
	input_dev->keycodesize = sizeof(unsigned short);
	input_dev->keycodemax = 0x1ff;
	
	ret = input_register_device(apollo_kp->input);
	if (ret < 0) {
		printk(KERN_ERR "Unable to register apollo-keypad input device\n");
		goto err2;
	}
	input_dbg("input_register_device completed \r\n");
	if(hardware_init(pdev))  goto err3;
	
  	register_remote_dev(gp_apollo_kp);
	remote_log_buf = (char*)__get_free_pages(GFP_KERNEL,REMOTE_LOG_BUF_ORDER);
	remote_log_buf[0]='\0';
	printk("physical address:0x%x\n",(unsigned int )virt_to_phys(remote_log_buf));
	return 0;
err3:
	 free_irq(NEC_REMOTE_IRQ_NO,apollo_kp_interrupt);
 	input_unregister_device(apollo_kp->input);
	input_dev = NULL;
err2:
	device_remove_file(&pdev->dev, &dev_attr_enable);
	device_remove_file(&pdev->dev, &dev_attr_log_buffer);
err1:
	
	kfree(apollo_kp);
	input_free_device(input_dev);

	return -EINVAL;
}

static int apollo_kp_remove(struct platform_device *pdev)
{
	struct apollo_kp *apollo_kp = platform_get_drvdata(pdev);

	/* disable keypad interrupt handling */
	input_dbg("remove apollo kpads \r\n");
	tasklet_disable(&kp_tasklet);
	tasklet_kill(&kp_tasklet);
	
	/* unregister everything */
	input_unregister_device(apollo_kp->input);
	free_pages((unsigned long)remote_log_buf,REMOTE_LOG_BUF_ORDER);
     	device_remove_file(&pdev->dev, &dev_attr_enable);
	device_remove_file(&pdev->dev, &dev_attr_log_buffer);
	free_irq(NEC_REMOTE_IRQ_NO,apollo_kp_interrupt);
	input_free_device(apollo_kp->input); 


	unregister_chrdev(apollo_kp->config_major,apollo_kp->config_name);
	if(apollo_kp->config_class)
	{
		if(apollo_kp->config_dev)
		device_destroy(apollo_kp->config_class,MKDEV(apollo_kp->config_major,0));
		class_destroy(apollo_kp->config_class);
	}

	kfree(apollo_kp);
	gp_apollo_kp=NULL ;	
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

