/*
 * Amlogic M1 
 * frame buffer driver-----------HDMI_TX
 * Copyright (C) 2010 Amlogic, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the named License,
 * or any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 */


#include <linux/version.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/mm.h>
#include <linux/major.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/cdev.h>
#include <linux/proc_fs.h> 
#include <asm/uaccess.h>

#include <linux/osd/apollo_main.h>
#include <linux/osd/apollodev.h>

#include "hdmi_info_global.h"
#include "hdmi_tx_module.h"
void hdmi_hw_init();
void hdmi_setup_irq();

#define ADD_DEVICE

#define DEVICE_NAME "amhdmitx"
#define HDMI_TX_COUNT 32
#define HDMI_TX_POOL_NUM  6
#define HDMI_TX_RESOURCE_NUM 4


#ifdef DEBUG
#define pr_dbg(fmt, args...) printk(KERN_DEBUG "amhdmitx: " fmt, ## args)
#else
#define pr_dbg(fmt, args...)
#endif
#define pr_error(fmt, args...) printk(KERN_ERR "amhdmitx: " fmt, ## args)


static hdmitx_dev_t hdmitx_device;

static dev_t hdmitx_id;
static struct class *hdmitx_class;
static struct device *hdmitx_dev;

static HDMI_TX_INFO_t hdmi_info;

#define HDMI_M1A 1
static hdmi_chip_type = 0;
/*****************************
*    hdmitx attr management :
*    enable
*    mode
*    reg
******************************/
static  int  set_disp_mode(char *mode)
{
    int ret=-1;
    if(strncmp(mode,"480p",4)==0){
        ret = hdmitx_set_display(&hdmitx_device, HDMI_480p60);
    }
    else if(strncmp(mode,"1080p",5)==0){
        ret = hdmitx_set_display(&hdmitx_device, HDMI_1080p60);
    }
    return ret;
}

static int set_disp_mode_auto()
{
    int ret=-1;
    vinfo_t *info = get_current_vinfo();
    HDMI_Video_Codes_t vic;
    vic = hdmitx_edid_get_VIC(&hdmitx_device, info->name);
    hdmitx_device.cur_VIC = HDMI_Unkown;
    if(vic != HDMI_Unkown ){
        ret = hdmitx_set_display(&hdmitx_device, vic);
        if(ret>=0){
            hdmitx_device.cur_VIC = vic;    
        }
    }
    return ret;
}    

/*mode attr*/
static ssize_t show_mode(struct device * dev, struct device_attribute *attr, char * buf)
{
    int pos=0;
    pos+=snprintf(buf+pos, PAGE_SIZE, "VIC:%d\r\n", hdmitx_device.cur_VIC);
    return pos;    
}
    
static ssize_t store_mode(struct device * dev, struct device_attribute *attr, const char * buf)
{
    set_disp_mode(buf);
    return 0;    
}

/*edid attr*/
static ssize_t show_edid(struct device *dev, struct device_attribute *attr, char *buf)
{
    return hdmitx_edid_dump(&hdmitx_device, buf, PAGE_SIZE);
}
    

static DEVICE_ATTR(mode, S_IWUSR | S_IRUGO, show_mode, store_mode);
static DEVICE_ATTR(edid, S_IWUSR | S_IRUGO, show_edid, NULL);

/*****************************
*    hdmitx display client interface 
*    
******************************/
static int hdmitx_notify_callback(struct notifier_block *block, unsigned long cmd , void *para)
{
    if (cmd != VOUT_EVENT_MODE_CHANGE)
        return -1;

    set_disp_mode_auto();

    return 0;
}


static struct notifier_block hdmitx_notifier_nb = {
    .notifier_call    = hdmitx_notify_callback,
};

/******************************
*  hdmitx kernel task
*******************************/
static int hdmi_task_handle(void *data) 
{
    hdmitx_dev_t* hdmitx_device = (hdmitx_dev_t*)data;

    hdmitx_init_parameters(&hdmi_info);

    if(hdmi_chip_type == HDMI_M1A){
        HDMITX_M1A_Init(hdmitx_device);
    }
    else{
        HDMITX_M1B_Init(hdmitx_device);
    }

    hdmitx_device->HWOp.SetupIRQ(hdmitx_device);

    while (1)
    {
        if (hdmitx_device->hpd_event == 1)
        {
            if(hdmitx_device->HWOp.GetEDIDData(hdmitx_device)){
                hdmitx_edid_clear(hdmitx_device);
                hdmitx_edid_parse(hdmitx_device);
                set_disp_mode_auto();
                hdmitx_device->hpd_event = 0;
            }    
        }
        else if(hdmitx_device->hpd_event == 2)
        {
            hdmitx_edid_clear(hdmitx_device);
            hdmitx_device->hpd_event = 0;
        }    
        msleep(500);
    }

    return 0;

}


/*****************************
*    hdmitx driver file_operations 
*    
******************************/
static int amhdmitx_open(struct inode *node, struct file *file)
{
    hdmitx_dev_t *hdmitx_in_devp;

    /* Get the per-device structure that contains this cdev */
    hdmitx_in_devp = container_of(node->i_cdev, hdmitx_dev_t, cdev);
    file->private_data = hdmitx_in_devp;

    return 0;

}


static int amhdmitx_release(struct inode *node, struct file *file)
{
    //hdmitx_dev_t *hdmitx_in_devp = file->private_data;

    /* Reset file pointer */

    /* Release some other fields */
    /* ... */
    return 0;
}



static int amhdmitx_ioctl(struct inode *node, struct file *file, unsigned int cmd,   unsigned long args)
{
    int   r = 0;
    switch (cmd) {
        default:
            break;
    }
    return r;
}

const static struct file_operations amhdmitx_fops = {
    .owner    = THIS_MODULE,
    .open     = amhdmitx_open,
    .release  = amhdmitx_release,
    .ioctl    = amhdmitx_ioctl,
};


static int amhdmitx_probe(struct platform_device *pdev)
{
    int r;
    pr_dbg("amhdmitx_probe\n");
    r = alloc_chrdev_region(&hdmitx_id, 0, HDMI_TX_COUNT, DEVICE_NAME);
    if (r < 0) {
        pr_error("Can't register major for amhdmitx device\n");
        return r;
    }
    hdmitx_class = class_create(THIS_MODULE, DEVICE_NAME);
    if (IS_ERR(hdmitx_class))
    {
        unregister_chrdev_region(hdmitx_id, HDMI_TX_COUNT);
        return -1;
        //return PTR_ERR(aoe_class);
    }

    cdev_init(&(hdmitx_device.cdev), &amhdmitx_fops);
    hdmitx_device.cdev.owner = THIS_MODULE;
    cdev_add(&(hdmitx_device.cdev), hdmitx_id, HDMI_TX_COUNT);

    //hdmitx_dev = device_create(hdmitx_class, NULL, hdmitx_id, "amhdmitx%d", 0);
    hdmitx_dev = device_create(hdmitx_class, NULL, hdmitx_id, NULL, "amhdmitx%d", 0); //kernel>=2.6.27 

    device_create_file(hdmitx_dev, &dev_attr_mode);
    device_create_file(hdmitx_dev, &dev_attr_edid);
    
    if (hdmitx_dev == NULL) {
        pr_error("device_create create error\n");
        class_destroy(hdmitx_class);
        r = -EEXIST;
        return r;
    }
    vout_register_client(&hdmitx_notifier_nb);

    hdmitx_device.task = kthread_run(hdmi_task_handle, &hdmitx_device, "kthread_hdmi");

    return r;
}

static int amhdmitx_remove(struct platform_device *pdev)
{
    /* Remove the cdev */
    device_remove_file(hdmitx_dev, &dev_attr_mode);
    device_remove_file(hdmitx_dev, &dev_attr_edid);

    cdev_del(&hdmitx_device.cdev);

    device_destroy(hdmitx_class, hdmitx_id);

    class_destroy(hdmitx_class);

    unregister_chrdev_region(hdmitx_id, HDMI_TX_COUNT);
    return 0;
}

static struct platform_driver amhdmitx_driver = {
    .probe      = amhdmitx_probe,
    .remove     = amhdmitx_remove,
    .driver     = {
        .name   = DEVICE_NAME,
    }
};

static struct platform_device amhdmi_tx_device = {
.name = DEVICE_NAME,
.id = 0,
.num_resources = 0,
};

static struct platform_device *devices[] = {
&amhdmi_tx_device,
};

static int  __init amhdmitx_init(void)
{
    pr_dbg("amhdmitx_init2\n");

    if (platform_driver_register(&amhdmitx_driver)) {
        pr_error("failed to register amhdmitx module\n");
        return -ENODEV;
    }
    platform_add_devices(devices,1);

    return 0;
}

static void __exit amhdmitx_exit(void)
{
    pr_dbg("amhdmitx_exit\n");

    platform_driver_unregister(&amhdmitx_driver);
    return ;
}

static  int __init hdmi_chip_select(char *s)
{
	switch(s[0])
	{
		case 'a':
		case 'A':
			hdmi_chip_type = HDMI_M1A;
			break;
	}
	return 0;
}

__setup("chip=",hdmi_chip_select);



module_init(amhdmitx_init);
module_exit(amhdmitx_exit);

MODULE_DESCRIPTION("AMLOGIC HDMI TX driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0.0");