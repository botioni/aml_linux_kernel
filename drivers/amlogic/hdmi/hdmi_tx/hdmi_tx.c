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
#include <linux/interrupt.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/mm.h>
#include <linux/major.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/cdev.h>
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


/* Per-device (per-bank) structure */
typedef struct hdmi_tx_dev_s {
    /* ... */
    struct cdev cdev;             /* The cdev structure */
    //wait_queue_head_t   wait_queue;            /* wait queues */
}hdmitx_dev_t;

static hdmitx_dev_t hdmitx_device;

static dev_t hdmitx_id;
static struct class *hdmitx_class;
static struct device *hdmitx_dev;

static HDMI_TX_INFO_t hdmi_info;

/*****************************
*    hdmitx attr management :
*    enable
*    mode
*    write_reg
* read_reg
******************************/

static void func_default_null(char* enable)
{
    
}    

static  void  set_disp_mode(char *mode)
{
    if(strncmp(mode,"480p",4)==0){
        hdmitx_set_display(HDMI_480p60);
    }
    else if(strncmp(mode,"1080p",5)==0){
        hdmitx_set_display(HDMI_1080p60);
    }
}



SET_HDMI_CLASS_ATTR(enable,func_default_null)
SET_HDMI_CLASS_ATTR(mode,set_disp_mode)
//SET_HDMI_CLASS_ATTR(wr_reg,write_reg)
//SET_HDMI_CLASS_ATTR(rd_reg,read_reg)

static  struct  class_attribute   *hdmi_attr[]={
&class_hdmi_attr_enable,
&class_hdmi_attr_mode,    
//&class_hdmi_attr_wr_reg,
//&class_hdmi_attr_rd_reg,
};

/*****************************
*    hdmitx display client interface 
*    
******************************/
static int hdmitx_notify_callback(struct notifier_block *block, unsigned long cmd , void *para)
{
    const vinfo_t *info;
    if (cmd != VOUT_EVENT_MODE_CHANGE)
        return -1;

    info = get_current_vinfo();
    
    //spin_lock_irqsave(&lock, flags);

    set_disp_mode(info->name);

    //spin_unlock_irqrestore(&lock, flags);

    return 0;
}


static struct notifier_block hdmitx_notifier_nb = {
    .notifier_call    = hdmitx_notify_callback,
};


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

    class_create_file(hdmitx_class, hdmi_attr[0]) ;
    class_create_file(hdmitx_class, hdmi_attr[1]) ;
    
    if (hdmitx_dev == NULL) {
        pr_error("device_create create error\n");
        class_destroy(hdmitx_class);
        r = -EEXIST;
        return r;
    }
    vout_register_client(&hdmitx_notifier_nb);

    hdmitx_init_parameters(&hdmi_info);

    HDMITX_HW_Init();

    HDMITX_HW_SetupIRQ();
    
    return r;
}

static int amhdmitx_remove(struct platform_device *pdev)
{
    /* Remove the cdev */
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

module_init(amhdmitx_init);
module_exit(amhdmitx_exit);

MODULE_DESCRIPTION("AMLOGIC HDMI TX driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0.0");
