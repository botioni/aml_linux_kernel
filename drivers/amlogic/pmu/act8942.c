/*
 * ACT8942 PMU driver
 *
 * Copyright (C) 2008 Rodolfo Giometti <giometti@linux.it>
 * Copyright (C) 2008 Eurotech S.p.A. <info@eurotech.it>
 *
 * Based on a previous work by Copyright (C) 2008 Texas Instruments, Inc.
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 */
#include <linux/module.h>
#include <linux/param.h>
#include <linux/jiffies.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/idr.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <asm/unaligned.h>
#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/fcntl.h>

#include "act8942.h"

#define DRIVER_VERSION			"0.0.1"
#define	ACT8942_DEVICE_NAME		"pmu act8942"
#define ACT8942_I2C_NAME		"act8942-i2c"

/* If the system has several batteries we need a different name for each
 * of them...
 */
static DEFINE_IDR(pmu_id);
static DEFINE_MUTEX(pmu_mutex);

static dev_t act8942_devno;

static struct class *act8942_class = NULL;

typedef struct pmu_dev_s {
    /* ... */
    struct cdev	cdev;             /* The cdev structure */
} pmu_dev_t;

static pmu_dev_t *act8942_dev = NULL;

struct act8942_device_info;

struct act8942_access_methods {
	int (*read)(u8 reg, int *rt_value, int b_single,
		struct act8942_device_info *di);
};

struct act8942_device_info {
	struct device 		*dev;
	int			id;
	struct act8942_access_methods	*bus;
	struct power_supply	bat;

	struct i2c_client	*client;
};

static struct i2c_driver act8942_i2c_driver;

/*
 * i2c specific code
 */

static int act8942_read_i2c(struct i2c_client *client,u8 reg, u8 *val)
{
	int err;

	if (!client->adapter)
		return -ENODEV;

	struct i2c_msg msgs[] = {
        {
            .addr = client->addr,
            .flags = 0,
            .len = 1,
            .buf = &reg,
        },
        {
            .addr = client->addr,
            .flags = I2C_M_RD,
            .len = 1,
            .buf = val,
        }
    };

	err = i2c_transfer(client->adapter, msgs, 1);

	return err;
}

inline void	act8942_dump(struct i2c_client *client)
{
	u8 val = 0;
	int ret = 0;
	
	ret = act8942_read_i2c(client, ACT8942_SYS_ADDR, &val);
	pr_info("act8942: [0x%x] : 0x%x\n", ACT8942_SYS_ADDR, val);
	
	ret = act8942_read_i2c(client, (ACT8942_SYS_ADDR+1), &val);
	pr_info("act8942: [0x%x] : 0x%x\n", ACT8942_SYS_ADDR+1, val);
	
	ret = act8942_read_i2c(client, ACT8942_REG1_ADDR, &val);
	pr_info("act8942: [0x%x] : 0x%x\n", ACT8942_REG1_ADDR, val);

	ret = act8942_read_i2c(client, (ACT8942_REG1_ADDR+1), &val);
	pr_info("act8942: [0x%x] : 0x%x\n", ACT8942_REG1_ADDR+1, val);

	ret = act8942_read_i2c(client, (ACT8942_REG1_ADDR+2), &val);
	pr_info("act8942: [0x%x] : 0x%x\n", ACT8942_REG1_ADDR+2, val);

	ret = act8942_read_i2c(client, ACT8942_REG2_ADDR, &val);
	pr_info("act8942: [0x%x] : 0x%x\n", ACT8942_REG2_ADDR, val);

	ret = act8942_read_i2c(client, (ACT8942_REG2_ADDR+1), &val);
	pr_info("act8942: [0x%x] : 0x%x\n", ACT8942_REG2_ADDR+1, val);

	ret = act8942_read_i2c(client, (ACT8942_REG2_ADDR+2), &val);
	pr_info("act8942: [0x%x] : 0x%x\n", ACT8942_REG2_ADDR+2, val);

	ret = act8942_read_i2c(client, ACT8942_REG3_ADDR, &val);
	pr_info("act8942: [0x%x] : 0x%x\n", ACT8942_REG3_ADDR, val);

	ret = act8942_read_i2c(client, (ACT8942_REG3_ADDR+1), &val);
	pr_info("act8942: [0x%x] : 0x%x\n", ACT8942_REG3_ADDR+1, val);

	/*

	ret = act8942_read_i2c(client, ACT8942_REG1_ADDR, &val);
	pr_info("act8942: [0x%x] : 0x%x\n", ACT8942_REG3_ADDR+2, act8942_read_i2c(ACT8942_REG3_ADDR+2));

	ret = act8942_read_i2c(client, ACT8942_REG1_ADDR, &val);
	pr_info("act8942: [0x%x] : 0x%x\n", ACT8942_REG4_ADDR, act8942_read_i2c(ACT8942_REG4_ADDR));

	ret = act8942_read_i2c(client, ACT8942_REG1_ADDR, &val);
	pr_info("act8942: [0x%x] : 0x%x\n", ACT8942_REG4_ADDR+1, act8942_read_i2c(ACT8942_REG4_ADDR+1));

	ret = act8942_read_i2c(client, ACT8942_REG1_ADDR, &val);
	pr_info("act8942: [0x%x] : 0x%x\n", ACT8942_REG5_ADDR, act8942_read_i2c(ACT8942_REG5_ADDR));

	ret = act8942_read_i2c(client, ACT8942_REG1_ADDR, &val);
	pr_info("act8942: [0x%x] : 0x%x\n", ACT8942_REG5_ADDR+1, act8942_read_i2c(ACT8942_REG5_ADDR+1));

	ret = act8942_read_i2c(client, ACT8942_REG1_ADDR, &val);
	pr_info("act8942: [0x%x] : 0x%x\n", ACT8942_REG6_ADDR, act8942_read_i2c(ACT8942_REG6_ADDR));

	ret = act8942_read_i2c(client, ACT8942_REG1_ADDR, &val);
	pr_info("act8942: [0x%x] : 0x%x\n", ACT8942_REG6_ADDR+1, act8942_read_i2c(ACT8942_REG6_ADDR+1));

	ret = act8942_read_i2c(client, ACT8942_REG1_ADDR, &val);
	pr_info("act8942: [0x%x] : 0x%x\n", ACT8942_REG7_ADDR, act8942_read_i2c(ACT8942_REG7_ADDR));

	ret = act8942_read_i2c(client, ACT8942_REG1_ADDR, &val);
	pr_info("act8942: [0x%x] : 0x%x\n", ACT8942_REG7_ADDR+1, act8942_read_i2c(ACT8942_REG7_ADDR+1));

	ret = act8942_read_i2c(client, ACT8942_REG1_ADDR, &val);
	pr_info("act8942: [0x%x] : 0x%x\n", ACT8942_APCH_ADDR, act8942_read_i2c(ACT8942_APCH_ADDR));

	ret = act8942_read_i2c(client, ACT8942_REG1_ADDR, &val);
	pr_info("act8942: [0x%x] : 0x%x\n", ACT8942_APCH_ADDR+1, act8942_read_i2c(ACT8942_APCH_ADDR+1));

	ret = act8942_read_i2c(client, ACT8942_REG1_ADDR, &val);
	pr_info("act8942: [0x%x] : 0x%x\n", ACT8942_APCH_ADDR+8, act8942_read_i2c(ACT8942_APCH_ADDR+8));

	ret = act8942_read_i2c(client, ACT8942_REG1_ADDR, &val);
	pr_info("act8942: [0x%x] : 0x%x\n", ACT8942_APCH_ADDR+9, act8942_read_i2c(ACT8942_APCH_ADDR+9));

	ret = act8942_read_i2c(client, ACT8942_REG1_ADDR, &val);
	pr_info("act8942: [0x%x] : 0x%x\n", ACT8942_APCH_ADDR+0xa, act8942_read_i2c(ACT8942_APCH_ADDR+0xa));
*/

}


static int act8942_i2c_probe(struct i2c_client *client,
				 const struct i2c_device_id *id)
{
	char *name;
	struct act8942_device_info *di;
	struct act8942_access_methods *bus;
	int num;
	int retval = 0;
	pr_info("act8942_i2c_probe\n");
	/* Get new ID for the new PMU device */
	retval = idr_pre_get(&pmu_id, GFP_KERNEL);
	if (retval == 0)
		return -ENOMEM;
	mutex_lock(&pmu_mutex);
	retval = idr_get_new(&pmu_id, client, &num);
	mutex_unlock(&pmu_mutex);
	if (retval < 0)
		return retval;

	name = kasprintf(GFP_KERNEL, "%s-%d", id->name, num);
	if (!name) {
		dev_err(&client->dev, "failed to allocate device name\n");
		retval = -ENOMEM;
		goto act8942_failed_1;
	}

	di = kzalloc(sizeof(*di), GFP_KERNEL);
	if (!di) {
		dev_err(&client->dev, "failed to allocate device info data\n");
		retval = -ENOMEM;
		goto act8942_failed_2;
	}
	di->id = num;
	//di->chip = id->driver_data; //elvis

	bus = kzalloc(sizeof(*bus), GFP_KERNEL);
	if (!bus) {
		dev_err(&client->dev, "failed to allocate access method "
					"data\n");
		retval = -ENOMEM;
		goto act8942_failed_3;
	}

	i2c_set_clientdata(client, di);
	di->dev = &client->dev;
	di->bat.name = name;
	bus->read = act8942_read_i2c;
	di->bus = bus;
	di->client = client;

	//act8942_powersupply_init(di);	//elvis tmp

	retval = power_supply_register(&client->dev, &di->bat);
	if (retval) {
		dev_err(&client->dev, "failed to register PMU\n");
		goto act8942_failed_4;
	}

	dev_info(&client->dev, "support ver. %s enabled\n", DRIVER_VERSION);

	act8942_dump(client);

	return 0;

act8942_failed_4:
	kfree(bus);
act8942_failed_3:
	kfree(di);
act8942_failed_2:
	kfree(name);
act8942_failed_1:
	mutex_lock(&pmu_mutex);
	idr_remove(&pmu_id, num);
	mutex_unlock(&pmu_mutex);

	return retval;
}

static int act8942_i2c_remove(struct i2c_client *client)
{
	struct act8942_device_info *di = i2c_get_clientdata(client);
	pr_info("act8942_i2c_remove\n");
	power_supply_unregister(&di->bat);

	kfree(di->bat.name);

	mutex_lock(&pmu_mutex);
	idr_remove(&pmu_id, di->id);
	mutex_unlock(&pmu_mutex);

	kfree(di);

	return 0;
}

static int act8942_open(struct inode *inode, struct file *file)
{
    pmu_dev_t *pmu_dev;
	pr_info("act8942_open\n");
    /* Get the per-device structure that contains this cdev */
    pmu_dev = container_of(inode->i_cdev, pmu_dev_t, cdev);
    file->private_data = pmu_dev;

    return 0;
}

static int act8942_release(struct inode *inode, struct file *file)
{
    file->private_data = NULL;

    return 0;
}

static int act8942_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
	int res = 0;
/*    void __user *argp = (void __user *)arg;
    struct camera_info_s para;
    switch(cmd)
    {
        case CAMERA_IOC_START:
			
            printk( " start camera engineer. \n ");
            if(copy_from_user(&para, argp, sizeof(struct camera_info_s)))
            {
                printk(KERN_ERR " para is error, use the default value. \n ");
            }
            else
            {
                memcpy(&GC0308_info.para, &para,sizeof(struct camera_info_s) );
            }
            printk("saturation = %d, brighrness = %d, contrast = %d, hue = %d, exposure = %d, sharpness = %d, mirro_flip = %d, resolution = %d . \n",
                para.saturation, para.brighrness, para.contrast, para.hue, para.exposure, para.sharpness, para.mirro_flip, para.resolution);
            start_camera();
            break;

        case CAMERA_IOC_STOP:
            printk(KERN_INFO " stop camera engineer. \n ");
            stop_camera();
            break;

        case CAMERA_IOC_SET_PARA:
            if(copy_from_user(&para, argp, sizeof(struct camera_info_s)))
            {
                pr_err(" no para for camera setting, do nothing. \n ");
            }
            else
            {
                set_camera_para(&para);
            }
            break;

        case CAMERA_IOC_GET_PARA:
            copy_to_user((void __user *)arg, &GC0308_info.para, sizeof(struct camera_info_s));
            break;

        default:
                printk("camera ioctl cmd is invalid. \n");
                break;
    }
    */
    return res;
}


static struct file_operations act8942_fops = {
    .owner   = THIS_MODULE,
    .open    = act8942_open,
    .release = act8942_release,
    .ioctl   = act8942_ioctl,
};


static int act8942_probe(struct platform_device *pdev)
{
    int ret;
	struct device *dev_p;
	
	pr_info("act8942_probe\n");
	act8942_dev = kmalloc(sizeof(pmu_dev_t), GFP_KERNEL);
	if (!act8942_dev)
	{
		pr_err("act8942: failed to allocate memory for pmu device\n");
		return -ENOMEM;
	}

	ret = alloc_chrdev_region(&act8942_devno, 0, 1, ACT8942_DEVICE_NAME);
	if (ret < 0) {
		pr_err("act8942: failed to allocate chrdev. \n");
		return 0;
	}

	act8942_class = class_create(THIS_MODULE, ACT8942_DEVICE_NAME);
	if (IS_ERR(act8942_class))
    {
        unregister_chrdev_region(act8942_devno, 1);
		pr_err("act8942: failed to create class. \n");
        return PTR_ERR(act8942_class);
    }

	/* connect the file operations with cdev */
	cdev_init(&act8942_dev->cdev, &act8942_fops);
	act8942_dev->cdev.owner = THIS_MODULE;

	/* connect the major/minor number to the cdev */
	ret = cdev_add(&act8942_dev->cdev, act8942_devno, 1);
	if (ret) {
		pr_err("act8942: failed to add device. \n");
		/* @todo do with error */
		return ret;
	}

	/* create /dev nodes */
    dev_p = device_create(act8942_class, NULL, MKDEV(MAJOR(act8942_devno), 0),
                        NULL, "act8942");
    if (IS_ERR(dev_p)) {
        pr_err("act8942: failed to create device node\n");
        class_destroy(act8942_class);
        /* @todo do with error */
        return PTR_ERR(dev_p);;
    }
	
	/* We expect this driver to match with the i2c device registered
	 * in the board file immediately. */
	ret = i2c_add_driver(&act8942_i2c_driver);
	if (ret < 0)
	{
        pr_err("act8942: failed to add i2c driver. \n");
        ret = -ENOTSUPP;
    }

    printk( "act8942: driver initialized ok\n");
	
    return ret;
}

static int act8942_remove(struct platform_device *pdev)
{
	pr_info("act8942_remove\n");
	i2c_del_driver(&act8942_i2c_driver);
    cdev_del(&act8942_dev->cdev);
    unregister_chrdev_region(act8942_devno, 1);
    kfree(act8942_dev);

     return 0;
}

static const struct i2c_device_id act8942_i2c_id[] = {
	{ ACT8942_I2C_NAME, 0 },
	{},
};

static struct i2c_driver act8942_i2c_driver = {
	.driver = {
		.name = "ACT8942-PMU",
	},
	.probe = act8942_i2c_probe,
	.remove = act8942_i2c_remove,
	.id_table = act8942_i2c_id,
};

static struct platform_driver ACT8942_platform_driver = {
	.probe = act8942_probe,
    .remove = act8942_remove,
	.driver = {
	.name = ACT8942_DEVICE_NAME,
	},
};


static int __init act8942_pmu_init(void)
{
	int ret;
	pr_info("act8942_pmu_init\n");
	ret = platform_driver_register(&ACT8942_platform_driver);
	if (ret) {
        printk(KERN_ERR "failed to register ACT8942 module, error %d\n", ret);
        return -ENODEV;
    }
	return ret;
}
module_init(act8942_pmu_init);

static void __exit act8942_pmu_exit(void)
{
	pr_info("act8942_pmu_exit\n");
    platform_driver_unregister(&ACT8942_platform_driver);
}
module_exit(act8942_pmu_exit);

MODULE_AUTHOR("Elvis Yu <elvis.yu@amlogic.com>");
MODULE_DESCRIPTION("ACT8942 PMU driver");
MODULE_LICENSE("GPL");

