/*
 * linux/drivers/input/touchscreen/itk.c
 *
 * Copyright (C) 2007-2008 Avionic Design Development GmbH
 * Copyright (C) 2008-2009 Avionic Design GmbH
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Written by x
 */

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>
#include <linux/i2c/itk.h>


#define        DRIVER_NAME     "itk"
#define        DRIVER_VERSION  "1"

/* basic commands */
#define CANDO           1
#define SINTEK          0
#define SINTEK_NEW      2

//#define TS_DELAY_WORK
#define MULTI_TOUCH

/* periodic polling delay and period */
#define        TS_POLL_DELAY   (1 * 1000000)
#define        TS_POLL_PERIOD  (5 * 1000000)

/**
 * struct ts_event - touchscreen event structure
 * @pendown:   state of the pen
 * @x:         X-coordinate of the event
 * @y:         Y-coordinate of the event
 * @z:         pressure of the event
 */
struct ts_event {
       short x;
       short y;
       short xz;
       short yz;
       short xw;
       short yw;
};

/**
 * struct itk - touchscreen controller context
 * @client:    I2C client
 * @input:     touchscreen input device
 * @lock:      lock for resource protection
 * @timer:     timer for periodical polling
 * @work:      workqueue structure
 * @pendown:   current pen state
 * @event:     current touchscreen event
 * @pdata:     platform-specific information
 */
struct itk {
       struct i2c_client *client;
       struct input_dev *input;
       spinlock_t lock;
       struct hrtimer timer;
#ifdef TS_DELAY_WORK
       struct delayed_work work;
#else
       struct work_struct work;
       struct workqueue_struct *workqueue;
#endif
       struct ts_event event[5];
       unsigned pendown:1;
       int touching_num;
       int xmax;
       int ymax;
       int vendor;
       struct itk_platform_data *pdata;

       struct delayed_work cal_work;
};

/**
 * itk_get_pendown_state() - obtain the current pen state
 * @ts:                touchscreen controller context
 */
static int itk_get_pendown_state(struct itk *ts)
{
       int state = 0;

       if (ts && ts->pdata && ts->pdata->get_irq_level)
               state = !ts->pdata->get_irq_level();

       return state;
}

static int itk_register_input(struct itk *ts)
{
    int ret;
    struct input_dev *dev;

    dev = input_allocate_device();
    if (dev == NULL)
        return -1;

    dev->name = "sintek capacitive touchscreen";
    //dev->phys = ts->phys;
    dev->id.bustype = BUS_I2C;

    dev->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
    dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);
    input_set_abs_params(dev, ABS_X, 0, ts->xmax, 0, 0);
    input_set_abs_params(dev, ABS_Y, 0, ts->ymax, 0, 0);
    input_set_abs_params(dev, ABS_PRESSURE, 0, 200, 0, 0);

#if 0//def MULTI_TOUCH
    set_bit(ABS_MT_TOUCH_MAJOR, dev->absbit);
    set_bit(ABS_MT_WIDTH_MAJOR, dev->absbit);
    set_bit(ABS_MT_POSITION_X, dev->absbit);
    set_bit(ABS_MT_POSITION_Y, dev->absbit);
    set_bit(ABS_TOOL_WIDTH, dev->absbit);

    input_set_abs_params(dev, ABS_MT_POSITION_X, 0, ts->xmax, 0, 0);
    input_set_abs_params(dev, ABS_MT_POSITION_Y, 0, ts->ymax, 0, 0);
    input_set_abs_params(dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
    input_set_abs_params(dev, ABS_MT_WIDTH_MAJOR, 0, 255, 0, 0);
#endif

    ret = input_register_device(dev);
    if (ret < 0) {
        input_free_device(dev);
        return -1;
    }
    
    ts->input = dev;
    return 0;
}

static int itk_read_block(struct i2c_client *client, u8 addr, u8 len, u8 *data)
{
    u8 msgbuf0[1] = { addr };
    u16 slave = client->addr;
    u16 flags = client->flags;
    struct i2c_msg msg[2] = { 
        { slave, flags, 1, msgbuf0 },
        { slave, flags | I2C_M_RD, len, data }
    };

    return i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg));
}

static int itk_write_block(struct i2c_client *client, u8 addr, u8 len, u8 *data)
{
    u8 msgbuf0[1] = { addr };
    u16 slave = client->addr;
    u16 flags = client->flags;
    struct i2c_msg msg[2] = {
        { slave, flags, 1, msgbuf0 },
        { slave, flags, len, data }
    };

    return i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg));
}


static void itk_reset(struct itk *ts)
{
    return;
}

#define ITK_INFO_ADDR   4
#define ITK_INFO_LEN        10

#define FIRST_POINT_ADDR    2
#define X_OFFSET            0
#define Y_OFFSET            2
#define XW_OFFSET       8
#define YW_OFFSET       9
#define XZ_OFFSET       12
#define YZ_OFFSET       13

static int itk_read_sensor(struct itk *ts)
{
    int ret,i;
    u8 data[ITK_INFO_LEN];
    struct ts_event *event;
    
    /* To ensure data coherency, read the sensor with a single transaction. */
    ret = itk_read_block(ts->client, ITK_INFO_ADDR, ITK_INFO_LEN, data);
    if (ret < 0) {
        dev_err(&ts->client->dev, "Read block failed: %d\n", ret);
        return ret;
    }

    int id = (data[1]>>2)& 0x1f;
    event = &ts->event[id];
    event->x = (data[3] << 8) | data[2];
    //printk(KERN_INFO "data[3][2] = 0x%2x%2x\n", data[3], data[2]);
    event->x = (event->x*1024)/17407; //(event->x*800)/32752;
    //printk(KERN_INFO "caculate event->x = %d\n\n", event->x);
    
    event->y = (data[5] << 8) | data[4];
    //printk(KERN_INFO "data[5][4] = 0x%2x%2x\n", data[5], data[4]);
    event->y = (event->y*768)/12799; //(event->y*600)/32752;
    //printk(KERN_INFO "caculate event->y = %d\n\n", event->y);
    ts->touching_num++;
    return 0;
}

/**
 * itk_work() - work queue handler (initiated by the interrupt handler)
 * @work:      work queue to handle
 */
static void itk_work(struct work_struct *work)
{
#ifdef TS_DELAY_WORK
    struct itk *ts = container_of(to_delayed_work(work), struct itk, work);
#else
    struct itk *ts = container_of(work, struct itk, work);
#endif
    struct ts_event *event;
    int i;

//  printk(KERN_INFO "itk work runing\n");
    if (itk_get_pendown_state(ts)) { 
        if (itk_read_sensor(ts) < 0) {
            printk(KERN_INFO "work read i2c failed\n");
            goto restart;
        }
        event = &ts->event[0];
        input_report_abs(ts->input, ABS_X, event->x);
        input_report_abs(ts->input, ABS_Y, event->y);
        //input_report_abs(ts->input, ABS_PRESSURE, event->xz + event->yz);
        input_report_abs(ts->input, ABS_PRESSURE, 100);
        if (!ts->pendown) {
            ts->pendown = 1;
            input_report_key(ts->input, BTN_TOUCH, 1);
                printk(KERN_INFO "DOWN\n");
        }
        
        for (i=0; i<ts->touching_num; i++) {
//          printk(KERN_INFO "point%d x=%d y=%d pressue=%d\n",
//              i, event->x, event->y, event->xz + event->yz);
#ifdef MULTI_TOUCH
            input_report_abs(ts->input, ABS_MT_POSITION_X, event->x);
            input_report_abs(ts->input, ABS_MT_POSITION_Y, event->y);
            //input_report_abs(ts->input, ABS_MT_PRESSURE, event->xz + event->yz);
            input_report_abs(ts->input, ABS_MT_PRESSURE, 100);
            input_mt_sync(ts->input);
#endif
            event++;
        }
        input_sync(ts->input);
restart:
#ifdef TS_DELAY_WORK
        schedule_delayed_work(&ts->work, msecs_to_jiffies(TS_POLL_PERIOD));
#else
        hrtimer_start(&ts->timer, ktime_set(0, TS_POLL_PERIOD), HRTIMER_MODE_REL);
#endif
    }
    else {
        /* enable IRQ after the pen was lifted */
        if (ts->pendown) {
            ts->pendown = 0;
            input_report_key(ts->input, BTN_TOUCH, 0);
            input_report_abs(ts->input, ABS_PRESSURE, 0);
            input_sync(ts->input);
                    printk(KERN_INFO "UP\n");
        }
        ts->touching_num = 0;
        enable_irq(ts->client->irq);
    }
}

static void itk_cal_work(struct work_struct *work)
{
    printk(KERN_INFO "\n ***********re-calibration************\n\n");
}

#ifndef TS_DELAY_WORK
/**
 * itk_timer() - timer callback function
 * @timer:     timer that caused this function call
 */
static enum hrtimer_restart itk_timer(struct hrtimer *timer)
{
    struct itk *ts = container_of(timer, struct itk, timer);
    unsigned long flags = 0;
    
    spin_lock_irqsave(&ts->lock, flags);
//  printk(KERN_INFO "enter timer\n");
    queue_work(ts->workqueue, &ts->work);   
    spin_unlock_irqrestore(&ts->lock, flags);
    return HRTIMER_NORESTART;
}
#endif

/**
 * itk_interrupt() - interrupt handler for touch events
 * @irq:       interrupt to handle
 * @dev_id:    device-specific information
 */
static irqreturn_t itk_interrupt(int irq, void *dev_id)
{
    struct i2c_client *client = (struct i2c_client *)dev_id;
    struct itk *ts = i2c_get_clientdata(client);
    unsigned long flags;
    
    spin_lock_irqsave(&ts->lock, flags);
    printk(KERN_INFO "enter penirq\n");
    /* if the pen is down, disable IRQ and start timer chain */
    if (itk_get_pendown_state(ts)) {
        disable_irq_nosync(client->irq);
#ifdef TS_DELAY_WORK
        schedule_delayed_work(&ts->work, msecs_to_jiffies(TS_POLL_DELAY));
#else
        hrtimer_start(&ts->timer, ktime_set(0, TS_POLL_DELAY), HRTIMER_MODE_REL);
#endif
    }
    spin_unlock_irqrestore(&ts->lock, flags);
    return IRQ_HANDLED;
}

/**
 * itk_probe() - initialize the I2C client
 * @client:    client to initialize
 * @id:                I2C device ID
 */
static int itk_probe(struct i2c_client *client,
    const struct i2c_device_id *id)
{
    struct itk *ts;
    int err = 0;

    ts = kzalloc(sizeof(struct itk), GFP_KERNEL);
    if (!ts) {
        err = -ENOMEM;
        goto fail;
    }

    ts->client = client;
    itk_reset(ts);

    if (itk_register_input(ts) < 0) {
        dev_err(&client->dev, "register input fail!\n");
        goto fail;
    }

    /* setup platform-specific hooks */
    ts->pdata = client->dev.platform_data;
    if (!ts->pdata || !ts->pdata->init_irq || !ts->pdata->get_irq_level) {
        dev_err(&client->dev, "no platform-specific callbacks "
            "provided\n");
        err = -ENXIO;
        goto fail;
    }

    if (ts->pdata->init_irq) {
        err = ts->pdata->init_irq();
        if (err < 0) {
            dev_err(&client->dev, "failed to initialize IRQ#%d: "
                "%d\n", client->irq, err);
            goto fail;
        }
    }

    spin_lock_init(&ts->lock);
#ifdef TS_DELAY_WORK
    INIT_DELAYED_WORK(&ts->work, itk_work);
#else
    hrtimer_init(&ts->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
    ts->timer.function = itk_timer;
    INIT_WORK(&ts->work, itk_work);
    ts->workqueue = create_singlethread_workqueue("itk");
    if (ts->workqueue == NULL) {
        dev_err(&client->dev, "can't create work queue\n");
        err = -ENOMEM;
        goto fail;
    }
    printk("work create: %x\n", ts->workqueue);
#endif

    ts->pendown = 0;
    ts->touching_num = 0;

    err = request_irq(client->irq, itk_interrupt, IRQF_TRIGGER_FALLING,
        client->dev.driver->name, client);
    if (err) {
        dev_err(&client->dev, "failed to request IRQ#%d: %d\n",
        client->irq, err);
        goto fail_irq;
    }

    i2c_set_clientdata(client, ts);
    INIT_DELAYED_WORK(&ts->cal_work, itk_cal_work);
    schedule_delayed_work(&ts->cal_work, 20*HZ);
    err = 0;
    goto out;

fail_irq:
    free_irq(client->irq, client);

fail:
    if (ts) {
        input_free_device(ts->input);
        kfree(ts);
    }

    i2c_set_clientdata(client, NULL);
out:
    printk("itk touch screen driver ok\n");
    return err;
}

/**
 * itk_remove() - cleanup the I2C client
 * @client:    client to clean up
 */
static int itk_remove(struct i2c_client *client)
{
    struct itk *priv = i2c_get_clientdata(client);

    free_irq(client->irq, client);
    i2c_set_clientdata(client, NULL);
    input_unregister_device(priv->input);
    kfree(priv);

    return 0;
}

static const struct i2c_device_id itk_ids[] = {
    { DRIVER_NAME, 0 },
    { }
};

MODULE_DEVICE_TABLE(i2c, itk_ids);
/* SINTEK I2C Capacitive Touch Screen driver */
static struct i2c_driver itk_driver = {
    .driver = {
    .name = DRIVER_NAME,
    .owner = THIS_MODULE,
    },
    .probe = itk_probe,
    .remove = __devexit_p(itk_remove),
    .id_table = itk_ids,
};

/**
 * itk_init() - module initialization
 */
static int __init itk_init(void)
{
    return i2c_add_driver(&itk_driver);
}

/**
 * itk_exit() - module cleanup
 */
static void __exit itk_exit(void)
{
    i2c_del_driver(&itk_driver);
}

module_init(itk_init);
module_exit(itk_exit);

MODULE_AUTHOR("");
MODULE_DESCRIPTION("itk I2C Capacitive Touch Screen driver");
MODULE_LICENSE("GPL v2");
MODULE_VERSION(DRIVER_VERSION);


