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


#define DRIVER_NAME "itk"
#define DRIVER_VERSION "1"

//#define ITK_TS_DEBUG_REPORT
#define ITK_TS_DEBUG_READ
//#define TS_DELAY_WORK

/* periodic polling delay and period */
#define TS_POLL_DELAY   (1 * 1000000)
#define TS_POLL_PERIOD  (5 * 1000000)

#define MAX_SUPPORT_POINT   5 //just support 2 point now
#define ITK_INFO_ADDR       0x4
#define ITK_INFO_LEN        10

/**
 * struct ts_event - touchscreen event structure
 * @contactid:  num id
 * @pendown:    state of the pen
 * @valid:      is valid data
 * @x:          X-coordinate of the event
 * @y:          Y-coordinate of the event
 * @z:          pressure of the event
 */
struct ts_event {
        short contactid;
        short pendown;
        short valid;
        short x;
        short y;
        short xz;
        short yz;
        short xw;
        short yw;
};

/**
 * struct itk - touchscreen controller context
 * @client:         I2C client
 * @input:          touchscreen input device
 * @lock:           lock for resource protection
 * @timer:          timer for periodical polling
 * @work:           workqueue structure
 * @event[]:        touchscreen event buff
 * @pendown:        current pen state
 * @touching_num:   count for check touching fingers
 * @lcd_xmax:       lcd resolution
 * @lcd_ymax:       lcd resolution
 * @tp_xmax:        max virtual resolution
 * @tp_ymax:        max virtual resolution
 * @pdata:          platform-specific information
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
       struct ts_event event[MAX_SUPPORT_POINT];
       unsigned pendown:1;
       unsigned touching_num;
       int lcd_xmax;
       int lcd_ymax;
       int tp_xmax;
       int tp_ymax;
       struct itk_platform_data *pdata;
//       struct delayed_work cal_work;
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

    dev->name = "Touch Screen";
    dev->phys = "I2C";
    dev->id.bustype = BUS_I2C;

    set_bit(EV_ABS, dev->evbit);
    set_bit(EV_KEY, dev->evbit);
    set_bit(BTN_TOUCH, dev->keybit);
    set_bit(ABS_MT_TOUCH_MAJOR, dev->absbit);
    set_bit(ABS_MT_WIDTH_MAJOR, dev->absbit);
    set_bit(ABS_MT_POSITION_X, dev->absbit);
    set_bit(ABS_MT_POSITION_Y, dev->absbit);
    set_bit(ABS_MT_TRACKING_ID, dev->absbit);
    //set_bit(ABS_MT_PRESSURE, dev->absbit);

    input_set_abs_params(dev, ABS_X, 0, ts->lcd_xmax, 0, 0);
    input_set_abs_params(dev, ABS_Y, 0, ts->lcd_ymax, 0, 0);
    input_set_abs_params(dev, ABS_MT_POSITION_X, 0, ts->lcd_xmax, 0, 0);
    input_set_abs_params(dev, ABS_MT_POSITION_Y, 0, ts->lcd_ymax, 0, 0);
    input_set_abs_params(dev, ABS_MT_TOUCH_MAJOR, 0, ts->lcd_xmax, 0, 0);
    input_set_abs_params(dev, ABS_MT_WIDTH_MAJOR, 0, ts->lcd_xmax, 0, 0);
    input_set_abs_params(dev, ABS_MT_TRACKING_ID, 0, 10, 0, 0);
    //input_set_abs_params(dev, ABS_MT_PRESSURE, 0, ???, 0, 0);

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
        { slave, flags|I2C_M_RD, len, data }
    };

    return i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg));
}

/* //just mark for not used warning
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
*/

static void itk_reset(struct itk *ts)
{
    int i = 0;
    if (NULL == ts)
        return;
    memset(ts->event, 0, sizeof(struct ts_event)*MAX_SUPPORT_POINT);
    for (i=0; i<MAX_SUPPORT_POINT; i++)
    {
        ts->event[i].pendown = -1;
    }
    return;
}

static int itk_read_sensor(struct itk *ts)
{
    int ret=-1, up_down=0, id=0, valid=0;
    u8 data[ITK_INFO_LEN];
    struct ts_event *event;

    /* To ensure data coherency, read the sensor with a single transaction. */
    ret = itk_read_block(ts->client, ITK_INFO_ADDR, ITK_INFO_LEN, data);
    if (ret < 0) {
        dev_err(&ts->client->dev, "Read block failed: %d\n", ret);
        return ret;
    }
    up_down = data[1]&0x1;
    valid = (data[1]&0x80)?(1):(0);
    id = (data[1]>>2)& 0x1f;
    if (ts->touching_num > 1)
        event = &ts->event[0];
    else
        event = &ts->event[ts->touching_num];
    event->contactid = id;
    event->pendown = up_down;
    event->valid = valid;
    event->x = (data[3] << 8) | data[2];
    event->y = (data[5] << 8) | data[4];
    event->x = (event->x*ts->lcd_xmax)/(ts->tp_xmax);
    event->y = (event->y*ts->lcd_ymax)/(ts->tp_ymax);
    #ifdef ITK_TS_DEBUG_READ
    printk(KERN_INFO "\nread_sensor valid = %d, id = %d, pendown = %d, event[%d]->x = %d, event[%d]->y = %d\n", event->valid, id, event->pendown, ts->touching_num, event->x, ts->touching_num, event->y);
    #endif
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
    int i = 0;

    if (itk_get_pendown_state(ts)) {
        if (itk_read_sensor(ts) < 0) {
            printk(KERN_INFO "work read i2c failed\n");
            goto restart;
        }
        if (!ts->pendown) {
            ts->pendown = 1;
            //input_report_key(ts->input, BTN_TOUCH, 1);
            #ifdef ITK_TS_DEBUG_REPORT
            printk(KERN_INFO "DOWN\n");
            #endif
        }
        if (ts->touching_num == 2) //tow points event got
        {
            for (i=0; i<2; i++) //to deliver two points separately
            {
                if ((i == 0) && (ts->event[i].contactid == ts->event[i+1].contactid)
                    && (ts->event[i].pendown == ts->event[i+1].pendown)
                    && (ts->event[i].x == ts->event[i+1].x)
                    && (ts->event[i].y == ts->event[i+1].y)
                    ) // if one finger and same position, sent one report
                {
                    continue;
                }
                event = &ts->event[i];
                if (event->valid == 1)
                {
                    input_report_abs(ts->input, ABS_MT_TRACKING_ID, event->contactid);
                    #ifdef ITK_TS_DEBUG_REPORT
                    printk(KERN_INFO "\nreport ABS_MT_TRACKING_ID %d\n", event->contactid);
                    #endif
                    input_report_abs(ts->input, ABS_MT_TOUCH_MAJOR, event->pendown);
                    #ifdef ITK_TS_DEBUG_REPORT
                    printk(KERN_INFO "report ABS_MT_TOUCH_MAJOR %d\n", event->pendown);
                    #endif
                    input_report_abs(ts->input, ABS_MT_WIDTH_MAJOR, 0);
                    #ifdef ITK_TS_DEBUG_REPORT
                    printk(KERN_INFO "report ABS_MT_WIDTH_MAJOR %d\n", 0);
                    #endif
                    input_report_abs(ts->input, ABS_MT_POSITION_X, event->x);
                    input_report_abs(ts->input, ABS_MT_POSITION_Y, event->y);
                    #ifdef ITK_TS_DEBUG_REPORT
                    printk(KERN_INFO "report ABS_MT_POSITION_XY %d,%d\n", event->x, event->y);
                    #endif
                    input_mt_sync(ts->input);
                    #ifdef ITK_TS_DEBUG_REPORT
                    printk(KERN_INFO "input_mt_sync\n");
                    #endif
                    if ((i == 0) && 
                        (ts->event[i].contactid != ts->event[i+1].contactid)
                        && ts->event[i+1].valid) //two fingers, just need one input_sync report.
                    {
                        continue;
                    }
                    #ifdef ITK_TS_DEBUG_REPORT
                    else
                    {
                        if (i == 0)
                            printk(KERN_INFO "[%d].contactid = %d, [%d].contactid = %d, [%d].valid = %d, [%d].pendown = %d\n", 
                            i, ts->event[i].contactid, i+1, ts->event[i+1].contactid, i+1, ts->event[i+1].valid, i+1, ts->event[i+1].pendown);
                    }
                    #endif
                    input_sync(ts->input);
                    #ifdef ITK_TS_DEBUG_REPORT
                    printk(KERN_INFO "input_sync\n");
                    #endif
                }
                #ifdef ITK_TS_DEBUG_READ
                else
                {
                    printk(KERN_INFO "Invalid Key %d\n", i);
                }
                #endif
            }
            ts->touching_num = 0;
        }
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
            input_mt_sync(ts->input);
            #ifdef ITK_TS_DEBUG_REPORT
            printk(KERN_INFO "\ninput_mt_sync\n");
            #endif
            input_sync(ts->input);
            #ifdef ITK_TS_DEBUG_REPORT
            printk(KERN_INFO "input_sync\n");
            printk(KERN_INFO "UP\n");
            #endif
            itk_reset(ts);
        }
        ts->touching_num = 0;
        enable_irq(ts->client->irq);
    }
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
    #ifdef ITK_TS_DEBUG_REPORT
    printk(KERN_INFO "enter penirq\n");
    #endif
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
    ts->pdata = (struct itk_platform_data*)client->dev.platform_data;
    if (!ts->pdata || !ts->pdata->init_irq || !ts->pdata->get_irq_level) {
        dev_err(&client->dev, "no platform-specific callbacks "
            "provided\n");
        err = -ENXIO;
        goto fail;
    }
    else
    {
        ts->lcd_xmax = ((struct itk_platform_data*) client->dev.platform_data)->lcd_max_width;
        ts->lcd_ymax = ((struct itk_platform_data*) client->dev.platform_data)->lcd_max_height;
        ts->tp_xmax = ((struct itk_platform_data*) client->dev.platform_data)->tp_max_width;
        ts->tp_ymax = ((struct itk_platform_data*) client->dev.platform_data)->tp_max_height;
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
    #ifdef ITK_TS_DEBUG_REPORT
    printk("work create: %x\n", ts->workqueue);
    #endif
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
    //schedule_delayed_work(&ts->cal_work, 20*HZ);
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
    itk_read_sensor(ts);
    itk_reset(ts);
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
/* ITK I2C Capacitive Touch Screen driver */
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


