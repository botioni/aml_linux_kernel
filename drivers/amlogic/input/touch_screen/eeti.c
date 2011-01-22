/*
 * linux/drivers/input/touchscreen/eeti.c
 *
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
#include <linux/i2c/eeti.h>
#include <linux/cdev.h>
#include <asm/uaccess.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
static struct early_suspend eeti_early_suspend;
#endif


#define DRIVER_NAME "eeti"
#define DRIVER_VERSION "1"

//#define EETI_TS_DEBUG_REPORT
//#define EETI_TS_DEBUG_READ
//#define EETI_TS_DEBUG_INFO
//#define TS_DELAY_WORK

/* periodic polling delay and period */
#define TS_POLL_DELAY   (80 * 1000000)
#define TS_POLL_PERIOD  (2 * 1000000)

#define MAX_SUPPORT_POINT   5 //just support 2 point now
#define EETI_INFO_ADDR       0x4
#define EETI_INFO_LEN        10

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
 * struct eeti - touchscreen controller context
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
struct eeti {
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
       struct eeti_platform_data *pdata;
//       struct delayed_work cal_work;
};

/**
 * eeti_get_pendown_state() - obtain the current pen state
 * @ts:                touchscreen controller context
 */
static int eeti_get_pendown_state(struct eeti *ts)
{
       int state = 0;

       if (ts && ts->pdata && ts->pdata->get_irq_level)
               state = !ts->pdata->get_irq_level();

       return state;
}

static int eeti_register_input(struct eeti *ts)
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

    input_set_abs_params(dev, ABS_X, 0, ts->tp_xmax, 0, 0);
    input_set_abs_params(dev, ABS_Y, 0, ts->tp_ymax, 0, 0);
    input_set_abs_params(dev, ABS_MT_POSITION_X, 0, ts->tp_xmax, 0, 0);
    input_set_abs_params(dev, ABS_MT_POSITION_Y, 0, ts->tp_ymax, 0, 0);
    input_set_abs_params(dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
    input_set_abs_params(dev, ABS_MT_WIDTH_MAJOR, 0, 255, 0, 0);
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

static int eeti_read_block(struct i2c_client *client, u8 addr, u8 len, u8 *data)
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
static int eeti_write_block(struct i2c_client *client, u8 addr, u8 len, u8 *data)
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

static void eeti_reset(struct eeti *ts)
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

static int eeti_read_sensor(struct eeti *ts)
{
    int ret=-1, up_down=0, id=0, valid=0;
    u8 data[EETI_INFO_LEN];
    struct ts_event *event;

    /* To ensure data coherency, read the sensor with a single transaction. */
    ret = eeti_read_block(ts->client, EETI_INFO_ADDR, EETI_INFO_LEN, data);
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
    #ifdef EETI_TS_DEBUG_READ
    printk(KERN_INFO "\nread_sensor valid = %d, id = %d, pendown = %d, event[%d]->x = %d, event[%d]->y = %d\n", event->valid, id, event->pendown, ts->touching_num, event->x, ts->touching_num, event->y);
    #endif
    ts->touching_num++;
    return 0;
}

/**
 * eeti_work() - work queue handler (initiated by the interrupt handler)
 * @work:      work queue to handle
 */
static void eeti_work(struct work_struct *work)
{
#ifdef TS_DELAY_WORK
    struct eeti *ts = container_of(to_delayed_work(work), struct eeti, work);
#else
    struct eeti *ts = container_of(work, struct eeti, work);
#endif
    struct ts_event *event;
    int i = 0;

    if (eeti_get_pendown_state(ts)) {
        if (eeti_read_sensor(ts) < 0) {
            printk(KERN_INFO "work read i2c failed\n");
            goto restart;
        }
        if (!ts->pendown) {
            ts->pendown = 1;
            //input_report_key(ts->input, BTN_TOUCH, 1);
            #ifdef EETI_TS_DEBUG_INFO
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
                    #ifdef EETI_TS_DEBUG_REPORT
                    printk(KERN_INFO "\nreport ABS_MT_TRACKING_ID %d\n", event->contactid);
                    #endif
                    input_report_abs(ts->input, ABS_MT_TOUCH_MAJOR, 1);
                    #ifdef EETI_TS_DEBUG_REPORT
                    printk(KERN_INFO "report ABS_MT_TOUCH_MAJOR %d\n", 1);
                    #endif
                    input_report_abs(ts->input, ABS_MT_WIDTH_MAJOR, 0);
                    #ifdef EETI_TS_DEBUG_REPORT
                    printk(KERN_INFO "report ABS_MT_WIDTH_MAJOR %d\n", 0);
                    #endif
                    input_report_abs(ts->input, ABS_MT_POSITION_X, event->x);
                    input_report_abs(ts->input, ABS_MT_POSITION_Y, event->y);
                    #ifdef EETI_TS_DEBUG_REPORT
                    printk(KERN_INFO "report ABS_MT_POSITION_XY %d,%d\n", event->x, event->y);
                    #endif
                    input_mt_sync(ts->input);
                    #ifdef EETI_TS_DEBUG_REPORT
                    printk(KERN_INFO "input_mt_sync\n");
                    #endif

                    if ((i == 0) && 
                        (ts->event[i].contactid != ts->event[i+1].contactid)
                        && ts->event[i+1].valid) //two fingers, just need one input_sync report.
                    {
                        continue;
                    }
                    #ifdef EETI_TS_DEBUG_REPORT
                    else
                    {
                        if (i == 0)
                            printk(KERN_INFO "[%d].contactid = %d, [%d].contactid = %d, [%d].valid = %d, [%d].pendown = %d\n", 
                            i, ts->event[i].contactid, i+1, ts->event[i+1].contactid, i+1, ts->event[i+1].valid, i+1, ts->event[i+1].pendown);
                    }
                    #endif
                    input_sync(ts->input);
                    #ifdef EETI_TS_DEBUG_REPORT
                    printk(KERN_INFO "input_sync\n");
                    #endif
                }
                #ifdef EETI_TS_DEBUG_READ
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
            #ifdef EETI_TS_DEBUG_INFO
            printk(KERN_INFO "UP\n");
            #endif
            input_report_abs(ts->input, ABS_MT_TOUCH_MAJOR, 0);
            #ifdef EETI_TS_DEBUG_REPORT
            printk(KERN_INFO "report ABS_MT_TOUCH_MAJOR %d\n", 0);
            #endif
            input_report_abs(ts->input, ABS_MT_WIDTH_MAJOR, 0);
            #ifdef EETI_TS_DEBUG_REPORT
            printk(KERN_INFO "report ABS_MT_WIDTH_MAJOR %d\n", 0);
            #endif
            input_mt_sync(ts->input);
            #ifdef EETI_TS_DEBUG_REPORT
            printk(KERN_INFO "input_mt_sync\n");
            #endif
            input_sync(ts->input);
            #ifdef EETI_TS_DEBUG_REPORT
            printk(KERN_INFO "input_sync\n");
            #endif
            eeti_reset(ts);
        }
        ts->touching_num = 0;
        enable_irq(ts->client->irq);
    }
}

#ifndef TS_DELAY_WORK
/**
 * eeti_timer() - timer callback function
 * @timer:     timer that caused this function call
 */
static enum hrtimer_restart eeti_timer(struct hrtimer *timer)
{
    struct eeti *ts = container_of(timer, struct eeti, timer);
    unsigned long flags = 0;
    
    spin_lock_irqsave(&ts->lock, flags);
//  printk(KERN_INFO "enter timer\n");
    queue_work(ts->workqueue, &ts->work);
    spin_unlock_irqrestore(&ts->lock, flags);
    return HRTIMER_NORESTART;
}
#endif

/**
 * eeti_interrupt() - interrupt handler for touch events
 * @irq:       interrupt to handle
 * @dev_id:    device-specific information
 */
static irqreturn_t eeti_interrupt(int irq, void *dev_id)
{
    struct i2c_client *client = (struct i2c_client *)dev_id;
    struct eeti *ts = i2c_get_clientdata(client);
    unsigned long flags;
    
    spin_lock_irqsave(&ts->lock, flags);
    #ifdef EETI_TS_DEBUG_REPORT
    printk(KERN_INFO "enter penirq\n");
    #endif
    /* if the pen is down, disable IRQ and start timer chain */
    if (eeti_get_pendown_state(ts)) {
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
 * eeti_probe() - initialize the I2C client
 * @client:    client to initialize
 * @id:                I2C device ID
 */
struct eeti_platform_data * eeti_data;
#ifdef CONFIG_HAS_EARLYSUSPEND
static void aml_eeti_early_suspend(struct early_suspend *h)
{
	printk("enter -----> %s \n",__FUNCTION__);
	eeti_data->touch_on(0);
}

static void aml_eeti_late_resume(struct early_suspend *h)
{
	printk("enter -----> %s \n",__FUNCTION__);
      eeti_data->touch_on(1);
}
#endif

static int eeti_probe(struct i2c_client *client,
    const struct i2c_device_id *id)
{
    struct eeti *ts;
    int err = 0;

    ts = kzalloc(sizeof(struct eeti), GFP_KERNEL);
    if (!ts) {
        err = -ENOMEM;
        goto fail;
    }

    ts->client = client;
    eeti_reset(ts);

    /* setup platform-specific hooks */
    ts->pdata = (struct eeti_platform_data*)client->dev.platform_data;
    eeti_data = (struct eeti_platform_data*)client->dev.platform_data;
    if (!ts->pdata || !ts->pdata->init_irq || !ts->pdata->get_irq_level) {
        dev_err(&client->dev, "no platform-specific callbacks "
            "provided\n");
        err = -ENXIO;
        goto fail;
    }
    else
    {
        ts->lcd_xmax = ((struct eeti_platform_data*) client->dev.platform_data)->lcd_max_width;
        ts->lcd_ymax = ((struct eeti_platform_data*) client->dev.platform_data)->lcd_max_height;
        ts->tp_xmax = ((struct eeti_platform_data*) client->dev.platform_data)->tp_max_width;
        ts->tp_ymax = ((struct eeti_platform_data*) client->dev.platform_data)->tp_max_height;
    }

    if (eeti_register_input(ts) < 0) {
        dev_err(&client->dev, "register input fail!\n");
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
    INIT_DELAYED_WORK(&ts->work, eeti_work);
#else
    hrtimer_init(&ts->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
    ts->timer.function = eeti_timer;
    INIT_WORK(&ts->work, eeti_work);
    ts->workqueue = create_singlethread_workqueue("eeti");
    if (ts->workqueue == NULL) {
        dev_err(&client->dev, "can't create work queue\n");
        err = -ENOMEM;
        goto fail;
    }
    #ifdef EETI_TS_DEBUG_REPORT
    printk("work create: %x\n", ts->workqueue);
    #endif
#endif

    ts->pendown = 0;
    ts->touching_num = 0;

    err = request_irq(client->irq, eeti_interrupt, IRQF_TRIGGER_FALLING,
        client->dev.driver->name, client);
    if (err) {
        dev_err(&client->dev, "failed to request IRQ#%d: %d\n",
        client->irq, err);
        goto fail_irq;
    }
    #ifdef CONFIG_HAS_EARLYSUSPEND
    printk("******* enter eeti early suspend register *******\n");
    eeti_early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN;
    eeti_early_suspend.suspend = aml_eeti_early_suspend;
    eeti_early_suspend.resume = aml_eeti_late_resume;
    eeti_early_suspend.param = client;
	register_early_suspend(&eeti_early_suspend);
    #endif
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
    eeti_read_sensor(ts);
    eeti_reset(ts);
    printk("eeti touch screen driver ok\n");
    return err;
}

/**
 * eeti_remove() - cleanup the I2C client
 * @client:    client to clean up
 */
static int eeti_remove(struct i2c_client *client)
{
    struct eeti *priv = i2c_get_clientdata(client);

    free_irq(client->irq, client);
    i2c_set_clientdata(client, NULL);
    input_unregister_device(priv->input);
    kfree(priv);
    #ifdef CONFIG_HAS_EARLYSUSPEND
      unregister_early_suspend(&eeti_early_suspend);
    #endif
    return 0;
}

static const struct i2c_device_id eeti_ids[] = {
    { DRIVER_NAME, 0 },
    { }
};

MODULE_DEVICE_TABLE(i2c, eeti_ids);
/* EETI I2C Capacitive Touch Screen driver */
static struct i2c_driver eeti_driver = {
    .driver = {
    .name = DRIVER_NAME,
    .owner = THIS_MODULE,
    },
    .probe = eeti_probe,
    .remove = __devexit_p(eeti_remove),
    .id_table = eeti_ids,
};

/**
 * eeti_init() - module initialization
 */
static int __init eeti_init(void)
{
    return i2c_add_driver(&eeti_driver);
}

/**
 * eeti_exit() - module cleanup
 */
static void __exit eeti_exit(void)
{
    i2c_del_driver(&eeti_driver);
}

module_init(eeti_init);
module_exit(eeti_exit);

MODULE_AUTHOR("");
MODULE_DESCRIPTION("eeti I2C Capacitive Touch Screen driver");
MODULE_LICENSE("GPL v2");
MODULE_VERSION(DRIVER_VERSION);


