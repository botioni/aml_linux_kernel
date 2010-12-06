/*
 * linux/drivers/input/touchscreen/capts.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/delay.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>
#include <linux/capts.h>


#define MULTI_TOUCH

#define capts_info  printk
#define capts_err  printk

/* periodic polling delay and period */
#define TS_POLL_DELAY     (1 * 1000000)
#define TS_POLL_PERIOD   (5 * 1000000)

/**
 * struct capts - touchscreen controller context
 * @dev:    device
 * @input:     touchscreen input device
 * @lock:      lock for resource protection
 * @work:     delayed work
 * @chip:   ts chip interface
 * @version:    chip version infomation
 * @event:     current touchscreen event
 * @event_num:     current event number
 */
struct capts {
    struct device *dev;
    struct input_dev *input;
    spinlock_t lock;
    struct hrtimer timer;
    struct work_struct work;
    struct workqueue_struct *workqueue;
    struct ts_chip *chip;
    struct ts_platform_data *pdata;
    struct ts_event event[EVENT_MAX];
    int event_num;
};


static struct input_dev* capts_register_input(const char *name, struct ts_info *info)
{
    struct input_dev *input;
    
    input = input_allocate_device();
    if (input) { 
        input->name = name;
    
#ifdef MULTI_TOUCH
        /* multi touch need only EV_ABS */
        set_bit(EV_ABS, input->evbit);
        set_bit(ABS_MT_POSITION_X, input->absbit);
        set_bit(ABS_MT_POSITION_Y, input->absbit);
        /* sim with ABS_PRESSURE in single touch */
        set_bit(ABS_MT_TOUCH_MAJOR, input->absbit);
        /* sim with ABS_TOOL_WIDTH in single touch */
        set_bit(ABS_MT_WIDTH_MAJOR, input->absbit);
        input_set_abs_params(input, ABS_MT_POSITION_X, info->xmin, info->xmax, 0, 0);
        input_set_abs_params(input, ABS_MT_POSITION_Y, info->ymin, info->ymax, 0, 0);
        input_set_abs_params(input, ABS_MT_TOUCH_MAJOR, info->zmin, info->zmax, 0, 0); 
        input_set_abs_params(input, ABS_MT_WIDTH_MAJOR, info->wmin, info->wmax, 0, 0);   
#else /*single touch */
        set_bit(EV_ABS | EV_SYN | EV_KEY, input->evbit);
        set_bit(BTN_TOUCH, input->keybit); 
        input_set_abs_params(input, ABS_X, info->xmin, info->xmax, 0, 0);
        input_set_abs_params(input, ABS_Y, info->ymin, info->ymax, 0, 0);
        input_set_abs_params(input, ABS_PRESSURE, info->zmin, info->zmax, 0, 0);
        input_set_abs_params(input, ABS_TOOL_WIDTH, info->wmin, info->wmax, 0, 0);
#endif
    
        if (input_register_device(input) < 0) {
            input_free_device(input);
            input = 0;
        }
    }
    
    return input;
}


static void capts_report_down(struct input_dev *input, struct ts_event *event, int event_num)
{
    int i;
    
#ifdef MULTI_TOUCH
    for (i=0; i<event_num; i++) {
        capts_info("point_%d: x=%d, y=%d, z=%d, w=%d\n",
            event->id, event->x, event->y, event->z, event->w);
        input_report_abs(input, ABS_MT_POSITION_X, event->x);
        input_report_abs(input, ABS_MT_POSITION_Y, event->y);
        input_report_abs(input, ABS_MT_TOUCH_MAJOR, event->z);
        input_report_abs(input, ABS_MT_WIDTH_MAJOR, event->w);
//        input_report_abs(input, ABS_MT_TRACKING_ID, event->id);
        input_mt_sync(input);
        event++;
    }
#else
    input_report_abs(input, ABS_X, event->x);
    input_report_abs(input, ABS_Y, event->y);
    input_report_abs(input, ABS_PRESSURE, event->z);
    input_report_abs(input, ABS_TOOL_WIDTH, event->w);
    input_report_key(input, BTN_TOUCH,  1);
#endif
    input_sync(input);
}


static void capts_report_up(struct input_dev *input)
{
#ifdef MULTI_TOUCH
    input_report_abs(input, ABS_MT_TOUCH_MAJOR, 0);
    input_report_abs(input, ABS_MT_WIDTH_MAJOR, 0);
    input_mt_sync(input);
#else
    input_report_abs(input, ABS_PRESSURE, 0);
    input_report_abs(input, ABS_TOOL_WIDTH, 0);
    input_report_key(input, BTN_TOUCH, 0);
#endif
    input_sync(input);
}


static ssize_t capts_read(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct capts *ts = (struct capts *)dev_get_drvdata(dev);
    
    if (!strcmp(attr->attr.name, "version")) {
        strcpy(buf, ts->chip->version);
        return strlen(ts->chip->version);
    }
    else if (!strcmp(attr->attr.name, "information")) {
        memcpy(buf, &ts->pdata->info, 8);
        return 8;
    } 
 
    return 0;
}

static ssize_t capts_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    struct capts *ts = (struct capts *)dev_get_drvdata(dev);

    if (!strcmp(attr->attr.name, "calibration")) {    
        capts_info("\n\ncalibrating... please don't touch your panel!\n");
        disable_irq_nosync(ts->pdata->irq);
        if (ts->chip->reset(ts->dev) < 0) {
            capts_err("calibration failed, please restart machine!\n\n");
            return 0;
        }
        else {
            capts_info("calibration ok\n\n");
            enable_irq(ts->pdata->irq);
        }
    }
    
    return count;
}

static DEVICE_ATTR(calibration, S_IRWXUGO, 0, capts_write);
static DEVICE_ATTR(version, S_IRWXUGO, capts_read, 0);
static DEVICE_ATTR(information, S_IRWXUGO, capts_read, 0);

static struct attribute *capts_attr[] = {
    &dev_attr_calibration.attr,
    &dev_attr_version.attr,
    &dev_attr_information.attr,
    NULL
};

static struct attribute_group capts_attr_group = {
    .name = NULL,
    .attrs = capts_attr,
};


/**
 * capts_work() - work queue handler (initiated by the interrupt handler)
 * @work:  delay work queue to handle
 */
static void capts_work(struct work_struct *work)
{
    struct capts *ts = container_of(work, struct capts, work);
    int event_num;

    if (!ts->pdata->get_irq_level()) {
        event_num = ts->chip->get_event(ts->dev, &ts->event[0]);
        event_num %= EVENT_MAX;
        if (event_num > 0) {
            capts_report_down(ts->input, ts->event, event_num);             
            if (!ts->event_num) {
                ts->event_num = event_num;
                capts_info( "DOWN\n");
            }
        }
        else {
             capts_err("read event failed, %d\n", event_num);
        }
        hrtimer_start(&ts->timer, ktime_set(0, TS_POLL_PERIOD), HRTIMER_MODE_REL);
    }
    else {
        if (ts->event_num) {
            capts_report_up(ts->input);
            ts->event_num = 0;
            capts_info( "UP\n");
        }
        /* enable IRQ after the pen was lifted */
        enable_irq(ts->pdata->irq);
    }
}


static enum hrtimer_restart capts_timer(struct hrtimer *timer)
{
	struct capts *ts = container_of(timer, struct capts, timer);
	unsigned long flags = 0;
	
	spin_lock_irqsave(&ts->lock, flags);
	queue_work(ts->workqueue, &ts->work);	
	spin_unlock_irqrestore(&ts->lock, flags);
	return HRTIMER_NORESTART;
}


/**
 * capts_interrupt() - interrupt handler for touch events
 * @irq:       interrupt to handle
 * @context:    device-specific information
 */
static irqreturn_t capts_interrupt(int irq, void *context)
{
    struct capts *ts = (struct capts *) context;
    unsigned long flags;
    
    capts_info( "enter interrrupt\n");
    spin_lock_irqsave(&ts->lock, flags);
    /* if the pen is down, disable IRQ and start timer chain */
    if (!ts->pdata->get_irq_level()) {
        disable_irq_nosync(ts->pdata->irq);
        hrtimer_start(&ts->timer, ktime_set(0, TS_POLL_DELAY), HRTIMER_MODE_REL);
    }
    spin_unlock_irqrestore(&ts->lock, flags);
    
    return IRQ_HANDLED;
}

/**
 * capts_probe()
 * @dev:    device to initialize
 * @chip:   chip interface    
 */
int capts_probe(struct device *dev, struct ts_chip *chip)
{
    struct capts *ts;
    struct ts_platform_data *pdata = dev->platform_data;
    int err = 0;

    capts_info("\ncapaticive touchscreen probe start\n");
   
    if (!chip || !pdata || !pdata->init_irq || !pdata->get_irq_level) {
        err = -ENODEV;
        capts_err("no chip registered\n");
        goto fail;
    }
     
    ts = kzalloc(sizeof(struct capts), GFP_KERNEL);
    if (!ts) {
        err = -ENOMEM;
        capts_err("allocate ts failed\n");
        goto fail;
    }
    
    ts->dev = dev;
    ts->pdata = pdata;
    ts->chip = chip;
    ts->event_num = 0;
    dev_set_drvdata(dev, ts);
    spin_lock_init(&ts->lock);

    hrtimer_init(&ts->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
    ts->timer.function = capts_timer;
    INIT_WORK(&ts->work, capts_work);
    ts->workqueue = create_singlethread_workqueue(dev->driver->name);
    if (ts->workqueue == NULL) {
        err = -ENOMEM;
        capts_err("can't create work queue\n");
        goto fail;
    }
    
    pdata->init_irq();
    err = request_irq(ts->pdata->irq, capts_interrupt,
            IRQF_TRIGGER_FALLING, dev->driver->name, ts);
    if (err) {
        capts_err("request gpio irq failed\n");
        goto fail;
    }

    disable_irq_nosync(ts->pdata->irq);
    capts_info("reseting...\n");
    err = ts->chip->reset(ts->dev);
    if (err) {
        capts_err("reset failed\n");
        goto fail;
    }
    capts_info("reset ok\n");
    enable_irq(ts->pdata->irq);
    
    ts->input = capts_register_input(dev->driver->name, &ts->pdata->info);
    if (!ts->input) {
        err = -ENOMEM;
        goto fail;
    }
    
    err = sysfs_create_group(&dev->kobj, &capts_attr_group);
    if (err) {
        capts_err("create device attribute group failed\n");
        goto fail;
    }

    capts_info("capaticive touchscreen probe ok\n");
    return 0;
    
fail:
    capts_info("capaticive touchscreen probe failed\n");
    dev_set_drvdata(dev, NULL);
    if (ts) {
        free_irq(ts->pdata->irq, ts);
        if (ts->input) {
            input_free_device(ts->input);
        }
        kfree(ts);
    }
    return err;
}

/**
 * capts_remove()
 * @dev:    device to clean up
 */
int capts_remove(struct device *dev)
{
       struct capts *ts = dev_get_drvdata(dev);
       free_irq(ts->pdata->irq, ts);
       dev_set_drvdata(dev, NULL);
       input_unregister_device(ts->input);
       kfree(ts);

       return 0;
}
