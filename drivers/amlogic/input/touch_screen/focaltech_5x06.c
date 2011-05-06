/*
 * linux/drivers/input/touchscreen/FT5x06.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/capts.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
	#include <linux/earlysuspend.h>
	static struct early_suspend ft_early_suspend;
#endif

#define DRIVER_NAME         "ft5x06"
#define DRIVER_VERSION   "1"

#define ft5x06_info printk

#define TD_STATUS 0x02
#define FT5X06_PACKET_SIZE            29

#define get_bits(val, start_bit, bit_num) ((val >> start_bit) & ((1 << bit_num) - 1))

#define buf_to_short(buf)   ((*buf << 8) +*(buf+1))

int ft5x06_reset(struct device *dev);
int ft5x06_calibration(struct device *dev);
int ft5x06_get_event (struct device *dev, struct ts_event *event);
#ifdef CONFIG_HAS_EARLYSUSPEND
static int ft5x06_suspend(struct early_suspend *handler);
static int ft5x06_resume(struct early_suspend *handler);
#endif
struct ts_chip ft5x06_chip = {
    .name = DRIVER_NAME,
    .version = DRIVER_VERSION,
    .reset = ft5x06_reset,
    .calibration = ft5x06_calibration,
    .get_event = ft5x06_get_event,
};

/*
* CRC16 implementation acording to CCITT standards
*/

static int ft5x06_write_block(struct i2c_client *client, u8 addr, u8 *buf, int len)
{ 
    struct i2c_msg msg[2] = {
        [0] = {
            .addr = client->addr,
            .flags = client->flags,
            .len = 1,
            .buf = &addr
        },
        [1] = {
            .addr = client->addr,
            .flags = client->flags | I2C_M_NOSTART,
            .len = len,
            .buf = buf
        },
    };
    int msg_num = len ? ARRAY_SIZE(msg) : 1;
    return i2c_transfer(client->adapter, msg, msg_num);
}


static int ft5x06_read_block(struct i2c_client *client, u8 addr, u8 *buf, int len)
{ 
    struct i2c_msg msg[2] = {
        [0] = {
            .addr = client->addr,
            .flags = client->flags,
            .len = 1,
            .buf = &addr
        },
        [1] = {
            .addr = client->addr,
            .flags = client->flags | I2C_M_RD,
            .len = len,
            .buf = buf
        },
    };
    
    return i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg));
}

	
int ft5x06_reset(struct device *dev)
{
	struct ts_platform_data *pdata = dev->platform_data;
	if(pdata&&pdata->power_off&&pdata->power_on){
		pdata->power_off();
		mdelay(50);
		pdata->power_on();
	}
    return 0;   

}
    

int ft5x06_calibration(struct device *dev)
{
	int ret = -1;
	char buf=0x00;
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	ret = raydium_write_block(client,0xa0,&buf,1);
	mdelay(500);
	return ret;
}

int ft5x06_get_event (struct device *dev, struct ts_event *event)

{
    struct i2c_client *client = container_of(dev, struct i2c_client, dev);
    struct ts_platform_data *pdata = dev->platform_data;
    struct ts_info *info = &pdata->info;
    u8 buf[FT5X06_PACKET_SIZE];
    int event_num;
    int i, ba;

    memset(buf, 0, ARRAY_SIZE(buf));
    if (ft5x06_read_block(client, TD_STATUS,
            buf, FT5X06_PACKET_SIZE) < 0) {    
        /* i2c read failed */
        ft5x06_info("ft5x06 read i2c failed!\n");
        return -1;
    }
    ft5x06_info("org data =%d, %d, %d, %d, %d,", buf[0], buf[1], buf[2], buf[3],buf[4]);
    ft5x06_info("%d, %d, %d, %d\n", buf[7], buf[8], buf[9], buf[10]);

    event_num = 0;
    event_num = get_bits(buf[0], 0, 4);
    event_num %= EVENT_MAX;

    for (i=0; i<event_num; i++) {
        ba = i*6;
        event->x = (buf[ba] << 8) | buf[ba+1];
        event->y = (buf[ba+2] << 8) | buf[ba+3];
        if (info->swap_xy) {
            swap(event->x, event->y);
        }
        if (info->x_pol) {
            event->x = info->xmax + info->xmin - event->x;
        }
        if (info->y_pol) {
            event->y = info->ymax + info->ymin - event->y;
        }
        if (event->x || event->y) {
            event->z = 1;
            event->w = 1;
            event++;
            event_num++;
        }
    }
    return event_num;
}

static int ft5x06_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret;
	ret = capts_probe(&client->dev, &ft5x06_chip);
#ifdef CONFIG_HAS_EARLYSUSPEND
		ft_early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN;
		ft_early_suspend.suspend = ft5x06_suspend;
		ft_early_suspend.resume = ft5x06_resume;
		ft_early_suspend.param = client;
		register_early_suspend(&ft_early_suspend);
#endif
    return ret;
}


static int ft5x06_remove(struct i2c_client *client)
{
#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&ft_early_suspend);
#endif
    return capts_remove(&client->dev);
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static int ft5x06_suspend(struct early_suspend *handler)
{
	int ret = -1;
	pm_message_t msg={0};
	if(handler && handler->param) {
		struct i2c_client *client = (struct i2c_client *)handler->param;
		ret = capts_suspend(&client->dev, msg);
	}
	return ret;
}

static int ft5x06_resume(struct early_suspend *handler)
{
 	int ret = -1;
	if(handler && handler->param) {
		struct i2c_client *client = (struct i2c_client *)handler->param;
		ret = capts_resume(&client->dev);
	}
	return ret;
}
#else
static int ft5x06_suspend(struct i2c_client *client, pm_message_t msg)
{
    return capts_suspend(&client->dev, msg);
}

static int ft5x06_resume(struct i2c_client *client)
{
    return capts_resume(&client->dev);
}
#endif

static const struct i2c_device_id ft5x06_ids[] = {
   { DRIVER_NAME, 0 },
   { }
};

MODULE_DEVICE_TABLE(i2c, ft5x06_ids);

static struct i2c_driver ft5x06_driver = {
    .driver = {
        .name = DRIVER_NAME,
        .owner = THIS_MODULE,
    },
    .probe = ft5x06_probe,
    .remove = ft5x06_remove,
    .suspend = ft5x06_suspend,
    .resume = ft5x06_resume,
    .id_table = ft5x06_ids,
};

static int __init ft5x06_init(void)
{
       return i2c_add_driver(&ft5x06_driver);
}

static void __exit ft5x06_exit(void)
{
       i2c_del_driver(&ft5x06_driver);
}

module_init(ft5x06_init);
module_exit(ft5x06_exit);

MODULE_AUTHOR("");
MODULE_DESCRIPTION("ft5x06 capacitive touch screen driver");
MODULE_LICENSE("GPL v2");
MODULE_VERSION(DRIVER_VERSION);
