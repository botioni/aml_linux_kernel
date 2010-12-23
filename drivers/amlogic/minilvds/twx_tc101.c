//FOR MINILVDS Driver TWX_TC101

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/jiffies.h> 
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <linux/miscdevice.h>
#include <linux/mutex.h>
#include <linux/mm.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/delay.h>
#include <linux/sysctl.h>
#include <asm/uaccess.h>
#include <mach/pinmux.h>
#include <mach/gpio.h>
#include <linux/twx_tc101.h>

static struct i2c_client *twx_tc101_client;
static int twx_tc101_i2c_read(unsigned char *buff, unsigned len)
{
    int res = 0;
    struct i2c_msg msgs[] = {
        {
            .addr = twx_tc101_client->addr,
            .flags = 0,
            .len = 1,
            .buf = buff,
        },
        {
            .addr = twx_tc101_client->addr,
            .flags = I2C_M_RD,
            .len = len,
            .buf = buff,
        }
    };

    res = i2c_transfer(twx_tc101_client->adapter, msgs, 2);
    if (res < 0) {
        pr_err("%s: i2c transfer failed\n", __FUNCTION__);
    }

    return res;
}

static int twx_tc101_i2c_write(unsigned char *buff, unsigned len)
{
    int res = 0;
    struct i2c_msg msg[] = {
        {
        .addr = twx_tc101_client->addr,
        .flags = 0,
        .len = len,
        .buf = buff,
        }
    };

    res = i2c_transfer(twx_tc101_client->adapter, msg, 1);
    if (res < 0) {
        pr_err("%s: i2c transfer failed\n", __FUNCTION__);
    }

    return res;
}

static int twx_tc101_send(unsigned short addr, u8 data)
{
	u8 buf[3]= {addr >> 8, addr&0xff, data};
	return twx_tc101_i2c_write(buf, 3);
}


static u8 twx_tc101_recv(unsigned short addr)
{

   int res = 0;
 	u8 buf[2]= {addr >> 8, addr&0xff };
	u8 data = 0x69;
    struct i2c_msg msgs[] = {
        {
            .addr = twx_tc101_client->addr,
            .flags = 0,
            .len = 2,
            .buf = buf,
        },
        {
            .addr = twx_tc101_client->addr,
            .flags = I2C_M_RD,
            .len = 1,
            .buf = &data,
        }
    };
    res = i2c_transfer(twx_tc101_client->adapter, msgs, 2);
	pr_err("i2c return=%d\n", res);
   if (res < 0) {
        pr_err("%s: i2c transfer failed\n", __FUNCTION__);
   }
   else
		res = data;

    return res;
}

static int twx_tc101_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    int res = 0;
    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
        pr_err("%s: functionality check failed\n", __FUNCTION__);
        res = -ENODEV;
        goto out;
    }
    twx_tc101_client = client;
	twx_tc101_send(0xf830, 0xb2);msleep(10);//
	twx_tc101_send(0xf831, 0xf0);msleep(10);//
	twx_tc101_send(0xf833, 0xc2);msleep(10);//
	twx_tc101_send(0xf840, 0x80);msleep(10);//
	twx_tc101_send(0xf881, 0xec);msleep(10);//

	//twx_tc101_send(0xf841, 0x02);msleep(10);
	//twx_tc101_send(0xf882, 0x18);msleep(10);

	// twx_tc101_send(0xf820, 0x41);msleep(10);
	// twx_tc101_send(0xf826, 0x1b);msleep(10);
	// twx_tc101_send(0xf827, 0x12);msleep(10);
	//while(1)
	{
	//twx_tc101_send(0xf830, 0xb2);msleep(10);//
		// twx_tc101_send(0xf827, 0x12);
		// msleep(10);
		// printk("0xf820=%x\n",twx_tc101_recv(0xf820));
		// msleep(10);
		// printk("0xf833=%x\n",twx_tc101_recv(0xf833));
		//msleep(10);
		// printk("0xf830=%x\n",twx_tc101_recv(0xf830));
		// msleep(20);
	}
		
out:
    return res;
}

static int twx_tc101_remove(struct i2c_client *client)
{
    return 0;
}

static const struct i2c_device_id twx_tc101_id[] = {
    { "twx_tc101", 0 },
    { }
};

static struct i2c_driver twx_tc101_driver = {
    .probe = twx_tc101_probe,
    .remove = twx_tc101_remove,
    .id_table = twx_tc101_id,
    .driver = {
    .name = "twx_tc101",
    },
};

static int __init twx_tc101_init(void)
{
    int res;
	
	printk("\n\nMINI LVDS Driver Init.\n\n");

    if (twx_tc101_client)
    {
        res = 0;
    }
    else
    {
        res = i2c_add_driver(&twx_tc101_driver);
        if (res < 0) {
            printk("add twx_tc101 i2c driver error\n");
        }
    }

    return res;
}

//arch_initcall(twx_tc101_init);
module_init(twx_tc101_init);

MODULE_AUTHOR("AMLOGIC");
MODULE_DESCRIPTION("MINILVDS driver for TWX_TC101");
MODULE_LICENSE("GPL");
