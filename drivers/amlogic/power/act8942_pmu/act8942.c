/*
 * PMU driver for ACT8942
 *
 * Copyright (c) 2010-2011 Amlogic Ltd.
 *	Elvis Yu <elvis.yu@amlogic.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
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
#include <asm/uaccess.h>
#include <mach/am_regs.h>
#include <mach/pinmux.h>
#include <mach/gpio.h>
#include <linux/saradc.h>
#include <linux/act8942.h>  


#define DRIVER_VERSION			"0.1.0"
#define	ACT8942_DEVICE_NAME		"pmu_act8942"
#define	ACT8942_CLASS_NAME		"act8942_class"
#define ACT8942_I2C_NAME		"act8942-i2c"

/* If the system has several batteries we need a different name for each
 * of them...
 */
static DEFINE_IDR(pmu_id);
static DEFINE_MUTEX(pmu_mutex);

struct act8942_device_info *act8942_dev;	//Elvis Fool

struct act8942_operations* act8942_opts = NULL;

static dev_t act8942_devno;

typedef struct pmu_dev_s {
    /* ... */
    struct cdev	cdev;             /* The cdev structure */
} pmu_dev_t;

static pmu_dev_t *act8942_pmu_dev = NULL;

struct act8942_device_info {
	struct device 		*dev;
	int			id;
	struct power_supply	bat;
	struct power_supply	ac;	
	struct power_supply	usb;
	
	struct i2c_client	*client;

	struct timer_list polling_timer;
	struct work_struct work_update;
};

static struct i2c_client *this_client;

static int act8942_read_i2c(struct i2c_client *client, u8 reg, u8 *val);

static enum power_supply_property bat_power_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_TEMP,
//	POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW,
//	POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG,
//	POWER_SUPPLY_PROP_TIME_TO_FULL_NOW,
};

static enum power_supply_property ac_power_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
};

static enum power_supply_property usb_power_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
};

/*
 *	ACINSTAT
 *	ACIN Status. Indicates the state of the ACIN input, typically
 *	in order to identify the type of input supply connected. Value
 *	is 1 when ACIN is above the 1.2V precision threshold, value
 *	is 0 when ACIN is below this threshold.
 */
static inline int is_ac_online(void)
{
	u8 val;	
	act8942_read_i2c(this_client, (ACT8942_APCH_ADDR+0xa), &val);
	
	logd("%s: get from pmu is %d.\n", __FUNCTION__, val);	
	return	(val & 0x2) ? 1 : 0;
}

static inline int is_usb_online(void)
{
	u8 val;
	act8942_read_i2c(this_client, (ACT8942_APCH_ADDR+0xa), &val);
	logd("%s: get from pmu is %d.\n", __FUNCTION__, val);
	//return	(val & 0x2) ? 0 : 1;
	return 0;
}


/*
 *	Charging Status Indication
 *
 *	CSTATE[1]	CSTATE[0]	STATE MACHINE STATUS
 *
 *		1			1		PRECONDITION State
 *		1			0		FAST-CHARGE / TOP-OFF State
 *		0			1		END-OF-CHARGE State
 *		0			0		SUSPEND/DISABLED / FAULT State
 *
 */
static inline int get_charge_status(void)
{
	u8 val;
	
	act8942_read_i2c(this_client, (ACT8942_APCH_ADDR+0xa), &val);

	logd("%s: get from pmu is %d.\n", __FUNCTION__, val);
	
	return ((val>>4) & 0x3);
}


#define to_act8942_device_info(x) container_of((x), \
				struct act8942_device_info, bat);

static int bat_power_get_property(struct power_supply *psy,
					enum power_supply_property psp,
					union power_supply_propval *val)
{
	int ret = 0;
	u8 status;
	struct act8942_device_info *act8942_dev = to_act8942_device_info(psy);

	switch (psp)
	{
		case POWER_SUPPLY_PROP_STATUS:
			if(act8942_opts->is_ac_online())
			{
				status = act8942_opts->get_charge_status();
				if(act8942_opts->get_charge_status == get_charge_status)
				{
					if(status == 0x1)
					{
						val->intval = POWER_SUPPLY_STATUS_FULL;
					}
					else if(status == 0x0)
					{
						val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
					}
					else
					{
						val->intval = POWER_SUPPLY_STATUS_CHARGING;
					}
				}
				else
				{
					if(status == 0x1)
					{
						val->intval = POWER_SUPPLY_STATUS_FULL;
					}
					else if(status == 0x0)
					{
						val->intval = POWER_SUPPLY_STATUS_CHARGING;
					}
				}
			}
			else
			{
				val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
			}
			break;
		case POWER_SUPPLY_PROP_VOLTAGE_NOW:
			val->intval = act8942_opts->measure_voltage();
			break;
		case POWER_SUPPLY_PROP_PRESENT:
			val->intval = val->intval <= 0 ? 0 : 1;
			break;
		case POWER_SUPPLY_PROP_CURRENT_NOW:
			val->intval = act8942_opts->measure_current();
			break;
		case POWER_SUPPLY_PROP_CAPACITY:
			if(act8942_opts->is_ac_online())
			{
				val->intval = act8942_opts->measure_capacity_charging();
			}
			else
			{
				val->intval = act8942_opts->measure_capacity_battery();
			}
			break;
		case POWER_SUPPLY_PROP_TEMP:
			val->intval = NULL;		//temporary
			break;
//	case POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW:
//		ret = bq27x00_battery_time(di, BQ27x00_REG_TTE, val);
//		break;
//	case POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG:
//		ret = bq27x00_battery_time(di, BQ27x00_REG_TTECP, val);
//		break;
//	case POWER_SUPPLY_PROP_TIME_TO_FULL_NOW:
//		ret = bq27x00_battery_time(di, BQ27x00_REG_TTF, val);
//		break;
	    case POWER_SUPPLY_PROP_TECHNOLOGY:
	        val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
	        break;
		case POWER_SUPPLY_PROP_HEALTH:	
			val->intval = POWER_SUPPLY_HEALTH_GOOD;
			break;
		default:
			return -EINVAL;
	}

	return ret;
}

static int ac_power_get_property(struct power_supply *psy,
		enum power_supply_property psp, union power_supply_propval *val)
{
	int retval = 0;

	switch (psp)
	{
		case POWER_SUPPLY_PROP_ONLINE:
		val->intval = act8942_opts->is_ac_online();
		break;
		case POWER_SUPPLY_PROP_VOLTAGE_NOW:	
		val->intval = act8942_opts->measure_voltage();
		break;
		default:
		return -EINVAL;
	}

	return retval;
}

static int usb_power_get_property(struct power_supply *psy,
		enum power_supply_property psp, union power_supply_propval *val)
{
	int retval = 0;

	switch (psp)
	{
		case POWER_SUPPLY_PROP_ONLINE:
		val->intval = act8942_opts->is_usb_online();	//temporary
		break;
		case POWER_SUPPLY_PROP_VOLTAGE_NOW:	
		val->intval = act8942_opts->measure_voltage();
		break;
		default:
		return -EINVAL;
	}

	return retval;
}

static char *power_supply_list[] = {
	"Battery",
};

static void act8942_powersupply_init(struct act8942_device_info *act8942_dev)
{
	act8942_dev->bat.name = "bat";
	act8942_dev->bat.type = POWER_SUPPLY_TYPE_BATTERY;
	act8942_dev->bat.properties = bat_power_props;
	act8942_dev->bat.num_properties = ARRAY_SIZE(bat_power_props);
	act8942_dev->bat.get_property = bat_power_get_property;
	act8942_dev->bat.external_power_changed = NULL;

    act8942_dev->ac.name = "ac";
	act8942_dev->ac.type = POWER_SUPPLY_TYPE_MAINS;
	act8942_dev->ac.supplied_to = power_supply_list,
	act8942_dev->ac.num_supplicants = ARRAY_SIZE(power_supply_list),
	act8942_dev->ac.properties = ac_power_props;
	act8942_dev->ac.num_properties = ARRAY_SIZE(ac_power_props);
	act8942_dev->ac.get_property = ac_power_get_property;

    act8942_dev->usb.name = "usb";
	act8942_dev->usb.type = POWER_SUPPLY_TYPE_USB;
	act8942_dev->usb.supplied_to = power_supply_list,
	act8942_dev->usb.num_supplicants = ARRAY_SIZE(power_supply_list),
	act8942_dev->usb.properties = usb_power_props;
	act8942_dev->usb.num_properties = ARRAY_SIZE(usb_power_props);
	act8942_dev->usb.get_property = usb_power_get_property;	
}

#ifdef CONFIG_USB_ANDROID
int pc_connect(int status) 
{
	//Elvis empty
    return 0;
} 

EXPORT_SYMBOL(pc_connect);

#endif

static void update_work_func(struct work_struct *work)
{
	logd("%s: work->data is 0x%x.\n", __FUNCTION__, work->data);
	if(!is_ac_online()){
		power_supply_changed(&act8942_dev->bat); 
	}
	else
	{
    //if((get_charge_status() > 0x0)){
		power_supply_changed(&act8942_dev->ac);      
	}
}

static void polling_func(unsigned long arg)
{
	struct act8942_device_info *act8942_dev = (struct act8942_device_info *)arg;
	schedule_work(&(act8942_dev->work_update));
	mod_timer(&(act8942_dev->polling_timer), jiffies + msecs_to_jiffies(act8942_opts->update_period));  
}


/*
 * i2c specific code
 */

static int act8942_read_i2c(struct i2c_client *client, u8 reg, u8 *val)
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

	err = i2c_transfer(client->adapter, msgs, 2);

	return err;
}

static int act8942_write_i2c(struct i2c_client *client, u8 reg, u8 *val)
{
	int err;
	unsigned char buff[2];
    buff[0] = reg;
    buff[1] = val;
    struct i2c_msg msgs[] = {
        {
        .addr = client->addr,
        .flags = 0,
        .len = 2,
        .buf = buff,
        }
    };

	err = i2c_transfer(client->adapter, msgs, 1);

	return err;
}


static inline void	act8942_dump(struct i2c_client *client)
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

	ret = act8942_read_i2c(client, (ACT8942_REG3_ADDR+2), &val);
	pr_info("act8942: [0x%x] : 0x%x\n", ACT8942_REG3_ADDR+2, val);

	ret = act8942_read_i2c(client, ACT8942_REG4_ADDR, &val);
	pr_info("act8942: [0x%x] : 0x%x\n", ACT8942_REG4_ADDR, val);

	ret = act8942_read_i2c(client, (ACT8942_REG4_ADDR+1), &val);
	pr_info("act8942: [0x%x] : 0x%x\n", ACT8942_REG4_ADDR+1, val);

	ret = act8942_read_i2c(client, ACT8942_REG5_ADDR, &val);
	pr_info("act8942: [0x%x] : 0x%x\n", ACT8942_REG5_ADDR, val);

	ret = act8942_read_i2c(client, (ACT8942_REG5_ADDR+1), &val);
	pr_info("act8942: [0x%x] : 0x%x\n", ACT8942_REG5_ADDR+1, val);

	ret = act8942_read_i2c(client, ACT8942_REG6_ADDR, &val);
	pr_info("act8942: [0x%x] : 0x%x\n", ACT8942_REG6_ADDR, val);

	ret = act8942_read_i2c(client, (ACT8942_REG6_ADDR+1), &val);
	pr_info("act8942: [0x%x] : 0x%x\n", ACT8942_REG6_ADDR+1, val);

	ret = act8942_read_i2c(client, ACT8942_REG7_ADDR, &val);
	pr_info("act8942: [0x%x] : 0x%x\n", ACT8942_REG7_ADDR, val);

	ret = act8942_read_i2c(client, (ACT8942_REG7_ADDR+1), &val);
	pr_info("act8942: [0x%x] : 0x%x\n", ACT8942_REG7_ADDR+1, val);

	ret = act8942_read_i2c(client, ACT8942_APCH_ADDR, &val);
	pr_info("act8942: [0x%x] : 0x%x\n", ACT8942_APCH_ADDR, val);

	ret = act8942_read_i2c(client, (ACT8942_APCH_ADDR+1), &val);
	pr_info("act8942: [0x%x] : 0x%x\n", ACT8942_APCH_ADDR+1, val);

	ret = act8942_read_i2c(client, (ACT8942_APCH_ADDR+8), &val);
	pr_info("act8942: [0x%x] : 0x%x\n", ACT8942_APCH_ADDR+8, val);

	ret = act8942_read_i2c(client, (ACT8942_APCH_ADDR+9), &val);
	pr_info("act8942: [0x%x] : 0x%x\n", ACT8942_APCH_ADDR+9, val);

	ret = act8942_read_i2c(client, (ACT8942_APCH_ADDR+0xa), &val);
	pr_info("act8942: [0x%x] : 0x%x\n", ACT8942_APCH_ADDR+0xa, val);
}

ssize_t act8942_info_show(struct class *class, struct class_attribute *attr, char *buf)
{
	act8942_dump(this_client);
	return 0;
}

static struct class_attribute act8942_class_attrs[] = {
	__ATTR(info, S_IRUGO | S_IWUSR, act8942_info_show, NULL),
	__ATTR_NULL
};

static struct class act8942_class = {
    .name = ACT8942_CLASS_NAME,
    .class_attrs = act8942_class_attrs,
};



/*
 *	Fast charge when CHG_CON(GPIOAO_11) is High.
 *	Slow charge when CHG_CON(GPIOAO_11) is Low.
 */
static int set_charge_current(int level)
{
	set_gpio_mode(GPIOAO_bank_bit0_11(11), GPIOAO_bit_bit0_11(11), GPIO_OUTPUT_MODE);
	set_gpio_val(GPIOAO_bank_bit0_11(11), GPIOAO_bit_bit0_11(11), (level ? 1 : 0));
	return 0;
}

static int act8942_operations_init(struct act8942_operations* pdata)
{
	act8942_opts = pdata;
	if(act8942_opts->is_ac_online == NULL)
	{
		act8942_opts->is_ac_online = is_ac_online;
	}
	if(act8942_opts->is_usb_online == NULL)
	{
		act8942_opts->is_usb_online = is_usb_online;
	}
	if(act8942_opts->set_bat_off== NULL)
	{
		pr_err("act8942_opts->measure_voltage is NULL!\n");
		return -1;
	}
	if(act8942_opts->get_charge_status == NULL)
	{
		act8942_opts->get_charge_status = get_charge_status;
	}
	if(act8942_opts->set_charge_current == NULL)
	{
		act8942_opts->set_charge_current = set_charge_current;
	}
	if(act8942_opts->measure_voltage == NULL)
	{
		pr_err("act8942_opts->measure_voltage is NULL!\n");
		return -1;
	}
	if(act8942_opts->measure_current == NULL)
	{
		pr_err("act8942_opts->measure_current is NULL!\n");
		return -1;
	}
	if(act8942_opts->measure_capacity_charging == NULL)
	{
		pr_err("act8942_opts->measure_capacity is NULL!\n");
		return -1;
	}
	if(act8942_opts->measure_capacity_battery== NULL)
	{
		pr_err("act8942_opts->measure_capacity is NULL!\n");
		return -1;
	}
	if(act8942_opts->update_period <= 0)
	{
		act8942_opts->update_period = 5000;
	}
	return 0;
}


#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>

static void act8942_suspend(struct early_suspend *h)
{
	set_charge_current(1);
	pr_info("fast charger on early_suspend\n\n");    
}

static void act8942_resume(struct early_suspend *h)
{
    set_charge_current(0);
	pr_info("slow charger on resume\n\n");
}


static struct early_suspend act8942_early_suspend = {
	.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN,
	.suspend = act8942_suspend,
	.resume = act8942_resume,
	.param = NULL,
};
#endif


static int act8942_i2c_probe(struct i2c_client *client,
				 const struct i2c_device_id *id)
{
	//struct act8942_device_info *act8942_dev;	//Elvis Fool
	int num;
	int retval = 0;
		
	pr_info("act8942_i2c_probe\n");

	if(act8942_operations_init((struct act8942_operations*)client->dev.platform_data))
	{
		dev_err(&client->dev, "failed to init act8942_opts!\n");
		return -EINVAL;
	}
	
	/* Get new ID for the new PMU device */
	retval = idr_pre_get(&pmu_id, GFP_KERNEL);
	if (retval == 0)
	{
		return -ENOMEM;
	}

	mutex_lock(&pmu_mutex);
	retval = idr_get_new(&pmu_id, client, &num);
	mutex_unlock(&pmu_mutex);
	if (retval < 0)
	{
		return retval;
	}

	act8942_dev = kzalloc(sizeof(*act8942_dev), GFP_KERNEL);
	if (!act8942_dev) {
		dev_err(&client->dev, "failed to allocate device info data\n");
		retval = -ENOMEM;
		goto act8942_failed_2;
	}
	act8942_dev->id = num;
	//act8942_dev->chip = id->driver_data; //elvis

	this_client = client;

	i2c_set_clientdata(client, act8942_dev);
	act8942_dev->dev = &client->dev;
	act8942_dev->client = client;

	act8942_powersupply_init(act8942_dev);

	retval = power_supply_register(&client->dev, &act8942_dev->bat);
	if (retval) {
		dev_err(&client->dev, "failed to register battery\n");
		goto act8942_failed_2;
	}

	retval = power_supply_register(&client->dev, &act8942_dev->ac);
	if (retval) {
		dev_err(&client->dev, "failed to register ac\n");
		goto act8942_failed_2;
	}
	
	retval = power_supply_register(&client->dev, &act8942_dev->usb);
	if (retval) {
		dev_err(&client->dev, "failed to register usb\n");
		goto act8942_failed_2;
	}

	INIT_WORK(&(act8942_dev->work_update), update_work_func);
	
	init_timer(&(act8942_dev->polling_timer));
	act8942_dev->polling_timer.expires = jiffies + msecs_to_jiffies(act8942_opts->update_period);
	act8942_dev->polling_timer.function = polling_func;
	act8942_dev->polling_timer.data = act8942_dev;
    add_timer(&(act8942_dev->polling_timer));

#ifdef CONFIG_HAS_EARLYSUSPEND
	register_early_suspend(&act8942_early_suspend);
#endif
	dev_info(&client->dev, "support ver. %s enabled\n", DRIVER_VERSION);

	return 0;

act8942_failed_2:
	kfree(act8942_dev);
act8942_failed_1:
	mutex_lock(&pmu_mutex);
	idr_remove(&pmu_id, num);
	mutex_unlock(&pmu_mutex);

	return retval;
}

static int act8942_i2c_remove(struct i2c_client *client)
{
	struct act8942_device_info *act8942_dev = i2c_get_clientdata(client);
	pr_info("act8942_i2c_remove\n");
	power_supply_unregister(&act8942_dev->bat);

	del_timer(&(act8942_dev->polling_timer));
	
	kfree(act8942_dev->bat.name);

	mutex_lock(&pmu_mutex);
	idr_remove(&pmu_id, act8942_dev->id);
	mutex_unlock(&pmu_mutex);

	kfree(act8942_dev);

#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&act8942_early_suspend);
#endif
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
	int ret = 0;
	int size, i;
	act8942_i2c_msg_t *msgs = NULL;
	//struct i2c_client *client = (struct i2c_client *)file->private_data;
	struct i2c_client *client = this_client;

	if(_IOC_DIR(cmd) == _IOC_READ)
	{
		size = _IOC_SIZE(cmd);
		if(size)
		{

			msgs = kmalloc((sizeof(act8942_i2c_msg_t)*size), GFP_KERNEL);
			if(!msgs)
			{
				pr_err("act8942_ioctl: failed to allocate memory for act8942_i2c_msgs.\n");
				return -ENOMEM;
			}

			ret = copy_from_user(msgs, (void __user *)arg, (sizeof(act8942_i2c_msg_t)*size));
			if(ret)
            {
                pr_err("act8942_ioctl: copy_from_user failed!\n ");
				kfree(msgs);
				return ret;
            }

			for(i=0; i<size; i++)
			{
				act8942_read_i2c(client, msgs[i].reg, &msgs[i].val);
			}

			copy_to_user((void __user *)arg, msgs, (sizeof(act8942_i2c_msg_t)*size));
			kfree(msgs);
			return 0;
		}
		else
		{
			return -EINVAL;
		}
	}
	
	if(_IOC_DIR(cmd) == _IOC_WRITE)
	{
		size = _IOC_SIZE(cmd);
		if(size)
		{

			msgs = kmalloc((sizeof(act8942_i2c_msg_t)*size), GFP_KERNEL);
			if(!msgs)
			{
				pr_err("act8942_ioctl: failed to allocate memory for act8942_i2c_msgs.\n");
				return -ENOMEM;
			}

			ret = copy_from_user(msgs, (void __user *)arg, (sizeof(act8942_i2c_msg_t)*size));
			if(ret)
            {
                pr_err("act8942_ioctl: copy_from_user failed!\n ");
				kfree(msgs);
				return ret;
            }

			for(i=0; i<size; i++)
			{
				act8942_write_i2c(client, msgs[i].reg, &msgs[i].val);
			}
			kfree(msgs);
			return 0;
		}
		else
		{
			return -EINVAL;
		}
	}
    return ret;
}


static struct file_operations act8942_fops = {
    .owner   = THIS_MODULE,
    .open    = act8942_open,
    .release = act8942_release,
    .ioctl   = act8942_ioctl,
};


static int act8942_probe(struct platform_device *pdev)
{
    int ret, i;
	struct device *dev_p;
	
	pr_info("act8942_probe\n");
	act8942_pmu_dev = kmalloc(sizeof(pmu_dev_t), GFP_KERNEL);
	if (!act8942_pmu_dev)
	{
		pr_err("act8942: failed to allocate memory for pmu device\n");
		return -ENOMEM;
	}

	ret = alloc_chrdev_region(&act8942_devno, 0, 1, ACT8942_DEVICE_NAME);
	if (ret < 0) {
		pr_err("act8942: failed to allocate chrdev. \n");
		return 0;
	}

	/* connect the file operations with cdev */
	cdev_init(&act8942_pmu_dev->cdev, &act8942_fops);
	act8942_pmu_dev->cdev.owner = THIS_MODULE;

	/* connect the major/minor number to the cdev */
	ret = cdev_add(&act8942_pmu_dev->cdev, act8942_devno, 1);
	if (ret) {
		pr_err("act8942: failed to add device. \n");
		/* @todo do with error */
		return ret;
	}

	ret = class_register(&act8942_class);
	if(ret)
	{
		printk(" class register i2c_class fail!\n");
		return ret;
	}

	/* create /dev nodes */
    dev_p = device_create(&act8942_class, NULL, MKDEV(MAJOR(act8942_devno), 0),
                        NULL, "act8942");
    if (IS_ERR(dev_p)) {
        pr_err("act8942: failed to create device node\n");
        /* @todo do with error */
        return PTR_ERR(dev_p);;
    }

    printk( "act8942: driver initialized ok\n");
	
    return ret;
}

static int act8942_remove(struct platform_device *pdev)
{
	pr_info("act8942_remove\n");
    cdev_del(&act8942_pmu_dev->cdev);
    unregister_chrdev_region(act8942_devno, 1);
    kfree(act8942_pmu_dev);

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
	ret = i2c_add_driver(&act8942_i2c_driver);
	if (ret < 0)
	{
        pr_err("act8942: failed to add i2c driver. \n");
        ret = -ENOTSUPP;
    }

	return ret;
}
module_init(act8942_pmu_init);

static void __exit act8942_pmu_exit(void)
{
	pr_info("act8942_pmu_exit\n");
	i2c_del_driver(&act8942_i2c_driver);
    platform_driver_unregister(&ACT8942_platform_driver);
}
module_exit(act8942_pmu_exit);

MODULE_AUTHOR("Elvis Yu <elvis.yu@amlogic.com>");
MODULE_DESCRIPTION("ACT8942 PMU driver");
MODULE_LICENSE("GPL");


