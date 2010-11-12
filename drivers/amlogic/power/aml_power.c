/*
 * Common power driver for Amlogic Devices with one or two external
 * power supplies (AC/USB) connected to main and backup batteries,
 * and optional builtin charger.
 *
 * Copyright © 2010 Larson Jiang <larson.jiang@amlogic.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/power_supply.h>
#include <linux/aml_power.h>
#include <linux/regulator/consumer.h>
#include <linux/timer.h>
#include <linux/jiffies.h>
#include <linux/usb/otg.h>

static inline unsigned int get_irq_flags(struct resource *res)
{
	unsigned int flags = IRQF_SAMPLE_RANDOM | IRQF_SHARED;

	flags |= res->flags & IRQF_TRIGGER_MASK;

	return flags;
}

static struct device *dev;
static struct aml_power_pdata *pdata;
static struct resource *ac_irq, *usb_irq;
static struct timer_list charger_timer;
static struct timer_list supply_timer;
static struct timer_list polling_timer;
static int polling;

#ifdef CONFIG_USB_OTG_UTILS
static struct otg_transceiver *transceiver;
#endif
static struct regulator *ac_draw;

enum {
	AML_PSY_OFFLINE = 0,
	AML_PSY_ONLINE = 1,
	AML_PSY_TO_CHANGE,
};
static int new_ac_status = -1;
static int new_usb_status = -1;
static int ac_status = -1;
static int usb_status = -1;
static int battery_capacity = -1;
static int new_battery_capacity = -1;

static int aml_power_get_property(struct power_supply *psy,
				  enum power_supply_property psp,
				  union power_supply_propval *val)
{
    int capacty;
	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		printk("get POWER_SUPPLY_TYPE_MAINS\n");
		if (psy->type == POWER_SUPPLY_TYPE_MAINS)
			val->intval = pdata->is_ac_online ?
				      pdata->is_ac_online() : 0;
		else
			val->intval = pdata->is_usb_online ?
				      pdata->is_usb_online() : 0;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = pdata->get_bat_vol()*(3300*2/1024);
		printk("curren voltage is :%dmV\n",val->intval);
		break;
	case POWER_SUPPLY_PROP_STATUS:
		printk("get POWER_SUPPLY_PROP_STATUS\n");
		if(pdata->is_ac_online())
		{
			if(pdata->get_charge_status())
				val->intval = POWER_SUPPLY_STATUS_FULL;
			else
				val->intval = POWER_SUPPLY_STATUS_CHARGING;
		}
		else
			val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:	
	    capacty = 100*(pdata->get_bat_vol() - 540)/80;
		val->intval = capacty>100? 100:capacty;
		printk("current capacity is %d%%\n,",val->intval);
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static enum power_supply_property aml_power_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static enum power_supply_property aml_battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
};

static char *aml_power_supplied_to[] = {
	"main-battery",
	"backup-battery",
};

static struct power_supply aml_psy_ac = {
	.name = "ac",
	.type = POWER_SUPPLY_TYPE_MAINS,
	.supplied_to = aml_power_supplied_to,
	.num_supplicants = ARRAY_SIZE(aml_power_supplied_to),
	.properties = aml_power_props,
	.num_properties = ARRAY_SIZE(aml_power_props),
	.get_property = aml_power_get_property,
};

static struct power_supply aml_psy_usb = {
	.name = "usb",
	.type = POWER_SUPPLY_TYPE_USB,
	.supplied_to = aml_power_supplied_to,
	.num_supplicants = ARRAY_SIZE(aml_power_supplied_to),
	.properties = aml_power_props,
	.num_properties = ARRAY_SIZE(aml_power_props),
	.get_property = aml_power_get_property,
};

static struct power_supply aml_psy_bat = {
	.name = "battery",
	.type = POWER_SUPPLY_TYPE_BATTERY,
	.properties = aml_battery_props,
	.num_properties = ARRAY_SIZE(aml_battery_props),
	.get_property = aml_power_get_property,
};

static void update_status(void)
{
	if (pdata->is_ac_online)
		new_ac_status = !!pdata->is_ac_online();

	if (pdata->is_usb_online)
		new_usb_status = !!pdata->is_usb_online();
}

static void update_charger(void)
{
	static int regulator_enabled;
	int max_uA = pdata->ac_max_uA;

	if (pdata->set_charge) {
		if (new_ac_status > 0) {
			dev_dbg(dev, "charger on (AC)\n");
			pdata->set_charge(AML_POWER_CHARGE_AC);
		} else if (new_usb_status > 0) {
			dev_dbg(dev, "charger on (USB)\n");
			pdata->set_charge(AML_POWER_CHARGE_USB);
		} else {
			dev_dbg(dev, "charger off\n");
			pdata->set_charge(0);
		}
	} else if (ac_draw) {
		if (new_ac_status > 0) {
			regulator_set_current_limit(ac_draw, max_uA, max_uA);
			if (!regulator_enabled) {
				dev_dbg(dev, "charger on (AC)\n");
				regulator_enable(ac_draw);
				regulator_enabled = 1;
			}
		} else {
			if (regulator_enabled) {
				dev_dbg(dev, "charger off\n");
				regulator_disable(ac_draw);
				regulator_enabled = 0;
			}
		}
	}
}

static void supply_timer_func(unsigned long unused)
{
	if (ac_status == AML_PSY_TO_CHANGE) {
		ac_status = new_ac_status;
		power_supply_changed(&aml_psy_ac);
	}

	if (usb_status == AML_PSY_TO_CHANGE) {
		usb_status = new_usb_status;
		power_supply_changed(&aml_psy_usb);
	}
}

static void psy_changed(void)
{
	update_charger();

	/*
	 * Okay, charger set. Now wait a bit before notifying supplicants,
	 * charge power should stabilize.
	 */
	mod_timer(&supply_timer,
		  jiffies + msecs_to_jiffies(pdata->wait_for_charger));
}

static void charger_timer_func(unsigned long unused)
{
	update_status();
	psy_changed();
}

static irqreturn_t power_changed_isr(int irq, void *power_supply)
{
	if (power_supply == &aml_psy_ac)
		ac_status = AML_PSY_TO_CHANGE;
	else if (power_supply == &aml_psy_usb)
		usb_status = AML_PSY_TO_CHANGE;
	else
		return IRQ_NONE;

	/*
	 * Wait a bit before reading ac/usb line status and setting charger,
	 * because ac/usb status readings may lag from irq.
	 */
	mod_timer(&charger_timer,
		  jiffies + msecs_to_jiffies(pdata->wait_for_status));

	return IRQ_HANDLED;
}

static void polling_timer_func(unsigned long unused)
{
	int changed = 0;

	dev_dbg(dev, "polling...\n");

	update_status();

	if (!ac_irq && new_ac_status != ac_status) {
		ac_status = AML_PSY_TO_CHANGE;
		changed = 1;
	}

	if (!usb_irq && new_usb_status != usb_status) {
		usb_status = AML_PSY_TO_CHANGE;
		changed = 1;
	}

	if (changed)
		psy_changed();

    new_battery_capacity = pdata->get_bat_vol();
	if(new_battery_capacity != battery_capacity){
		battery_capacity = new_battery_capacity;
		power_supply_changed(&aml_psy_bat);
    }	
    
	mod_timer(&polling_timer,
		  jiffies + msecs_to_jiffies(pdata->polling_interval));
}

#ifdef CONFIG_USB_OTG_UTILS
static int otg_is_usb_online(void)
{
	return (transceiver->state == OTG_STATE_B_PERIPHERAL);
}
#endif

static int aml_power_probe(struct platform_device *pdev)
{
	int ret = 0;

	dev = &pdev->dev;

	if (pdev->id != -1) {
		dev_err(dev, "it's meaningless to register several "
			"pda_powers; use id = -1\n");
		ret = -EINVAL;
		goto wrongid;
	}

	pdata = pdev->dev.platform_data;

	if (pdata->init) {
		ret = pdata->init(dev);
		if (ret < 0)
			goto init_failed;
	}

	update_status();
	update_charger();

	if (!pdata->wait_for_status)
		pdata->wait_for_status = 500;

	if (!pdata->wait_for_charger)
		pdata->wait_for_charger = 500;

	if (!pdata->polling_interval)
		pdata->polling_interval = 2000;

	if (!pdata->ac_max_uA)
		pdata->ac_max_uA = 500000;

	setup_timer(&charger_timer, charger_timer_func, 0);
	setup_timer(&supply_timer, supply_timer_func, 0);

	ac_irq = platform_get_resource_byname(pdev, IORESOURCE_IRQ, "ac");
	usb_irq = platform_get_resource_byname(pdev, IORESOURCE_IRQ, "usb");

	if (pdata->supplied_to) {
		aml_psy_ac.supplied_to = pdata->supplied_to;
		aml_psy_ac.num_supplicants = pdata->num_supplicants;
		aml_psy_usb.supplied_to = pdata->supplied_to;
		aml_psy_usb.num_supplicants = pdata->num_supplicants;
	}

	ac_draw = regulator_get(dev, "ac_draw");
	if (IS_ERR(ac_draw)) {
		dev_dbg(dev, "couldn't get ac_draw regulator\n");
		ac_draw = NULL;
		ret = PTR_ERR(ac_draw);
	}

	if (pdata->is_ac_online) {
		ret = power_supply_register(&pdev->dev, &aml_psy_ac);
		if (ret) {
			dev_err(dev, "failed to register %s power supply\n",
				aml_psy_ac.name);
			goto ac_supply_failed;
		}

		if (ac_irq) {
			ret = request_irq(ac_irq->start, power_changed_isr,
					  get_irq_flags(ac_irq), ac_irq->name,
					  &aml_psy_ac);
			if (ret) {
				dev_err(dev, "request ac irq failed\n");
				goto ac_irq_failed;
			}
		} else {
			polling = 1;
		}
	}

#ifdef CONFIG_USB_OTG_UTILS
	transceiver = otg_get_transceiver();
	if (transceiver && !pdata->is_usb_online) {
		pdata->is_usb_online = otg_is_usb_online;
	}
#endif

	if (pdata->is_usb_online) {
		ret = power_supply_register(&pdev->dev, &aml_psy_usb);
		if (ret) {
			dev_err(dev, "failed to register %s power supply\n",
				aml_psy_usb.name);
			goto usb_supply_failed;
		}

		if (usb_irq) {
			ret = request_irq(usb_irq->start, power_changed_isr,
					  get_irq_flags(usb_irq),
					  usb_irq->name, &aml_psy_usb);
			if (ret) {
				dev_err(dev, "request usb irq failed\n");
				goto usb_irq_failed;
			}
		} else {
			polling = 1;
		}
	}

	ret = power_supply_register(&pdev->dev, &aml_psy_bat);
	if (ret) {
		dev_err(dev, "failed to register battery\n");
		goto bat_supply_failed;
	}
	
	if (polling) {
		dev_dbg(dev, "will poll for status\n");
		setup_timer(&polling_timer, polling_timer_func, 0);
		mod_timer(&polling_timer,
			  jiffies + msecs_to_jiffies(pdata->polling_interval));
	}

	if (ac_irq || usb_irq)
		device_init_wakeup(&pdev->dev, 1);

	return 0;
bat_supply_failed:
	power_supply_unregister(&aml_psy_bat);
usb_irq_failed:
	if (pdata->is_usb_online)
		power_supply_unregister(&aml_psy_usb);
usb_supply_failed:
	if (pdata->is_ac_online && ac_irq)
		free_irq(ac_irq->start, &aml_psy_ac);
#ifdef CONFIG_USB_OTG_UTILS
	if (transceiver)
		otg_put_transceiver(transceiver);
#endif
ac_irq_failed:
	if (pdata->is_ac_online)
		power_supply_unregister(&aml_psy_ac);
ac_supply_failed:
	if (ac_draw) {
		regulator_put(ac_draw);
		ac_draw = NULL;
	}
	if (pdata->exit)
		pdata->exit(dev);
init_failed:
wrongid:
	return ret;
}

static int aml_power_remove(struct platform_device *pdev)
{
	if (pdata->is_usb_online && usb_irq)
		free_irq(usb_irq->start, &aml_psy_usb);
	if (pdata->is_ac_online && ac_irq)
		free_irq(ac_irq->start, &aml_psy_ac);

	if (polling)
		del_timer_sync(&polling_timer);
	del_timer_sync(&charger_timer);
	del_timer_sync(&supply_timer);
	
	power_supply_unregister(&aml_psy_bat);
	if (pdata->is_usb_online)
		power_supply_unregister(&aml_psy_usb);
	if (pdata->is_ac_online)
		power_supply_unregister(&aml_psy_ac);
	
#ifdef CONFIG_USB_OTG_UTILS
	if (transceiver)
		otg_put_transceiver(transceiver);
#endif
	if (ac_draw) {
		regulator_put(ac_draw);
		ac_draw = NULL;
	}
	if (pdata->exit)
		pdata->exit(dev);

	return 0;
}

#ifdef CONFIG_PM
static int ac_wakeup_enabled;
static int usb_wakeup_enabled;

static int aml_power_suspend(struct platform_device *pdev, pm_message_t state)
{
	if (device_may_wakeup(&pdev->dev)) {
		if (ac_irq)
			ac_wakeup_enabled = !enable_irq_wake(ac_irq->start);
		if (usb_irq)
			usb_wakeup_enabled = !enable_irq_wake(usb_irq->start);
	}

	return 0;
}

static int aml_power_resume(struct platform_device *pdev)
{
	if (device_may_wakeup(&pdev->dev)) {
		if (usb_irq && usb_wakeup_enabled)
			disable_irq_wake(usb_irq->start);
		if (ac_irq && ac_wakeup_enabled)
			disable_irq_wake(ac_irq->start);
	}

	return 0;
}
#else
#define aml_power_suspend NULL
#define aml_power_resume NULL
#endif /* CONFIG_PM */

MODULE_ALIAS("platform:aml-power");

static struct platform_driver aml_power_pdrv = {
	.driver = {
		.name = "aml-power",
	},
	.probe = aml_power_probe,
	.remove = aml_power_remove,
	.suspend = aml_power_suspend,
	.resume = aml_power_resume,
};

static int __init aml_power_init(void)
{
	printk("amlogic power supply init\n");
	return platform_driver_register(&aml_power_pdrv);
}

static void __exit aml_power_exit(void)
{
	platform_driver_unregister(&aml_power_pdrv);
}

module_init(aml_power_init);
module_exit(aml_power_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Larson Jiang");
