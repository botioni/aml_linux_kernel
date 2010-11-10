#include <linux/pm.h>
#include <linux/suspend.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/spinlock.h>

#include <asm/cacheflush.h>
#include <asm/delay.h>

#include <mach/pm.h>
#include <mach/am_regs.h>



static struct meson_pm_config *pdata;

static void meson_pm_suspend(void)
{
	printk(KERN_INFO "enter meson_pm_suspend!\n");

	WRITE_CBUS_REG(0x21d0/*RTC_ADDR0*/, (READ_CBUS_REG(0x21d0/*RTC_ADDR0*/) &~(1<<11)));
    WRITE_CBUS_REG(0x21d1/*RTC_ADDR0*/, (READ_CBUS_REG(0x21d1/*RTC_ADDR0*/) &~(1<<3)));
	int powerPress = 0;
	while(1){
		udelay(jiffies+msecs_to_jiffies(20));
		powerPress = ((READ_CBUS_REG(0x21d1/*RTC_ADDR1*/) >> 2) & 1) ? 0 : 1;
		if(powerPress)
			break;
	}
}

static int meson_pm_prepare(void)
{
	printk(KERN_INFO "enter meson_pm_prepare!\n");
	return 0;
}

static int meson_pm_enter(suspend_state_t state)
{
	int ret = 0;

	switch (state) {
	case PM_SUSPEND_STANDBY:
	case PM_SUSPEND_MEM:
		meson_pm_suspend();
		break;
	default:
		ret = -EINVAL;
	}

	return ret;
}

static struct platform_suspend_ops meson_pm_ops = {
	.enter		= meson_pm_enter,
	.prepare    = meson_pm_prepare,
	.valid		= suspend_valid_only_mem,
};

static int __init meson_pm_probe(struct platform_device *pdev)
{
	printk(KERN_INFO "enter meson_pm_probe!\n");
	
	pdata = pdev->dev.platform_data;
	if (!pdata) {
		dev_err(&pdev->dev, "cannot get platform data\n");
		return -ENOENT;
	}
	
	suspend_set_ops(&meson_pm_ops);

	return 0;
}

static int __exit meson_pm_remove(struct platform_device *pdev)
{
	return 0;
}

static struct platform_driver meson_pm_driver = {
	.driver = {
		.name	 = "pm-meson",
		.owner	 = THIS_MODULE,
	},
	.remove = __exit_p(meson_pm_remove),
};

static int __init meson_pm_init(void)
{
	return platform_driver_probe(&meson_pm_driver, meson_pm_probe);
}
late_initcall(meson_pm_init);