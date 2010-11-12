/*
 * Meson Power Management Routines
 *
 * Copyright (C) 2010 Amlogic, Inc. http://www.amlogic.com/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

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
#include <mach/sram.h>

static void (*meson_sram_suspend) (struct meson_pm_config *);
static struct meson_pm_config *pdata;

static void meson_sram_push(void *dest, void *src, unsigned int size)
{
	memcpy(dest, src, size);
	flush_icache_range((unsigned long)dest, (unsigned long)(dest + size));
	int res = 0;
	res = memcmp(dest,src,size);
	printk("meson_sram_push (%d)", res);
}

static void meson_pm_suspend(void)
{
	int mask_save[4];
	printk(KERN_INFO "enter meson_pm_suspend!\n");

	mask_save[0] = READ_CBUS_REG(A9_0_IRQ_IN0_INTR_MASK);
	mask_save[1] = READ_CBUS_REG(A9_0_IRQ_IN1_INTR_MASK);
	mask_save[2] = READ_CBUS_REG(A9_0_IRQ_IN2_INTR_MASK);
	mask_save[3] = READ_CBUS_REG(A9_0_IRQ_IN3_INTR_MASK);
	WRITE_CBUS_REG(A9_0_IRQ_IN0_INTR_MASK, 0x0);
	WRITE_CBUS_REG(A9_0_IRQ_IN1_INTR_MASK, 0x0);
	WRITE_CBUS_REG(A9_0_IRQ_IN2_INTR_MASK, (1<<8));
	WRITE_CBUS_REG(A9_0_IRQ_IN3_INTR_MASK, 0x0);
	meson_sram_suspend(pdata);
	printk("intr stat %x %x %x %x\n", 
		READ_CBUS_REG(A9_0_IRQ_IN0_INTR_STAT), 
		READ_CBUS_REG(A9_0_IRQ_IN1_INTR_STAT),
		READ_CBUS_REG(A9_0_IRQ_IN2_INTR_STAT),
		READ_CBUS_REG(A9_0_IRQ_IN3_INTR_STAT));
	WRITE_CBUS_REG(A9_0_IRQ_IN0_INTR_MASK, mask_save[0]);
	WRITE_CBUS_REG(A9_0_IRQ_IN1_INTR_MASK, mask_save[1]);
	WRITE_CBUS_REG(A9_0_IRQ_IN2_INTR_MASK, mask_save[2]);
	WRITE_CBUS_REG(A9_0_IRQ_IN3_INTR_MASK, mask_save[3]);

}

static int meson_pm_prepare(void)
{
	printk(KERN_INFO "enter meson_pm_prepare!\n");
	meson_sram_push(meson_sram_suspend, meson_cpu_suspend,
						meson_cpu_suspend_sz);
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

	meson_sram_suspend = sram_alloc(meson_cpu_suspend_sz);
	if (!meson_sram_suspend) {
		dev_err(&pdev->dev, "cannot allocate SRAM memory\n");
		return -ENOMEM;
	}

	meson_sram_push(meson_sram_suspend, meson_cpu_suspend,
						meson_cpu_suspend_sz);

	suspend_set_ops(&meson_pm_ops);
	printk(KERN_INFO "meson_pm_probe done 0x%x %d!\n", meson_sram_suspend, meson_cpu_suspend_sz);
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
