/*
 *
 * arch/arm/mach-meson/bcm-bt.c
 *
 *  Copyright (C) 2010 AMLOGIC, INC.
 *
 * License terms: GNU General Public License (GPL) version 2
 * Platform machine definition.
 */
 
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/string.h>
#include <linux/ctype.h>
#include <linux/leds.h>
#include <linux/gpio.h>
#include <linux/rfkill.h>


static int bcm_bt_set_block(void *data, bool blocked)
{
    return 0;
}

static const struct rfkill_ops bcm_bt_rfkill_ops = {
	.set_block = bcm_bt_set_block,
};

static int __init bcm_bt_probe(struct platform_device *pdev)
{
	int rc = 0;
	struct rfkill *rfk;

	rfk = rfkill_alloc("bcm-bt", &pdev->dev, RFKILL_TYPE_BLUETOOTH,
			&bcm_bt_rfkill_ops, NULL);
						   
	if (!rfk) {
        printk("rfk alloc fail\n");
		rc = -ENOMEM;
		goto err_rfk_alloc;
	}
	
	rc = rfkill_register(rfk);
	if (rc){
        printk("rfkill_register fail\n");
		goto err_rfkill;
    }
	platform_set_drvdata(pdev, rfk);

	return 0;	
	
err_rfkill:
	rfkill_destroy(rfk);
err_rfk_alloc:
	return rc;
	
}

static int bcm_bt_remove(struct platform_device *pdev)
{
	struct rfkill *rfk = platform_get_drvdata(pdev);

	platform_set_drvdata(pdev, NULL);

	if (rfk) {
		rfkill_unregister(rfk);
		rfkill_destroy(rfk);
	}
	rfk = NULL;

	return 0;
}
static struct platform_driver bcm_bt_driver = {
	.driver		= {
		.name	= "bcm-bt",
	},
	.probe		= bcm_bt_probe,
	.remove		= bcm_bt_remove,
};

static int __init bcm_bt_init(void)
{
    printk("amlogic rfkill init\n");
	return platform_driver_register(&bcm_bt_driver);
}
static void __exit bcm_bt_exit(void)
{
	platform_driver_unregister(&bcm_bt_driver);
}
module_init(bcm_bt_init);
module_exit(bcm_bt_exit);
MODULE_DESCRIPTION("bcm bt rfkill");
MODULE_AUTHOR("");
MODULE_LICENSE("GPL");
