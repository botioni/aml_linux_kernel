#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/ar1520.h>

#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
static struct early_suspend ar1520_early_suspend;
#endif

static int ar1520_probe(struct platform_device *pdev);
static int ar1520_remove(struct platform_device *pdev);
#ifdef CONFIG_HAS_EARLYSUSPEND
static int ar1520_suspend(struct early_suspend *handler);
static int ar1520_resume(struct early_suspend *handler);
#else
static int ar1520_suspend(struct i2c_client *client, pm_message_t msg);
static int ar1520_resume(struct i2c_client *client);
#endif


static struct platform_driver ar1520_driver = {
	.probe = ar1520_probe,
	.remove = ar1520_remove,
#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend = ar1520_suspend,
	.resume = ar1520_resume,
#endif	
	.driver = {
	.name = "ar1520_gps",
	.owner = THIS_MODULE,	
	},
};

static int ar1520_probe(struct platform_device *pdev)
{
	struct ar1520_platform_data *pdata = pdev->dev.platform_data;
	if (!pdata) {
		dev_err(&pdev->dev, "platform data is required!\n");
		return -EINVAL;
	}
	if(pdata->power_on)
		pdata->power_on();
	if(pdata->reset)//ar1520 need to reset at startup
		pdata->reset();
#ifdef CONFIG_HAS_EARLYSUSPEND
	ar1520_early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN;
	ar1520_early_suspend.suspend = ar1520_suspend;
	ar1520_early_suspend.resume = ar1520_resume;
	ar1520_early_suspend.param = pdata;
	register_early_suspend(&ar1520_early_suspend);
#endif
	return 0;
}

static int ar1520_remove(struct platform_device *pdev)
{
#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&ar1520_early_suspend);
#endif
	return 0;
}


#ifdef CONFIG_HAS_EARLYSUSPEND
static int ar1520_suspend(struct early_suspend *handler)
{
	int ret = -1;
	if(handler && handler->param) {
		struct ar1520_platform_data *pdata= (struct ar1520_platform_data *)handler->param;
		if(pdata->power_off){
			pdata->power_off();
			ret = 0;
		}
	}
	return ret;
}

static int ar1520_resume(struct early_suspend *handler)
{
 	int ret = -1;
	if(handler && handler->param) {
		struct ar1520_platform_data *pdata= (struct ar1520_platform_data *)handler->param;
		if(pdata->power_on){
			pdata->power_on();
			if(pdata->reset){
				pdata->reset();
				ret = 0;
			}
		}
	}
	return ret;
}
#else
static int ar1520_suspend(struct i2c_client *client, pm_message_t msg)
{
	return 0;
}

static int ar1520_resume(struct i2c_client *client)
{
	return 0;
}
#endif


static int __init init_gps(void)
{
	int ret = -1;
	ret = platform_driver_register(&ar1520_driver);
	if (ret != 0) {
		printk(KERN_ERR "failed to register ar1520 gps module, error %d\n", ret);
		return -ENODEV;
	}
	return ret;
}

late_initcall(init_gps);

static void __exit unload_gps(void)
{
    platform_driver_unregister(&ar1520_driver);
}
module_exit(unload_gps);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("AMLOGIC");
MODULE_DESCRIPTION("GPS driver for AR1520");


