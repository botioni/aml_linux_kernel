#include "pr_dbg.h"
#include <asm/uaccess.h>
#include <linux/kernel.h>
#include <linux/module.h>
type_printk	pr_dbg;
type_printk	pr_err;
EXPORT_SYMBOL(pr_dbg);
EXPORT_SYMBOL(pr_err);
int  mute_printk(const char *fmt, ...)
{
	return 0;
}
EXPORT_SYMBOL(mute_printk);
int  normal_printk(const char *fmt, ...)
{
	va_list args;
	int r;
	va_start(args, fmt);
	r = vprintk(fmt, args);
	va_end(args);
	return r;
}
EXPORT_SYMBOL(normal_printk);
