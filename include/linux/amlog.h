#ifndef __AMLOG_H
#define __AMLOG_H

#if (AMLOG > 0)
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>

#ifndef LOG_DEFAULT_LEVEL
#define LOG_DEFAULT_LEVEL 0
#endif
#ifndef LOG_LEVEL_DESC
#define LOG_LEVEL_DESC "log_level."
#endif
static u32 amlog_level = LOG_DEFAULT_LEVEL;
module_param(amlog_level, uint, S_IRUSR);
MODULE_PARM_DESC(amlog_level, LOG_LEVEL_DESC);

#ifndef LOG_DEFAULT_MASK
#define LOG_DEFAULT_MASK 0xffffffffUL
#endif
#ifndef LOG_MASK_DESC
#define LOG_MASK_DESC "log_mask."
#endif
static u32 amlog_mask = LOG_DEFAULT_MASK;
module_param(amlog_mask, uint, S_IRUSR);
MODULE_PARM_DESC(amlog_mask, LOG_MASK_DESC);

#define amlog(x...) printk(x)

#define amlog_level(level, x...) \
	do { \
		if (level >= amlog_level) \
			printk(x); \
	} while (0);

#define amlog_mask(mask, x...) \
	do { \
		if (mask & amlog_mask) \
			printk(x); \
	} while (0);

#define amlog_mask_level(mask, level, x...) \
	do { \
		if ((level >= amlog_level) && (mask & amlog_mask)) \
			printk(x); \
	} while (0);

#define amlog_if(cond, x...) do {if (cond) printk(x);} while {0};

#define amlog_level_if(cond, level, x...) \
	do { \
		if ((cond) && (level >= amlog_level)) \
			printk(x); \
	} while (0);

#define amlog_mask_if(cond, mask, x...) \
	do { \
		if ((cond) && (mask & amlog_mask)) \
			printk(x); \
	} while (0);

#define amlog_mask_levelif(cond, mask, level, x...) \
	do { \
		if ((cond) && (level >= amlog_level) && (mask & amlog_mask)) \
			printk(x...); \
	} while (0);

#else
#define amlog(x...)
#define amlog_level(level, x...)
#define amlog_mask(mask, x...)
#define amlog_mask_level(mask, level, x...)
#define amlog_if(cond, x...)
#define amlog_level_if(cond, level, x...)
#define amlog_mask_if(cond, mask, x...)
#define amlog_mask_level_if(cond, mask, level, x...)
#endif 

#endif /* __AMLOG_H */
