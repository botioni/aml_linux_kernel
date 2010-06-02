/*
 * AMLOGIC LED controller driver.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the named License,
 * or any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307, USA
 *
 * Author:  Tim Yao <timyao@amlogic.com>
 *
 */

#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/spinlock.h>
#include <linux/platform_device.h>

#include <asm/arch/gpio.h>

#define LED0_PIN_NO 12
#define LED1_PIN_NO 11
#define LED2_PIN_NO 8

enum {
    LED0 = 0,
    LED1 = 1,
    LED2 = 2,
    LED_MAX = 3,
};

static int onoff[LED_MAX];
static spinlock_t lock = SPIN_LOCK_UNLOCKED;

#define DECLARE_LED_FUNC_SET(id, grp) \
    static void led##id##_set(void)  \
    { \
        ulong flags; \
     \
        spin_lock_irqsave(&lock, flags); \
     \
        GPIO##grp##_MODE(LED##id##_PIN_NO, GPIO_OUT_MODE); \
     \
        if (onoff[LED##id]) \
            GPIO##grp##_SET_PIN(LED##id##_PIN_NO, PIN_DOWN); \
        else \
            GPIO##grp##_SET_PIN(LED##id##_PIN_NO, PIN_UP); \
     \
        spin_unlock_irqrestore(&lock, flags); \
    }

#define DECLARE_LED_FUNC_SHOW(id) \
    static ssize_t led##id##_show(struct class *cla, char *buf)  \
    { \
        if (onoff[id]) \
            return sprintf(buf, "on\n"); \
        else \
            return sprintf(buf, "off\n"); \
    }

#define DECLARE_LED_FUNC_STORE(id) \
    static ssize_t led##id##_store(struct class *cla, const char *buf, \
                                size_t count)   \
    { \
        size_t r; \
    \
        r = sscanf(buf, "%d", &onoff[id]); \
        if (r != 1) {\
            return -EINVAL; \
        } \
    \
        led##id##_set(); \
        return count; \
    }

#define DECLARE_LED_FUNC(id, grp) \
    DECLARE_LED_FUNC_SET(id, grp) \
    DECLARE_LED_FUNC_SHOW(id) \
    DECLARE_LED_FUNC_STORE(id)

DECLARE_LED_FUNC(0, B)
DECLARE_LED_FUNC(1, B)
DECLARE_LED_FUNC(2, _CARD)

static struct class_attribute led_class_attrs[] = {
    __ATTR(led0,
           S_IRUGO | S_IWUSR,
           led0_show,
           led0_store),
    __ATTR(led1,
           S_IRUGO | S_IWUSR,
           led1_show,
           led1_store),
    __ATTR(led2,
           S_IRUGO | S_IWUSR,
           led2_show,
           led2_store),
    __ATTR_NULL
};

static struct class led_class = {
    .name = "led",
    .class_attrs = led_class_attrs,
};

/*********************************************************/
static int __init led_init(void)
{
    int r;

    r = class_register(&led_class);

    if (r) {
        printk("create led class fail\n");

        return r;
    }

    return (0);
}

static void __exit led_exit(void)
{
    class_unregister(&led_class);
}

module_init(led_init);
module_exit(led_exit);

MODULE_DESCRIPTION("AMLOGIC LED controller driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Tim Yao <timyao@amlogic.com>");

