#include <linux/module.h>
#include <linux/spinlock.h>
#include <linux/kernel.h>
#include <linux/device.h>

#include <linux/amlog.h>
MODULE_AMLOG(AMLOG_DEFAULT_LEVEL, 0, LOG_DEFAULT_LEVEL_DESC, LOG_DEFAULT_MASK_DESC);

static int subtitle_enable = 1;
static int subtitle_total = 0;
static int subtitle_width = 0;
static int subtitle_height = 0;
static int subtitle_type = -1;
static int subtitle_current = 0; // no subtitle


// total
// curr
// bimap
// text
// type
// info
// pts
// duration
// color pallete
// width/height

static ssize_t show_curr(struct class *class,
                           struct class_attribute *attr,
                           char *buf)
{
    return sprintf(buf, "%d: current\n", subtitle_current);
}

static ssize_t store_curr(struct class *class,
                            struct class_attribute *attr,
                            const char *buf,
                            size_t size)
{
    unsigned curr;
    ssize_t r;

    r = sscanf(buf, "%d", &curr);
    if ((r != 1))
        return -EINVAL;

    subtitle_current = curr;

    return size;
}


static ssize_t show_type(struct class *class,
                           struct class_attribute *attr,
                           char *buf)
{
    return sprintf(buf, "%d: type\n", subtitle_type);
}

static ssize_t store_type(struct class *class,
                            struct class_attribute *attr,
                            const char *buf,
                            size_t size)
{
    unsigned type;
    ssize_t r;

    r = sscanf(buf, "%d", &type);
    if ((r != 1))
        return -EINVAL;

    subtitle_type = type;

    return size;
}

static ssize_t show_width(struct class *class,
                           struct class_attribute *attr,
                           char *buf)
{
    return sprintf(buf, "%d: width\n", subtitle_width);
}

static ssize_t store_width(struct class *class,
                            struct class_attribute *attr,
                            const char *buf,
                            size_t size)
{
    unsigned width;
    ssize_t r;

    r = sscanf(buf, "%d", &width);
    if ((r != 1))
        return -EINVAL;

    subtitle_width = width;

    return size;
}

static ssize_t show_height(struct class *class,
                           struct class_attribute *attr,
                           char *buf)
{
    return sprintf(buf, "%d: height\n", subtitle_height);
}

static ssize_t store_height(struct class *class,
                            struct class_attribute *attr,
                            const char *buf,
                            size_t size)
{
    unsigned height;
    ssize_t r;

    r = sscanf(buf, "%d", &height);
    if ((r != 1))
        return -EINVAL;

    subtitle_height = height;

    return size;
}

static ssize_t show_total(struct class *class,
                           struct class_attribute *attr,
                           char *buf)
{
    return sprintf(buf, "%d: disabled\n", subtitle_total);
}

static ssize_t store_total(struct class *class,
                            struct class_attribute *attr,
                            const char *buf,
                            size_t size)
{
    unsigned total;
    ssize_t r;

    r = sscanf(buf, "%d", &total);
    if ((r != 1))
        return -EINVAL;

    subtitle_total = total;

    return size;
}

static ssize_t show_enable(struct class *class,
                           struct class_attribute *attr,
                           char *buf)
{
    if (subtitle_enable)
        return sprintf(buf, "1: enabled\n");

    return sprintf(buf, "0: disabled\n");
}

static ssize_t store_enable(struct class *class,
                            struct class_attribute *attr,
                            const char *buf,
                            size_t size)
{
    unsigned mode;
    ssize_t r;

    r = sscanf(buf, "%d", &mode);
    if ((r != 1))
        return -EINVAL;

    subtitle_enable = mode ? 1 : 0;

    return size;
}


static struct class_attribute subtitle_class_attrs[] = {
    __ATTR(enable,     S_IRUGO | S_IWUSR, show_enable,  store_enable),
    __ATTR(total,     S_IRUGO | S_IWUSR, show_total,  store_total),
    __ATTR(width,     S_IRUGO | S_IWUSR, show_width,  store_width),
    __ATTR(height,     S_IRUGO | S_IWUSR, show_height,  store_height),
    __ATTR(type,     S_IRUGO | S_IWUSR, show_type,  store_type),
    __ATTR(curr,     S_IRUGO | S_IWUSR, show_curr,  store_curr),
    __ATTR_NULL
};

static struct class subtitle_class = {
    .name = "subtitle",
    .class_attrs = subtitle_class_attrs,
};

static int __init subtitle_init(void)
{
    int r;

    r = class_register(&subtitle_class);

    if (r) {
        amlog_level(LOG_LEVEL_ERROR, "subtitle class create fail.\n");
        return r;
    }
		
		
    return (0);
}

static void __exit subtitle_exit(void)
{
    class_unregister(&subtitle_class);
}

module_init(subtitle_init);
module_exit(subtitle_exit);

MODULE_DESCRIPTION("AMLOGIC Subtitle management driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Kevin Wang <kevin.wang@amlogic.com>");