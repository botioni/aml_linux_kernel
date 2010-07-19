/*
 * AMLOGIC Audio/Video streaming port driver.
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

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/platform_device.h>
#include <linux/amports/vformat.h>

#define MC_SIZE (4096 * 4)

#define SUPPORT_VCODEC_NUM  1
static int inited_vcodec_num = 0;

static struct platform_device *vdec_device = NULL;

static struct resource amvdec_mem_resource[]  = {
    [0] = {
        .start = 0,
        .end   = 0,
        .flags = 0,
    },
    [1] = {
        .start = 0,
        .end   = 0,
        .flags = 0,
    }
};

static const char *vdec_device_name[] = {
    "amvdec_mpeg12",
    "amvdec_mpeg4",
    "amvdec_h264",
    "amvdec_mjpeg",
    "amvdec_real",
    "amjpegdec",
    "amvdec_vc1"
};

/*
This function used for change the memory reasource on system run;
It can used for system swap memory for codec and some othor use;
We must call it at the amstream start for register a memory resource
*/
int vdec_set_resource(struct resource *s, void *param)
{
    if (inited_vcodec_num != 0) {
		printk("ERROR:We can't support the change resource at code running\n");
        return -1;
    }

    amvdec_mem_resource[0].start = s->start;
    amvdec_mem_resource[0].end = s->end;
    amvdec_mem_resource[0].flags = s->flags;

    amvdec_mem_resource[1].start = (resource_size_t)param;
    
    return 0;
}

s32 vdec_init(vformat_t vf)
{
    s32 r;

    if (inited_vcodec_num >= SUPPORT_VCODEC_NUM) {
        printk("We only support the one video code at each time\n");
        return -EIO;
    }

    inited_vcodec_num++;

    if (amvdec_mem_resource[0].flags != IORESOURCE_MEM) {
        printk("no memory resouce for codec,Maybe have not set it\n");
        inited_vcodec_num--;
        return -ENOMEM;
    }

    vdec_device = platform_device_alloc(vdec_device_name[vf], -1);

    if (!vdec_device) {
        printk("vdec: Device allocation failed\n");
        r = -ENOMEM;
        goto error;
    }

    r = platform_device_add_resources(vdec_device, amvdec_mem_resource,
                                      ARRAY_SIZE(amvdec_mem_resource));

    if (r) {
        printk("vdec: Device resource addition failed (%d)\n", r);
        goto error;
    }

    r = platform_device_add(vdec_device);

    if (r) {
        printk("vdec: Device addition failed (%d)\n", r);
        goto error;
    }

    return 0;

error:
    if (vdec_device) {
        platform_device_put(vdec_device);
        vdec_device = NULL;
    }

    inited_vcodec_num--;

    return r;
}

s32 vdec_release(vformat_t vf)
{
    if (vdec_device)
        platform_device_unregister(vdec_device);

    inited_vcodec_num--;

    vdec_device = NULL;

    return 0;
}
