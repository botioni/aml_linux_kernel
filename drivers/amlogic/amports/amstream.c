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
#include <linux/module.h>
#include <linux/types.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/vmalloc.h>
#include <linux/mm.h>
#include <linux/major.h>
#include <linux/sched.h>
#include <linux/amports/amstream.h>
#include <linux/amports/vformat.h>
#include <linux/amports/aformat.h>
#include <linux/amports/tsync.h>
#include <linux/amports/ptsserv.h>
#include <linux/amports/timestamp.h>

#include <asm/types.h>
#include <asm/uaccess.h>
#include <asm/sections.h>
#include <asm/io.h>
#include <mach/am_regs.h>

#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/poll.h>
#include <linux/dma-mapping.h>
#include <asm/uaccess.h>

#include "streambuf.h"
#include "streambuf_reg.h"
#include "tsdemux.h"
#include "psparser.h"
#include "esparser.h"
#include "vdec.h"
#include "adec.h"
#include "rmparser.h"

#define DEVICE_NAME "amstream-dev"
#define DRIVER_NAME "amstream"
#define MODULE_NAME "amstream"

#define MAX_PORT_NUM ARRAY_SIZE(ports)

extern void set_real_audio_info(void *arg);
//#define DATA_DEBUG

#ifdef DATA_DEBUG
#include <linux/fs.h>
#define DEBUG_FILE_NAME     "/tmp/debug.tmp"
static struct file* debug_filp = NULL;
static loff_t debug_file_pos = 0;

void debug_file_write(const char __user *buf, size_t count)
{
    mm_segment_t old_fs;

    if (!debug_filp)
    {
        return;
    }

    old_fs = get_fs();
    set_fs(KERNEL_DS);

    if (count != vfs_write(debug_filp, buf, count, &debug_file_pos))
    {
        printk("Failed to write debug file\n");
    }

    set_fs(old_fs);

    return;
}
#endif

#define DEFAULT_VIDEO_BUFFER_SIZE       (1024*1024*3)
#define DEFAULT_AUDIO_BUFFER_SIZE       (1024*384)
#define DEFAULT_SUBTITLE_BUFFER_SIZE     (1024*128)
#if 0
static ulong vbuf_start;
module_param(vbuf_start, ulong, 0644);
MODULE_PARM_DESC(vbuf_start, "Amstreaming ports video buffer start address");

static ulong vbuf_size;
module_param(vbuf_size, ulong, 0644);
MODULE_PARM_DESC(vbuf_size, "Amstreaming ports video buffer size");

static ulong abuf_start;
module_param(abuf_start, ulong, 0644);
MODULE_PARM_DESC(abuf_start, "Amstreaming ports audio buffer start address");

static ulong abuf_size;
module_param(abuf_size, ulong, 0644);
MODULE_PARM_DESC(abuf_size, "Amstreaming ports audio buffer size");
#endif
#if 0
typedef struct stream_port_s {
    /* driver info */
    const char *name;
    struct device *class_dev;
    const struct file_operations *fops;

    /* ports control */
    s32 type;
    s32 flag;

    /* decoder info */
    s32 vformat;
    s32 aformat;
    s32 achanl;
    s32 asamprate;

    /* parser info */
    u32 vid;
    u32 aid;
} stream_port_t;
#endif
static int amstream_open
    (struct inode *inode, struct file *file);
static int amstream_release
    (struct inode *inode, struct file *file);
static int amstream_ioctl
    (struct inode *inode, struct file *file,
     unsigned int cmd, ulong arg);
static ssize_t amstream_vbuf_write
    (struct file *file, const char *buf,
     size_t count, loff_t * ppos);
static ssize_t amstream_abuf_write
    (struct file *file, const char *buf,
     size_t count, loff_t * ppos);
static ssize_t amstream_mpts_write
    (struct file *file, const char *buf,
     size_t count, loff_t * ppos);
static ssize_t amstream_mpps_write
    (struct file *file, const char *buf,
     size_t count, loff_t * ppos);
static ssize_t amstream_sub_read
    (struct file *file, char *buf,
     size_t count, loff_t * ppos);
static unsigned int amstream_sub_poll
    (struct file *file, poll_table *wait_table);
static int (*amstream_vdec_status)
    (struct vdec_status *vstatus);
static int (*amstream_adec_status)
    (struct adec_status *astatus);
static int (*amstream_vdec_trickmode)
    (unsigned long trickmode);
static ssize_t amstream_mprm_write
    (struct file *file, const char *buf,
     size_t count, loff_t * ppos);

const static struct file_operations vbuf_fops = {
    .owner    = THIS_MODULE,
    .open     = amstream_open,
    .release  = amstream_release,
    .write    = amstream_vbuf_write,
    .ioctl    = amstream_ioctl,
};

const static struct file_operations abuf_fops = {
    .owner    = THIS_MODULE,
    .open     = amstream_open,
    .release  = amstream_release,
    .write    = amstream_abuf_write,
    .ioctl    = amstream_ioctl,
};

const static struct file_operations mpts_fops = {
    .owner    = THIS_MODULE,
    .open     = amstream_open,
    .release  = amstream_release,
    .write    = amstream_mpts_write,
    .ioctl    = amstream_ioctl,
};

const static struct file_operations mpps_fops = {
    .owner    = THIS_MODULE,
    .open     = amstream_open,
    .release  = amstream_release,
    .write    = amstream_mpps_write,
    .ioctl    = amstream_ioctl,
};

const static struct file_operations mprm_fops = {
    .owner    = THIS_MODULE,
    .open     = amstream_open,
    .release  = amstream_release,
    .write    = amstream_mprm_write,
    .ioctl    = amstream_ioctl,
};

const static struct file_operations sub_fops = {
    .owner    = THIS_MODULE,
    .open     = amstream_open,
    .release  = amstream_release,
    .read     = amstream_sub_read,
    .poll     = amstream_sub_poll,
    .ioctl    = amstream_ioctl,
};

const static struct file_operations amstream_fops = {
    .owner    = THIS_MODULE,
    .open     = amstream_open,
    .release  = amstream_release,
    .ioctl    = amstream_ioctl,
};

/**************************************************/

struct dec_sysinfo amstream_dec_info;
static struct class *amstream_dev_class;
static DEFINE_MUTEX(amstream_mutex);

atomic_t subdata_ready = ATOMIC_INIT(0);
/* wait queue for poll */
static wait_queue_head_t amstream_sub_wait;

static stream_port_t ports[] =
{
    {
        .name = "amstream_vbuf",
        .type = PORT_TYPE_ES | PORT_TYPE_VIDEO,
        .fops = &vbuf_fops,
    },
    {
        .name = "amstream_abuf",
        .type = PORT_TYPE_ES | PORT_TYPE_AUDIO,
        .fops = &abuf_fops,
    },
    {
        .name = "amstream_mpts",
        .type = PORT_TYPE_MPTS | PORT_TYPE_VIDEO | PORT_TYPE_AUDIO | PORT_TYPE_SUB,
        .fops = &mpts_fops,
    },
    {
        .name  = "amstream_mpps",
        .type  = PORT_TYPE_MPPS | PORT_TYPE_VIDEO | PORT_TYPE_AUDIO | PORT_TYPE_SUB,
        .fops  = &mpps_fops,
    },
    {
        .name  = "amstream_rm",
        .type  = PORT_TYPE_RM | PORT_TYPE_VIDEO | PORT_TYPE_AUDIO,
        .fops  = &mprm_fops,
    },
    {
        .name  = "amstream_sub",
        .type  = PORT_TYPE_SUB,
        .fops  = &sub_fops,
    }
};

static stream_buf_t bufs[BUF_MAX_NUM] =
{
    {
        .reg_base = VLD_MEM_VIFIFO_REG_BASE,
        .type = BUF_TYPE_VIDEO,
        .buf_start=0,
        .buf_size=0,
        .first_tstamp=INVALID_PTS
    },
    {
        .reg_base = AIU_MEM_AIFIFO_REG_BASE,
        .type = BUF_TYPE_AUDIO,
        .buf_start=0,
        .buf_size=0,
        .first_tstamp=INVALID_PTS
    },
    {
        .reg_base = 0,
        .type = BUF_TYPE_SUBTITLE,
        .buf_start=0,
        .buf_size=0,
        .first_tstamp=INVALID_PTS

    }
};

static  void video_port_release( stream_port_t *port,struct stream_buf_s * pbuf,int release_num)
{
    switch(release_num)
        {
        default:
        case 0: /*release all*/
            /*  */
        case 4:esparser_release(pbuf);
        case 3:vdec_release(port->vformat);
        case 2:stbuf_release(pbuf);
        case 1:
            ;
        }
    return ;
}
static  int video_port_init( stream_port_t *port,struct stream_buf_s * pbuf)
{
    int r;
    if ((port->flag & PORT_FLAG_VFORMAT) == 0) {
        printk("vformat not set\n");
        return -EPERM;
    }

    r = stbuf_init(pbuf);
    if (r < 0) {
        return r;
    }

    r = vdec_init(port->vformat);
    if (r < 0) {
        video_port_release(port,pbuf,2);
        return r;
    }
    if(port->type & PORT_TYPE_ES)
        {
        r = esparser_init(pbuf);
        if (r < 0) {
            video_port_release(port,pbuf,3);
            printk("esparser_init() failed\n");
            return r;
        }
        }
    pbuf->flag|= BUF_FLAG_IN_USE;
    return 0;
}

static void audio_port_release( stream_port_t *port,struct stream_buf_s * pbuf,int release_num)
{
    switch(release_num)
        {
        default:
        case 0: /*release all*/
            /*  */
        case 4:esparser_release(pbuf);
        case 3:adec_release(port->vformat);
        case 2:stbuf_release(pbuf);
        case 1:
            ;
        }
    return ;
}

static int audio_port_reset(stream_port_t *port,struct stream_buf_s * pbuf)
{
    int r;

    if ((port->flag & PORT_FLAG_AFORMAT) == 0)
    {
        printk("aformat not set\n");
        return 0;
    }

    pts_stop(PTS_TYPE_AUDIO);

    stbuf_release(pbuf);

    r = stbuf_init(pbuf);
    if (r < 0)
        return r;

    r = adec_init(port);
    if (r < 0)
    {
        audio_port_release(port,pbuf,2);
        return r;
    }

    if(port->type & PORT_TYPE_ES)
        esparser_audio_reset(pbuf);

    if(port->type & PORT_TYPE_MPTS)
        tsdemux_audio_reset();

    if(port->type & PORT_TYPE_MPPS)
        psparser_audio_reset();

    if(port->type & PORT_TYPE_RM)
        rm_audio_reset();

    pbuf->flag |= BUF_FLAG_IN_USE;

    pts_start(PTS_TYPE_AUDIO);

    return 0;
}

static int sub_port_reset(stream_port_t *port,struct stream_buf_s * pbuf)
{
    int r;

    stbuf_release(pbuf);

    r = stbuf_init(pbuf);
    if (r < 0)
        return r;

    if(port->type & PORT_TYPE_MPTS)
        tsdemux_sub_reset();

    if(port->type & PORT_TYPE_MPPS)
        psparser_sub_reset();

    pbuf->flag |= BUF_FLAG_IN_USE;
    
    return 0;
}

static  int audio_port_init( stream_port_t *port,struct stream_buf_s * pbuf)
{
    int r;

        if ((port->flag & PORT_FLAG_AFORMAT) == 0){
        printk("aformat not set\n");
        return 0;
            }

        r = stbuf_init(pbuf);
        if (r < 0)
            return r;

        r = adec_init(port);
        if (r < 0)
            {
              audio_port_release(port,pbuf,2);
          return r;
            }
    if(port->type & PORT_TYPE_ES)
        {
            r = esparser_init(pbuf);
            if (r < 0)
            {
                    audio_port_release(port,pbuf,3);
                    return r;
                }
        }
    pbuf->flag |= BUF_FLAG_IN_USE;
    return 0;
}

static void sub_port_release(stream_port_t *port,struct stream_buf_s * pbuf)
{
    stbuf_release(pbuf);
    return;
}

static int sub_port_init(stream_port_t *port,struct stream_buf_s * pbuf)
{
    if ((port->flag & PORT_FLAG_SID) == 0)
    {
        printk("subtitle id not set\n");
        return 0;
    }

    return stbuf_init(pbuf);
}

static  int amstream_port_init( stream_port_t *port)
{
    int r;
    stream_buf_t *pvbuf=&bufs[BUF_TYPE_VIDEO];
    stream_buf_t *pabuf=&bufs[BUF_TYPE_AUDIO];
    stream_buf_t *psbuf=&bufs[BUF_TYPE_SUBTITLE];

    if((port->type & PORT_TYPE_AUDIO) && (port->flag& PORT_FLAG_AFORMAT))
        {
        r=audio_port_init(port,pabuf);
        if(r<0)
            {
            printk("audio_port_init  failed\n");
            goto error1;
            }
        }
    if((port->type & PORT_TYPE_VIDEO) && (port->flag& PORT_FLAG_VFORMAT))
        {
        r=video_port_init(port,pvbuf);
        if(r<0)
            {
            printk("video_port_init  failed\n");
            goto error2;
            }
        }
    if((port->type & PORT_TYPE_SUB) && (port->flag& PORT_FLAG_SID))
        {
        r=sub_port_init(port,psbuf);
        if(r<0)
            {
            printk("sub_port_init  failed\n");
            goto error3;
            }
        }

    if(port->type & PORT_TYPE_MPTS)
        {
        r = tsdemux_init((port->flag & PORT_FLAG_VID) ? port->vid : 0xffff,
                                     (port->flag & PORT_FLAG_AID) ? port->aid : 0xffff,
                                     (port->flag & PORT_FLAG_SID) ? port->sid : 0xffff);
        if(r<0)
            {
            printk("tsdemux_init  failed\n");
            goto error4;
            }
        }
    if(port->type & PORT_TYPE_MPPS)
        {
           r = psparser_init((port->flag & PORT_FLAG_VID) ? port->vid : 0xffff,
                                     (port->flag & PORT_FLAG_AID) ? port->aid : 0xffff,
                                     (port->flag & PORT_FLAG_SID) ? port->sid : 0xffff);
           if(r<0)
            {
            printk("psparser_init  failed\n");
            goto error5;
            }
        }
    if(port->type & PORT_TYPE_RM)
        {
         rm_set_vasid((port->flag & PORT_FLAG_VID) ? port->vid : 0xffff,
                                     (port->flag & PORT_FLAG_AID) ? port->aid : 0xffff);
        }

    tsync_audio_break(0); // clear audio break

    port->flag|=PORT_FLAG_INITED;
    return 0;
/*errors follow here*/
error5:
    tsdemux_release();
error4:
    sub_port_release(port,psbuf);
error3:
    video_port_release(port,pvbuf,0);
error2:
    audio_port_release(port,pabuf,0);
error1:

    return r;
}
static  int amstream_port_release( stream_port_t *port)
{
    stream_buf_t *pvbuf=&bufs[BUF_TYPE_VIDEO];
    stream_buf_t *pabuf=&bufs[BUF_TYPE_AUDIO];

    if (port->type & PORT_TYPE_MPTS) {
        tsdemux_release();
    }

    if (port->type & PORT_TYPE_MPPS) {
        psparser_release();
    }

    if (port->type & PORT_TYPE_RM) {
        rmparser_release();
    }

    if (port->type & PORT_TYPE_VIDEO) {
        video_port_release(port,pvbuf,0);
    }

    if (port->type & PORT_TYPE_AUDIO) {
        audio_port_release(port,pabuf,0);
    }

    port->flag = 0;
    return 0;
}

static void amstream_change_avid(stream_port_t *port)
{
    if(port->type & PORT_TYPE_MPTS)
    {
        tsdemux_change_avid((port->flag & PORT_FLAG_VID) ? port->vid : 0xffff,
                            (port->flag & PORT_FLAG_AID) ? port->aid : 0xffff);
    }

    if(port->type & PORT_TYPE_MPPS)
    {
        psparser_change_avid((port->flag & PORT_FLAG_VID) ? port->vid : 0xffff,
                             (port->flag & PORT_FLAG_AID) ? port->aid : 0xffff);
    }

    if(port->type & PORT_TYPE_RM)
    {
        rm_set_vasid((port->flag & PORT_FLAG_VID) ? port->vid : 0xffff,
                      (port->flag & PORT_FLAG_AID) ? port->aid : 0xffff);
    }

    return;
}

static void amstream_change_sid(stream_port_t *port)
{
    if(port->type & PORT_TYPE_MPTS)
    {
        tsdemux_change_sid((port->flag & PORT_FLAG_SID) ? port->sid : 0xffff);
    }

    if(port->type & PORT_TYPE_MPPS)
    {
        psparser_change_sid((port->flag & PORT_FLAG_SID) ? port->sid : 0xffff);
    }

    return;
}

/**************************************************/
static ssize_t amstream_vbuf_write(struct file *file, const char *buf,
                               size_t count, loff_t * ppos)
{
    stream_port_t *port = (stream_port_t *)file->private_data;
    stream_buf_t *pbuf=&bufs[BUF_TYPE_VIDEO];
    int r;
    if (!(port->flag & PORT_FLAG_INITED)) {
        r=amstream_port_init(port);
        if(r<0)
            return r;
    }

    r = esparser_write(file, pbuf, buf, count);

#ifdef DATA_DEBUG
    debug_file_write(buf, r);
#endif

    return r;
}

static ssize_t amstream_abuf_write(struct file *file, const char *buf,
                                size_t count, loff_t * ppos)
{
    stream_port_t *port = (stream_port_t *)file->private_data;
    stream_buf_t *pbuf=&bufs[BUF_TYPE_AUDIO];
    int r;

    if (!(port->flag & PORT_FLAG_INITED)) {
        r=amstream_port_init(port);
        if(r<0)
            return r;
    }

    return esparser_write(file,pbuf, buf, count);
}

static ssize_t amstream_mpts_write(struct file *file, const char *buf,
                                   size_t count, loff_t * ppos)
{
    stream_port_t *port = (stream_port_t *)file->private_data;
    stream_buf_t *pvbuf=&bufs[BUF_TYPE_VIDEO];
    stream_buf_t *pabuf=&bufs[BUF_TYPE_AUDIO];
    int r;
    if (!(port->flag & PORT_FLAG_INITED))
       {
           r=amstream_port_init(port);
           if(r<0)
            return r;
       }

#ifdef DATA_DEBUG
    debug_file_write(buf, count);
#endif
    return tsdemux_write(file, pvbuf, pabuf,buf, count);
}

static ssize_t amstream_mpps_write(struct file *file, const char *buf,
                                size_t count, loff_t * ppos)
{
    stream_port_t *port = (stream_port_t *)file->private_data;
    stream_buf_t *pvbuf=&bufs[BUF_TYPE_VIDEO];
    stream_buf_t *pabuf=&bufs[BUF_TYPE_AUDIO];
    int r;

   if (!(port->flag & PORT_FLAG_INITED))
    {
       r=amstream_port_init(port);
          if(r<0)
        return r;
    }
    return psparser_write(file, pvbuf, pabuf,buf, count);
}

static ssize_t amstream_mprm_write(struct file *file, const char *buf,
                                size_t count, loff_t * ppos)
{
    stream_port_t *port = (stream_port_t *)file->private_data;
    stream_buf_t *pvbuf=&bufs[BUF_TYPE_VIDEO];
    stream_buf_t *pabuf=&bufs[BUF_TYPE_AUDIO];
    int r;

    if (!(port->flag & PORT_FLAG_INITED))
    {
        r=amstream_port_init(port);
        if(r<0)
            return r;
    }
    return rmparser_write(file, pvbuf,pabuf,buf, count);
}

static ssize_t amstream_sub_read(struct file *file, char __user *buf, size_t count, loff_t * ppos)
{
    u32 sub_rp, sub_wp, sub_start, data_size, res;
    stream_buf_t *s_buf = &bufs[BUF_TYPE_SUBTITLE];
    stream_port_t *st = (stream_port_t *)file->private_data;
    dma_addr_t buf_map;

    sub_rp = stbuf_sub_rp_get();
    sub_wp = stbuf_sub_wp_get();
    sub_start = stbuf_sub_start_get();

    if (sub_wp > sub_rp)
    {
        data_size = sub_wp - sub_rp;
    }
    else
    {
        data_size = s_buf->buf_size - sub_rp + sub_wp;
    }

    if (data_size > count)
        data_size = count;

    if (sub_wp < sub_rp)
    {
        int first_num = s_buf->buf_size - (sub_rp - sub_start);

        buf_map = dma_map_single(st->class_dev, (void *)buf, first_num, DMA_FROM_DEVICE);
        res = copy_to_user((void *)buf_map, (void *)sub_rp, first_num);
        dma_unmap_single(st->class_dev, buf_map, first_num, DMA_FROM_DEVICE);
        if (res)
        {
            if (res > 0)
            {
                stbuf_sub_rp_set(sub_rp + first_num - res);
            }
            return first_num-res;
        }

        buf_map = dma_map_single(st->class_dev, (void *)buf + first_num, data_size - first_num, DMA_FROM_DEVICE);
        res = copy_to_user((void *)buf_map, (void *)sub_start, data_size - first_num);
        dma_unmap_single(st->class_dev, buf_map, data_size - first_num, DMA_FROM_DEVICE);
        if (res >= 0)
        {            
            stbuf_sub_rp_set(sub_start + data_size - first_num - res);
        }
        return data_size-res;
    }
    else
    {
        buf_map = dma_map_single(st->class_dev, (void *)buf, data_size, DMA_FROM_DEVICE);
        res = copy_to_user((void *)buf_map, (void *)sub_rp, data_size);
        dma_unmap_single(st->class_dev, buf_map, data_size, DMA_FROM_DEVICE);
        if (res >= 0)
        {
            stbuf_sub_rp_set(sub_rp + data_size - res);
        }
        return data_size-res;
    }
}

static unsigned int amstream_sub_poll(struct file *file, poll_table *wait_table)
{
    poll_wait(file, &amstream_sub_wait, wait_table);

    if (atomic_read(&subdata_ready))
    {
        atomic_set(&subdata_ready, 0);
        return POLLOUT | POLLWRNORM;
    }

    return 0;
}

static int amstream_open(struct inode *inode, struct file *file)
{
    s32 i;
    stream_port_t *s;
    stream_port_t *this = &ports[iminor(inode)];

    if (iminor(inode) >= MAX_PORT_NUM) {
        return (-ENODEV);
    }

    if (this->flag & PORT_FLAG_IN_USE) {
        return (-EBUSY);
    }

    /* check other ports conflict */
    for (s = &ports[0], i = 0; i < MAX_PORT_NUM; i++, s++) {
        if ((s->flag & PORT_FLAG_IN_USE) &&
            ((this->type) & (s->type) & (PORT_TYPE_VIDEO | PORT_TYPE_AUDIO))) {
            return (-EBUSY);
        }
    }

    file->f_op = this->fops;
    file->private_data = this;

    this->flag = PORT_FLAG_IN_USE;

    #ifdef DATA_DEBUG
    debug_filp = filp_open(DEBUG_FILE_NAME, O_WRONLY, 0);
    if (IS_ERR(debug_filp))
    {
        printk("amstream: open debug file failed\n");
        debug_filp = NULL;
    }
    #endif

    return 0;
}

static int amstream_release(struct inode *inode, struct file *file)
{
    stream_port_t *this = &ports[iminor(inode)];

    if (iminor(inode) >= MAX_PORT_NUM) {
        return (-ENODEV);
    }
    if(this->flag &PORT_FLAG_INITED)
    amstream_port_release(this);
    this->flag=0;

    timestamp_pcrscr_set(0);
    
    #ifdef DATA_DEBUG
    if (debug_filp)
    {
        filp_close(debug_filp, current->files);
        debug_filp = NULL;
        debug_file_pos = 0;
    }
    #endif

    return 0;
}

static int amstream_ioctl(struct inode *inode, struct file *file,
                        unsigned int cmd, ulong arg)
{
    s32 r = 0;
    stream_port_t *this = &ports[iminor(inode)];

    switch (cmd) {

        case AMSTREAM_IOC_VB_START:
            if ((this->type & PORT_TYPE_VIDEO) &&
                ((bufs[BUF_TYPE_VIDEO].flag & BUF_FLAG_IN_USE) == 0)) {
                bufs[BUF_TYPE_VIDEO].buf_start = arg;
            }
            else
                r = -EINVAL;
            break;

        case AMSTREAM_IOC_VB_SIZE:
            if ((this->type & PORT_TYPE_VIDEO) &&
                ((bufs[BUF_TYPE_VIDEO].flag & BUF_FLAG_IN_USE) == 0)) {
                    r=stbuf_change_size(&bufs[BUF_TYPE_VIDEO],arg);
            }
            else
                r = -EINVAL;
            break;

        case AMSTREAM_IOC_AB_START:
            if ((this->type & PORT_TYPE_AUDIO) &&
                ((bufs[BUF_TYPE_AUDIO].flag & BUF_FLAG_IN_USE) == 0)) {
                bufs[BUF_TYPE_AUDIO].buf_start = arg;
            }
            else
                r = -EINVAL;
            break;

        case AMSTREAM_IOC_AB_SIZE:
            if ((this->type & PORT_TYPE_AUDIO) &&
                ((bufs[BUF_TYPE_AUDIO].flag & BUF_FLAG_IN_USE) == 0)) {
                r=stbuf_change_size(&bufs[BUF_TYPE_AUDIO],arg);
            }
            else
                r = -EINVAL;
            break;

        case AMSTREAM_IOC_VFORMAT:
            if ((this->type & PORT_TYPE_VIDEO) &&
                (arg < VFORMAT_MAX)) {
                this->vformat = (vformat_t)arg;
                this->flag |= PORT_FLAG_VFORMAT;
            }
            else
                r = -EINVAL;
            break;

        case AMSTREAM_IOC_AFORMAT:
            if ((this->type & PORT_TYPE_AUDIO) &&
                (arg < AFORMAT_MAX)) {
                this->aformat = (aformat_t)arg;
                this->flag |= PORT_FLAG_AFORMAT;
            }
            else
                r = -EINVAL;
            break;

        case AMSTREAM_IOC_VID:
            if (this->type & PORT_TYPE_VIDEO) {
                this->vid = (u32)arg;
                this->flag |= PORT_FLAG_VID;
            }
            else
                r = -EINVAL;

            break;

        case AMSTREAM_IOC_AID:
            if (this->type & PORT_TYPE_AUDIO) {
                this->aid = (u32)arg;
                this->flag |= PORT_FLAG_AID;

                if (this->flag & PORT_FLAG_INITED)
                {
                    tsync_audio_break(1);
                    amstream_change_avid(this);
                }
            }
            else
                r = -EINVAL;
            break;

        case AMSTREAM_IOC_SID:
            if (this->type & PORT_TYPE_SUB)
            {
                this->sid = (u32)arg;
                this->flag |= PORT_FLAG_SID;

                if (this->flag & PORT_FLAG_INITED)
                {
                    amstream_change_sid(this);
                }
            }
            else
                r = -EINVAL;

            break;

        case AMSTREAM_IOC_VB_STATUS:
            if (this->type & PORT_TYPE_VIDEO) {
                struct am_io_param *p = (void*)arg;
                stream_buf_t *buf = &bufs[BUF_TYPE_VIDEO];

                if (p == NULL)
                    r = -EINVAL;

                p->status.size=buf->buf_size;
                p->status.data_len=stbuf_level(buf);
                p->status.free_len=stbuf_space(buf);
            } else
                r = -EINVAL;
            break;

        case AMSTREAM_IOC_AB_STATUS:
            if (this->type & PORT_TYPE_AUDIO) {
                struct am_io_param *p = (void*)arg;
                stream_buf_t *buf = &bufs[BUF_TYPE_AUDIO];

                if (p == NULL)
                    r = -EINVAL;

                p->status.size=buf->buf_size;
                p->status.data_len=stbuf_level(buf);
                p->status.free_len=stbuf_space(buf);
            } else
                r = -EINVAL;
            break;

        case AMSTREAM_IOC_SYSINFO:
            if (this->type & PORT_TYPE_VIDEO)
                copy_from_user((void *)&amstream_dec_info, (void *)arg, sizeof(amstream_dec_info));
            else
                r = -EINVAL;
            break;

        case AMSTREAM_IOC_ACHANNEL:
            if (this->type & PORT_TYPE_AUDIO) {
                this->achanl = (u32)arg;
            } else
                r = -EINVAL;
            break;

        case AMSTREAM_IOC_SAMPLERATE:
            if (this->type & PORT_TYPE_AUDIO) {
                this->asamprate = (u32)arg;
            } else
                r = -EINVAL;
            break;

        case AMSTREAM_IOC_DATAWIDTH:
            if (this->type & PORT_TYPE_AUDIO) {
                this->adatawidth = (u32)arg;
            } else
                r = -EINVAL;
            break;

        case AMSTREAM_IOC_TSTAMP:
            if ((this->type & (PORT_TYPE_AUDIO | PORT_TYPE_VIDEO)) ==
                ((PORT_TYPE_AUDIO | PORT_TYPE_VIDEO)))
                r = -EINVAL;
            else if (this->type & PORT_TYPE_VIDEO) {
                r = es_vpts_checkin(&bufs[BUF_TYPE_VIDEO], arg);
            }
            else if (this->type & PORT_TYPE_AUDIO) {
                r = es_apts_checkin(&bufs[BUF_TYPE_AUDIO], arg);
            }
            break;

        case AMSTREAM_IOC_VDECSTAT:
            if ((this->type & PORT_TYPE_VIDEO) == 0)
                return -EINVAL;
            if (amstream_vdec_status == NULL)
                return -ENODEV;
            else {
                struct vdec_status vstatus;
                struct am_io_param *p = (void*)arg;

                if (p == NULL)
                    return -EINVAL;
                amstream_vdec_status(&vstatus);
                p->vstatus.width = vstatus.width;
                p->vstatus.height = vstatus.height;
                p->vstatus.fps = vstatus.fps;
                p->vstatus.error_count = vstatus.error_count;
                p->vstatus.status = vstatus.status;

                return 0;
            }

        case AMSTREAM_IOC_ADECSTAT:
            if ((this->type & PORT_TYPE_AUDIO) == 0)
                return -EINVAL;
            if (amstream_adec_status == NULL)
                return -ENODEV;
            else {
                struct adec_status astatus;
                struct am_io_param *p = (void*)arg;

                if (p == NULL)
                    return -EINVAL;
                amstream_adec_status(&astatus);
                p->astatus.channels = astatus.channels;
                p->astatus.sample_rate = astatus.sample_rate;
                p->astatus.resolution = astatus.resolution;
                p->astatus.error_count = astatus.error_count;
                p->astatus.status = astatus.status;

                return 0;
            }

        case AMSTREAM_IOC_PORT_INIT:
            r=amstream_port_init(this);
            break;

        case AMSTREAM_IOC_TRICKMODE:
            if ((this->type & PORT_TYPE_VIDEO) == 0)
                return -EINVAL;
            if (amstream_vdec_trickmode == NULL)
                return -ENODEV;
            else
                amstream_vdec_trickmode(arg);
            break;

        case AMSTREAM_IOC_AUDIO_INFO:
            if (this->type & PORT_TYPE_VIDEO)
            	set_real_audio_info((void *)arg);
            else
                r = -EINVAL;
            break;

        case AMSTREAM_IOC_AUDIO_RESET:
        	if (this->type & PORT_TYPE_AUDIO) {
            	stream_buf_t *pabuf=&bufs[BUF_TYPE_AUDIO];

            r=audio_port_reset(this, pabuf);
        }
        else
            r = -EINVAL;

        break;

    case AMSTREAM_IOC_SUB_RESET:
        if (this->type & PORT_TYPE_SUB)
        {
            stream_buf_t *psbuf = &bufs[BUF_TYPE_SUBTITLE];

            r = sub_port_reset(this, psbuf);
        }
        else
            r = -EINVAL;
        break;

    case AMSTREAM_IOC_SUB_LENGTH:
        if (this->type & PORT_TYPE_SUB)
        {
            u32 sub_wp, sub_rp;
            stream_buf_t *psbuf = &bufs[BUF_TYPE_SUBTITLE];

            sub_wp = stbuf_sub_wp_get();
            sub_rp = stbuf_sub_rp_get();

            if (sub_wp > sub_rp)
            {
                *((u32 *)arg) = sub_wp - sub_rp;
            }
            else
            {
                *((u32 *)arg) = psbuf->buf_size - (sub_rp - sub_wp);
            }
        }
        else
            r = -EINVAL;
        break;

    case AMSTREAM_IOC_SET_DEC_RESET:
        tsync_set_dec_reset();
        break;

    case AMSTREAM_IOC_TS_SKIPBYTE:
        if ((int)arg >= 0)
            tsdemux_set_skipbyte(arg);
        else
            r = -EINVAL;
        break;
        
        default:
            r = -ENOIOCTLCMD;
    }

    return r;
}

static int  amstream_probe(struct platform_device *pdev)
{
    int i;
    int r;
    stream_port_t *st;

    printk("Amlogic A/V streaming port init\n");

    r = astream_dev_register();
    if (r) {
        return r;
    }

    r = register_chrdev(AMSTREAM_MAJOR, "amstream", &amstream_fops);

    if (r < 0) {
        printk("Can't allocate major for amstreaming device\n");

    goto error2;
    }

    vdec_set_resource(platform_get_resource(pdev, IORESOURCE_MEM, 0), (void *)&amstream_dec_info);

    amstream_dev_class = class_create(THIS_MODULE, DEVICE_NAME);

    for (st = &ports[0], i = 0; i < MAX_PORT_NUM; i++, st++) {
        st->class_dev = device_create(amstream_dev_class, NULL,
                                MKDEV(AMSTREAM_MAJOR, i), NULL,
                                ports[i].name);
    }

    amstream_vdec_status = NULL;
    amstream_adec_status = NULL;
    if(tsdemux_class_register()!=0)
        {
         r= (-EIO);
         goto error3;
        }
     if(stbuf_change_size(&bufs[BUF_TYPE_VIDEO],DEFAULT_VIDEO_BUFFER_SIZE)!=0)
        {
         r= (-ENOMEM);
         goto error4;
        }
    if(stbuf_change_size(&bufs[BUF_TYPE_AUDIO],DEFAULT_AUDIO_BUFFER_SIZE)!=0)
        {
             r= (-ENOMEM);
         goto error5;
        }
    if(stbuf_change_size(&bufs[BUF_TYPE_SUBTITLE],DEFAULT_SUBTITLE_BUFFER_SIZE)!=0)
        {
            r= (-ENOMEM);
            goto error6;
        }
    if(stbuf_fetch_init()!=0)
        {
            r= (-ENOMEM);
            goto error7;
        }
    init_waitqueue_head(&amstream_sub_wait);

	return 0;

error7:
    stbuf_change_size(&bufs[BUF_TYPE_SUBTITLE],0);
error6:
    stbuf_change_size(&bufs[BUF_TYPE_AUDIO],0);
error5:
    stbuf_change_size(&bufs[BUF_TYPE_VIDEO],0);
error4:
    tsdemux_class_unregister();
error3:
    for (st = &ports[0], i = 0; i < MAX_PORT_NUM; i++, st++)
        device_destroy(amstream_dev_class,MKDEV(AMSTREAM_MAJOR, i));
    class_destroy(amstream_dev_class);
error2:
    unregister_chrdev(AMSTREAM_MAJOR,"amstream");
//error1:
    astream_dev_unregister();
    return (r);
}

static int  amstream_remove(struct platform_device *pdev)
{
    int i;
    stream_port_t *st;
    stbuf_change_size(&bufs[BUF_TYPE_VIDEO],0);
    stbuf_change_size(&bufs[BUF_TYPE_AUDIO],0);
    stbuf_fetch_release();
    tsdemux_class_unregister();
    for (st = &ports[0], i = 0; i < MAX_PORT_NUM; i++, st++) {
        device_destroy(amstream_dev_class, MKDEV(AMSTREAM_MAJOR, i));
    }

    class_destroy(amstream_dev_class);

    unregister_chrdev(AMSTREAM_MAJOR, DEVICE_NAME);

    astream_dev_unregister();

    amstream_vdec_status = NULL;
    amstream_adec_status = NULL;
    amstream_vdec_trickmode = NULL;

    printk("Amlogic A/V streaming port release\n");

    return 0;
}

void set_vdec_func(int (*vdec_func)(struct vdec_status *))
{
    amstream_vdec_status = vdec_func;
    return;
}

void set_adec_func(int (*adec_func)(struct adec_status *))
{
    amstream_adec_status = adec_func;
    return;
}

void set_trickmode_func(int (*trickmode_func)(unsigned long trickmode))
{
    amstream_vdec_trickmode = trickmode_func;
    return;
}

void wakeup_sub_poll(void)
{
    atomic_set(&subdata_ready, 1);
    wake_up_interruptible(&amstream_sub_wait);
    
    return;
}

EXPORT_SYMBOL(set_vdec_func);
EXPORT_SYMBOL(set_adec_func);
EXPORT_SYMBOL(set_trickmode_func);
EXPORT_SYMBOL(wakeup_sub_poll);

static struct platform_driver
amstream_driver = {
    .probe      = amstream_probe,
    .remove     = amstream_remove,
    .driver     = {
        .name   = "amstream",
    }
};

static int __init amstream_module_init(void)
{
    if (platform_driver_register(&amstream_driver)) {
        printk("failed to register amstream module\n");
        return -ENODEV;
    }

    return 0;
}

static void __exit amstream_module_exit(void)
{
    platform_driver_unregister(&amstream_driver);
    return ;
}

module_init(amstream_module_init);
module_exit(amstream_module_exit);

MODULE_DESCRIPTION("AMLOGIC streaming port driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Tim Yao <timyao@amlogic.com>");
