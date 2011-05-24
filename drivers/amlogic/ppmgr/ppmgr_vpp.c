/*
 *  video post process. 
 */

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/timer.h>
#include <linux/platform_device.h>
#include <linux/amports/ptsserv.h>
#include <linux/amports/canvas.h>
#include <linux/vout/vinfo.h>
#include <linux/vout/vout_notify.h>
#include <linux/amports/vframe.h>
#include <linux/amports/vfp.h>
#include <linux/amports/vframe_provider.h>
#include <mach/am_regs.h>
#include <linux/amlog.h>
#include <linux/ge2d/ge2d_main.h>
#include <linux/ge2d/ge2d.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/semaphore.h>
#include <linux/sched.h>
#include "ppmgr_log.h"
#include "ppmgr_pri.h"
#include "ppmgr_dev.h"

#define VF_POOL_SIZE 4

#define PPMGR_CANVAS_INDEX 0x50

#define THREAD_INTERRUPT 0
#define THREAD_RUNNING 1

typedef struct ppframe_s {
    vframe_t  frame;
    int       index;
    vframe_t *dec_frame;
} ppframe_t;

static spinlock_t lock = SPIN_LOCK_UNLOCKED;
static bool ppmgr_blocking = false;

static struct ppframe_s vfp_pool[VF_POOL_SIZE];
static struct vframe_s *vfp_pool_free[VF_POOL_SIZE+1];
static struct vframe_s *vfp_pool_ready[VF_POOL_SIZE+1];

static vfq_t q_ready, q_free;

static struct semaphore thread_sem;
static DEFINE_MUTEX(ppmgr_mutex);

static inline void ppmgr_vf_put_dec(vframe_t *vf);

#define to_ppframe(vf)	\
	container_of(vf, struct ppframe_s, frame)

/************************************************
*
*   Canvas helpers.
*
*************************************************/
static inline u32 index2canvas(u32 index)
{
    const u32 canvas_tab[4] = {
        PPMGR_CANVAS_INDEX+0, PPMGR_CANVAS_INDEX+1, PPMGR_CANVAS_INDEX+2, PPMGR_CANVAS_INDEX+3
    };
    return canvas_tab[index];
}

/************************************************
*
*   ppmgr as a frame provider
*
*************************************************/

static vframe_t *ppmgr_vf_peek(void)
{
    return vfq_peek(&q_ready);
}

static vframe_t *ppmgr_vf_get(void)
{
    return vfq_pop(&q_ready);
}

static void ppmgr_vf_put(vframe_t *vf)
{
    ppframe_t *pp_vf = to_ppframe(vf);

    /* the frame is in bypass mode, put the decoder frame */
    if (pp_vf->dec_frame)
        ppmgr_vf_put_dec(pp_vf->dec_frame);

    vfq_push(&q_free, vf);
}

static int ppmgr_event_cb(int type, void *data, void *private_data)
{
    if (type & VFRAME_EVENT_RECEIVER_PUT) {
#if DDD
            printk("video put, avail=%d, free=%d\n", vfq_level(&q_ready),  vfq_level(&q_free));
#endif
        up(&thread_sem);
    }

    return 0;        
}

static int ppmgr_vf_states(vframe_states_t *states)
{
    unsigned long flags;
    spin_lock_irqsave(&lock, flags);

    states->vf_pool_size = VF_POOL_SIZE;
    states->buf_recycle_num = 0;
    states->buf_free_num = vfq_level(&q_free);
    states->buf_avail_num = vfq_level(&q_ready);

    spin_unlock_irqrestore(&lock, flags);

    return 0;
}

static const struct vframe_provider_s ppmgr_vf_provider =
{
    .peek = ppmgr_vf_peek,
    .get  = ppmgr_vf_get,
    .put  = ppmgr_vf_put,
    .event_cb = ppmgr_event_cb,
    .vf_states = ppmgr_vf_states,
};

/************************************************
*
*   ppmgr as a frame receiver
*
*************************************************/

static int ppmgr_receiver_event_fun(int type, void* data, void*);

static const struct vframe_receiver_op_s ppmgr_vf_receiver =
{
    .event_cb = ppmgr_receiver_event_fun
};

static int ppmgr_receiver_event_fun(int type, void *data, void *private_data)
{
    switch(type) {
        case VFRAME_EVENT_PROVIDER_VFRAME_READY:
#if DDD
            printk("dec put, avail=%d, free=%d\n", vfq_level(&q_ready),  vfq_level(&q_free));
#endif
            up(&thread_sem);
            break;
        default:
            break;        
	  }    		

    return 0;
}

void vf_local_init(void) 
{
    int i;

    vfq_init(&q_free, VF_POOL_SIZE+1, &vfp_pool_free[0]);
    vfq_init(&q_ready, VF_POOL_SIZE+1, &vfp_pool_ready[0]);

    for (i=0; i < VF_POOL_SIZE; i++) {
        vfp_pool[i].index = i;
        vfq_push(&q_free, &vfp_pool[i].frame);
    }
    
    init_MUTEX(&thread_sem);
}

static const struct vframe_provider_s *dec_vfp = NULL;

const vframe_receiver_op_t* vf_ppmgr_reg_provider(const struct vframe_provider_s *p)
{  
    const vframe_receiver_op_t *r = NULL;
    
    mutex_lock(&ppmgr_mutex);

    if (dec_vfp) {
        mutex_unlock(&ppmgr_mutex);
        return NULL;
    }

    vf_local_init();

    vf_reg_provider(&ppmgr_vf_provider);

    if (start_vpp_task() == 0) {
        r = &ppmgr_vf_receiver;
        dec_vfp = p;
    }
    
    mutex_unlock(&ppmgr_mutex);

    return r;
}

void vf_ppmgr_unreg_provider(void)
{
    mutex_lock(&ppmgr_mutex);

    stop_vpp_task();

    vf_unreg_provider();

    dec_vfp = NULL;

    mutex_unlock(&ppmgr_mutex);
}

void vf_ppmgr_reset(void)
{
	ppmgr_blocking = true;
	
	up(&thread_sem);
}

static inline vframe_t *ppmgr_vf_peek_dec(void)
{
    if (dec_vfp)
        return dec_vfp->peek();

    return NULL;
}

static inline vframe_t *ppmgr_vf_get_dec(void)
{
    if ((dec_vfp) && (!ppmgr_blocking))
    	return dec_vfp->get();

    return NULL;
}

static inline void ppmgr_vf_put_dec(vframe_t *vf)
{
    if ((dec_vfp) && (!ppmgr_blocking))
        dec_vfp->put(vf);
}

/************************************************
*
*   main task functions.
*
*************************************************/
static void vf_rotate_adjust(vframe_t *vf, vframe_t *new_vf, int angle)
{
    int w, h;

    if (angle & 1) {
        int ar = (vf->ratio_control >> DISP_RATIO_ASPECT_RATIO_BIT) & 0xff;

        h = min((int)vf->width, (int)ppmgr_device.disp_height);

        if (ar == 0)
            w = vf->height * h / vf->width;
        else
            w = (ar * h) >> 8;

        new_vf->ratio_control = 0;

    } else {
        if ((vf->width < ppmgr_device.disp_width) && (vf->height < ppmgr_device.disp_height)) {
            w = vf->width;
            h = vf->height;
        } else {
            if ((vf->width * ppmgr_device.disp_height) > (ppmgr_device.disp_width * vf->height)) {
                w = ppmgr_device.disp_width;
                h = ppmgr_device.disp_width * vf->height / vf->width;
            } else {
                h = ppmgr_device.disp_height;
                w = ppmgr_device.disp_height * vf->width / vf->height;
            }
        }

        new_vf->ratio_control = vf->ratio_control;
    }
    
    new_vf->width = w;
    new_vf->height = h;
}

static void process_vf_rotate(vframe_t *vf, ge2d_context_t *context, config_para_ex_t *ge2d_config)
{
    vframe_t *new_vf;
    ppframe_t *pp_vf;
    canvas_t cs0,cs1,cs2,cd;

    new_vf = vfq_pop(&q_free);
    
    if (unlikely((!new_vf) || (!vf)))
        return;

    pp_vf = to_ppframe(new_vf);

    pp_vf->dec_frame = (ppmgr_device.bypass || (ppmgr_device.angle == 0)) ? vf : NULL;

    if (pp_vf->dec_frame) {
        /* bypass mode */
        *new_vf = *vf;
        vfq_push(&q_ready, new_vf);
        return;
    }

    new_vf->duration = vf->duration;
    new_vf->duration_pulldown = vf->duration_pulldown;
    new_vf->pts = vf->pts;
    new_vf->type = VIDTYPE_VIU_444 | VIDTYPE_VIU_SINGLE_PLANE | VIDTYPE_VIU_FIELD;
    new_vf->canvas0Addr = new_vf->canvas1Addr = index2canvas(pp_vf->index);

    vf_rotate_adjust(vf, new_vf, ppmgr_device.angle);

    /* data operating. */ 
    ge2d_config->alu_const_color= 0;//0x000000ff;
    ge2d_config->bitmask_en  = 0;
    ge2d_config->src1_gb_alpha = 0;//0xff;
    ge2d_config->dst_xy_swap = 0;

    canvas_read(vf->canvas0Addr&0xff,&cs0);
    canvas_read((vf->canvas0Addr>>8)&0xff,&cs1);
    canvas_read((vf->canvas0Addr>>16)&0xff,&cs2);
    ge2d_config->src_planes[0].addr = cs0.addr;
    ge2d_config->src_planes[0].w = cs0.width;
    ge2d_config->src_planes[0].h = cs0.height;
    ge2d_config->src_planes[1].addr = cs1.addr;
    ge2d_config->src_planes[1].w = cs1.width;
    ge2d_config->src_planes[1].h = cs1.height;
    ge2d_config->src_planes[2].addr = cs2.addr;
    ge2d_config->src_planes[2].w = cs2.width;
    ge2d_config->src_planes[2].h = cs2.height;
    
    canvas_read(new_vf->canvas0Addr&0xff,&cd);
    ge2d_config->dst_planes[0].addr = cd.addr;
    ge2d_config->dst_planes[0].w = cd.width;
    ge2d_config->dst_planes[0].h = cd.height;

    ge2d_config->src_key.key_enable = 0;
    ge2d_config->src_key.key_mask = 0;
    ge2d_config->src_key.key_mode = 0;

    ge2d_config->src_para.canvas_index=vf->canvas0Addr;
    ge2d_config->src_para.mem_type = CANVAS_TYPE_INVALID;
    ge2d_config->src_para.format = GE2D_FORMAT_M24_YUV420;
    ge2d_config->src_para.fill_color_en = 0;
    ge2d_config->src_para.fill_mode = 0;
    ge2d_config->src_para.x_rev = 0;
    ge2d_config->src_para.y_rev = 0;
    ge2d_config->src_para.color = 0xffffffff;
    ge2d_config->src_para.top = 0;
    ge2d_config->src_para.left = 0;
    ge2d_config->src_para.width = vf->width;
    ge2d_config->src_para.height = vf->height;

    ge2d_config->src2_para.mem_type = CANVAS_TYPE_INVALID;

    ge2d_config->dst_para.canvas_index=new_vf->canvas0Addr;
    ge2d_config->dst_para.mem_type = CANVAS_TYPE_INVALID;
    //ge2d_config->dst_para.mem_type = CANVAS_OSD0;
    //ge2d_config->dst_para.format = GE2D_FORMAT_M24_YUV420;
    ge2d_config->dst_para.format = GE2D_FORMAT_S24_YUV444;
    ge2d_config->dst_para.fill_color_en = 0;
    ge2d_config->dst_para.fill_mode = 0;
    ge2d_config->dst_para.x_rev = 0;
    ge2d_config->dst_para.y_rev = 0;
    ge2d_config->dst_xy_swap=0;

    if(ppmgr_device.angle==1)
        ge2d_config->dst_xy_swap=1;
    else if(ppmgr_device.angle==2)
        ge2d_config->dst_para.y_rev=1;
    else if(ppmgr_device.angle==3)  {
		ge2d_config->dst_xy_swap=1;
		ge2d_config->dst_para.y_rev=1;
		ge2d_config->dst_para.x_rev = 1;
    }
    ge2d_config->dst_para.color = 0;
    ge2d_config->dst_para.top = 0;
    ge2d_config->dst_para.left = 0;
    ge2d_config->dst_para.width = new_vf->width;
    ge2d_config->dst_para.height = new_vf->height;

    if(ge2d_context_config_ex(context,ge2d_config)<0) {
        printk("++ge2d configing error.\n");
        vfq_push(&q_free, new_vf);
        return;
    }

    stretchblt_noalpha(context,0,0,vf->width,vf->height,0,0,new_vf->width,new_vf->height);

    ppmgr_vf_put_dec(vf);

    vfq_push(&q_ready, new_vf);

#if DDD
    printk("rotate avail=%d, free=%d\n", vfq_level(&q_ready),  vfq_level(&q_free));
#endif
}

static struct task_struct *task=NULL;

static int ppmgr_task(void *data)
{
    struct sched_param param = {.sched_priority = MAX_RT_PRIO - 1 };
    ge2d_context_t *context=create_ge2d_work_queue();
    config_para_ex_t ge2d_config;
    memset(&ge2d_config,0,sizeof(config_para_ex_t));

    sched_setscheduler(current, SCHED_FIFO, &param);
    allow_signal(SIGTERM);

    while (down_interruptible(&thread_sem) == 0) {
        if (kthread_should_stop())
            break;

        /* process when we have both input and output space */
        while (ppmgr_vf_peek_dec() && (!vfq_empty(&q_free)) && (!ppmgr_blocking)) {
            process_vf_rotate(ppmgr_vf_get_dec(), context, &ge2d_config);
        }
        
        if (ppmgr_blocking) {
            vf_light_unreg_provider();
            vf_local_init();
            vf_reg_provider(&ppmgr_vf_provider);
            ppmgr_blocking = false;
            up(&thread_sem);
            printk("ppmgr rebuild from light-unregister\n");
        }

#if DDD
        printk("process paused, dec %p, free %d, avail %d\n",
            ppmgr_vf_peek_dec(), vfq_level(&q_free), vfq_level(&q_ready));
#endif
    }

    destroy_ge2d_work_queue(context);

    return 0;
}

/************************************************
*
*   init functions.
*
*************************************************/
static int vout_notify_callback(struct notifier_block *block, unsigned long cmd , void *para)
{
    if (cmd == VOUT_EVENT_MODE_CHANGE)
        ppmgr_device.vinfo = get_current_vinfo();

    return 0;
}

static struct notifier_block vout_notifier = 
{
    .notifier_call  = vout_notify_callback,
};

int ppmgr_buffer_init(void)
{
    int i;
    u32 canvas_width, canvas_height;
    u32 decbuf_size;
    char* buf_start;
    int buf_size;

    get_ppmgr_buf_info(&buf_start,&buf_size);
    vf_reg_provider(&ppmgr_vf_provider);

    vout_register_client(&vout_notifier);
    ppmgr_device.vinfo = get_current_vinfo();

    if (ppmgr_device.disp_width == 0)
        ppmgr_device.disp_width = ppmgr_device.vinfo->width;

    if (ppmgr_device.disp_height == 0)
        ppmgr_device.disp_height = ppmgr_device.vinfo->height;

    canvas_width = (ppmgr_device.disp_width +0xff) & ~0xff;
    canvas_height = (ppmgr_device.disp_height+0xff) & ~0xff;
    decbuf_size = canvas_width * canvas_height * 3;

    if(decbuf_size*VF_POOL_SIZE>buf_size) {
        amlog_level(LOG_LEVEL_HIGH, "size of ppmgr memory resource too small.\n");
        return -1;
    }

    for (i = 0; i < VF_POOL_SIZE; i++) {
        canvas_config(PPMGR_CANVAS_INDEX + i,
                      (ulong)(buf_start + i * decbuf_size),
                      canvas_width*3, canvas_height,
                      CANVAS_ADDR_NOWRAP, CANVAS_BLKMODE_32X32);
    }

    return 0;

}

int start_vpp_task(void)
{
    if (!task) {
        vf_local_init();
        task = kthread_run(ppmgr_task, 0, "ppmgr");
    }

    return 0;
}

void stop_vpp_task(void)
{
    if (task) {
        send_sig(SIGTERM, task, 1);
        kthread_stop(task);
        task = NULL;
    }
     
    vf_local_init();
}
