/*
 * Amlogic M3
 * frame buffer driver  -------viu input
 * Copyright (C) 2010 Amlogic, Inc.
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
 */

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/etherdevice.h>
#include <linux/interrupt.h>
#include <linux/timer.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>
#include <linux/dma-mapping.h>
#include <asm/delay.h>
#include <asm/atomic.h>

#include <linux/amports/amstream.h>
#include <linux/amports/ptsserv.h>
#include <linux/amports/canvas.h>
#include <linux/amports/vframe.h>
#include <linux/amports/vframe_provider.h>
#include <linux/tvin/tvin.h>
#include <mach/am_regs.h>

#include "tvin_global.h"
#include "vdin_regs.h"
#include "tvin_notifier.h"
#include "vdin.h"
#include "vdin_ctl.h"
#include "tvin_format_table.h"


#define DEVICE_NAME "amvdec_viuin"
#define MODULE_NAME "amvdec_viuin"
#define viuin_IRQ_NAME "amvdec_viuin-irq"


//#define viuin_COUNT             1
//#define VIUIN_ANCI_DATA_SIZE        0x4000


typedef struct {
    unsigned char       rd_canvas_index;
    unsigned char       wr_canvas_index;
    unsigned char       buff_flag[VIUIN_VF_POOL_SIZE];

    unsigned char       fmt_check_cnt;
    unsigned char       watch_dog;


    unsigned        pbufAddr;
    unsigned        decbuf_size;


    unsigned        active_pixel;
    unsigned        active_line;
    unsigned        dec_status : 1;
    unsigned        wrap_flag : 1;
    unsigned        canvas_total_count : 4;
    struct tvin_parm_s para ;



}amviuin_t;


static struct vdin_dev_s *vdin_devp_viu = NULL;


amviuin_t amviuin_dec_info = {
    .pbufAddr = 0x81000000,
    .decbuf_size = 0x70000,
    .active_pixel = 720,
    .active_line = 288,
    .watch_dog = 0,
    .para = {
        .port       = TVIN_PORT_NULL,
		.fmt_info.fmt = TVIN_SIG_FMT_NULL,
        .fmt_info.v_active=0,
        .fmt_info.h_active=0,
        .fmt_info.vsync_phase=0,
        .fmt_info.hsync_phase=0,        
        .fmt_info.frame_rate=0,
        .status     = TVIN_SIG_STATUS_NULL,
        .cap_addr   = 0,
        .flag       = 0,        //TVIN_PARM_FLAG_CAP
        .cap_size   = 0,
        .canvas_index   = 0,
     },

    .rd_canvas_index = 0xff - (VIUIN_VF_POOL_SIZE + 2),
    .wr_canvas_index =  0,

    .buff_flag = {
        VIDTYPE_INTERLACE_BOTTOM,
        VIDTYPE_INTERLACE_TOP,
        VIDTYPE_INTERLACE_BOTTOM,
        VIDTYPE_INTERLACE_TOP,
        VIDTYPE_INTERLACE_BOTTOM,
        VIDTYPE_INTERLACE_TOP},

    .fmt_check_cnt = 0,
    .dec_status = 0,
    .wrap_flag = 0,
    .canvas_total_count = VIUIN_VF_POOL_SIZE,
};


#ifdef HANDLE_viuin_IRQ
static const char viuin_dec_id[] = "viuin-dev";
#endif


static void reset_viuin_dec_parameter(void)
{
    amviuin_dec_info.wrap_flag = 0;
    amviuin_dec_info.rd_canvas_index = 0xff - (amviuin_dec_info.canvas_total_count + 2);
    amviuin_dec_info.wr_canvas_index =  0;
    amviuin_dec_info.fmt_check_cnt = 0;
    amviuin_dec_info.watch_dog = 0;

}

static void init_viuin_dec_parameter(fmt_info_t fmt_info)
{
    amviuin_dec_info.para.fmt_info.fmt= fmt_info.fmt;
	amviuin_dec_info.para.fmt_info.frame_rate=fmt_info.frame_rate;
    switch(fmt_info.fmt)
    {
		case TVIN_SIG_FMT_MAX+1:
		case 0xFFFF:     //default camera
            amviuin_dec_info.active_pixel = fmt_info.h_active;
            amviuin_dec_info.active_line = fmt_info.v_active;
			amviuin_dec_info.para.fmt_info.h_active = fmt_info.h_active;
            amviuin_dec_info.para.fmt_info.v_active = fmt_info.v_active;
            amviuin_dec_info.para.fmt_info.hsync_phase = fmt_info.hsync_phase;
            amviuin_dec_info.para.fmt_info.vsync_phase = fmt_info.vsync_phase;
            pr_dbg("%dx%d input mode is selected for camera, \n",fmt_info.h_active,fmt_info.v_active);
            break;
        case TVIN_SIG_FMT_NULL:
        default:     
            amviuin_dec_info.active_pixel = 0;
            amviuin_dec_info.active_line = 0;               
            break;
    }

    return;
}

static int viu_in_canvas_init(unsigned int mem_start, unsigned int mem_size)
{
    int i, canvas_start_index ;
    unsigned int canvas_width  = 1600 << 1;
    unsigned int canvas_height = 1200;
    unsigned decbuf_start = mem_start + VIUIN_ANCI_DATA_SIZE;
    amviuin_dec_info.decbuf_size   = 0x400000;

    i = (unsigned)((mem_size - VIUIN_ANCI_DATA_SIZE) / amviuin_dec_info.decbuf_size);

    amviuin_dec_info.canvas_total_count = (VIUIN_VF_POOL_SIZE > i)? i : VIUIN_VF_POOL_SIZE;

    amviuin_dec_info.pbufAddr  = mem_start;

    if(vdin_devp_viu->index )
        canvas_start_index = tvin_canvas_tab[1][0];
    else
        canvas_start_index = tvin_canvas_tab[0][0];

    for ( i = 0; i < amviuin_dec_info.canvas_total_count; i++)
    {
        canvas_config(canvas_start_index + i, decbuf_start + i * amviuin_dec_info.decbuf_size,
            canvas_width, canvas_height, CANVAS_ADDR_NOWRAP, CANVAS_BLKMODE_LINEAR);
    }
    set_tvin_canvas_info(canvas_start_index ,amviuin_dec_info.canvas_total_count );
    return 0;
}


static void viu_release_canvas(void)
{
//    int i = 0;

//    for(;i < amviuin_dec_info.canvas_total_count; i++)
//    {
//        canvas_release(VDIN_START_CANVAS + i);
//    }
    return;
}


static inline u32 viu_index2canvas(u32 index)
{
    int tv_group_index ;

    if(vdin_devp_viu->index )
        tv_group_index = 1;
    else
        tv_group_index = 0;

    if(index < amviuin_dec_info.canvas_total_count)
        return tvin_canvas_tab[tv_group_index][index];
    else
        return 0xff;
}


static void start_amvdec_viu_in(unsigned int mem_start, unsigned int mem_size,
                        tvin_port_t port, fmt_info_t fmt_info)
{

    if(amviuin_dec_info.dec_status != 0)
    {
        pr_dbg("viu_in is processing, don't do starting operation \n");
        return;
    }

    pr_dbg("start ");
    if(port == TVIN_PORT_VIU_ENCT)
    {
        pr_dbg("viu in decode. \n");
        viu_in_canvas_init(mem_start, mem_size);
        init_viuin_dec_parameter(fmt_info);
        reset_viuin_dec_parameter();
        WRITE_CBUS_REG_BITS(VPU_VIU_VENC_MUX_CTRL, 4, 4, 4); //select ENCT
        WRITE_CBUS_REG_BITS(VPU_VIU_VENC_MUX_CTRL, 4, 8, 4); //select ENCT

        amviuin_dec_info.para.port = TVIN_PORT_VIU_ENCT;
		    amviuin_dec_info.wr_canvas_index = 0xff;
        amviuin_dec_info.dec_status = 1;
    }
    return;
}

static void stop_amvdec_viu_in(void)
{
    if(amviuin_dec_info.dec_status)
    {
        pr_dbg("stop viu_in decode. \n");

        viu_release_canvas();

        amviuin_dec_info.para.fmt_info.fmt= TVIN_SIG_FMT_NULL;
        amviuin_dec_info.dec_status = 0;

    }
    else
    {
         pr_dbg("viu_in is not started yet. \n");
    }

    return;
}

static void viu_send_buff_to_display_fifo(vframe_t * info)
{
    if(amviuin_dec_info.para.fmt_info.fmt==TVIN_SIG_FMT_MAX+1)
        info->duration = 960000/amviuin_dec_info.para.fmt_info.frame_rate;
    else
        info->duration = tvin_fmt_tbl[amviuin_dec_info.para.fmt_info.fmt].duration;

    //info->duration = tvin_fmt_tbl[amviuin_dec_info.para.fmt].duration;
    info->type = VIDTYPE_VIU_SINGLE_PLANE | VIDTYPE_VIU_422 | VIDTYPE_VIU_FIELD | VIDTYPE_PROGRESSIVE ;
    info->width= amviuin_dec_info.active_pixel;
    info->height = amviuin_dec_info.active_line;
    if(amviuin_dec_info.para.fmt_info.fmt==TVIN_SIG_FMT_MAX+1)
        info->duration = 960000/amviuin_dec_info.para.fmt_info.frame_rate;
    else
        info->duration = tvin_fmt_tbl[amviuin_dec_info.para.fmt_info.fmt].duration;
    //info->duration = tvin_fmt_tbl[amviuin_dec_info.para.fmt].duration;
    info->canvas0Addr = info->canvas1Addr = viu_index2canvas(amviuin_dec_info.wr_canvas_index);
}


static void viuin_wr_vdin_canvas(void)
{
    unsigned char canvas_id;
    amviuin_dec_info.wr_canvas_index += 1;
    if(amviuin_dec_info.wr_canvas_index > (amviuin_dec_info.canvas_total_count -1) )
    {
        amviuin_dec_info.wr_canvas_index = 0;
        amviuin_dec_info.wrap_flag = 1;
    }
    canvas_id = viu_index2canvas(amviuin_dec_info.wr_canvas_index);

    vdin_set_canvas_id(vdin_devp_viu->index, canvas_id);
}

static void viu_wr_vdin_canvas(int index)
{
	vdin_set_canvas_id(vdin_devp_viu->index, viu_index2canvas(index));
}

static void viu_in_dec_run(vframe_t * info)
{
	if (amviuin_dec_info.wr_canvas_index == 0xff) {
		viu_wr_vdin_canvas(0);
		amviuin_dec_info.wr_canvas_index = 0;
		return;
	}
	
	viu_send_buff_to_display_fifo(info);

	amviuin_dec_info.wr_canvas_index++;
	if (amviuin_dec_info.wr_canvas_index >= amviuin_dec_info.canvas_total_count)
		amviuin_dec_info.wr_canvas_index = 0;
		
	viu_wr_vdin_canvas(amviuin_dec_info.wr_canvas_index);

    return;
}

/*as use the spin_lock,
 *1--there is no sleep,
 *2--it is better to shorter the time,
*/
int amvdec_viu_in_run(vframe_t *info)
{
    unsigned ccir656_status;
    unsigned char canvas_id = 0;
    unsigned char last_receiver_buff_index = 0;

    if(amviuin_dec_info.dec_status == 0){
//        pr_error("viuin decoder is not started\n");
        return -1;
    }
    
    if(amviuin_dec_info.active_pixel == 0){
    	return -1;	
    }
    amviuin_dec_info.watch_dog = 0;

    if(vdin_devp_viu->para.flag & TVIN_PARM_FLAG_CAP)
    {
        if(amviuin_dec_info.wr_canvas_index == 0)
              last_receiver_buff_index = amviuin_dec_info.canvas_total_count - 1;
        else
              last_receiver_buff_index = amviuin_dec_info.wr_canvas_index - 1;

        if(last_receiver_buff_index != amviuin_dec_info.rd_canvas_index)
            canvas_id = last_receiver_buff_index;
        else
        {
            if(amviuin_dec_info.wr_canvas_index == amviuin_dec_info.canvas_total_count - 1)
                  canvas_id = 0;
            else
                  canvas_id = amviuin_dec_info.wr_canvas_index + 1;
        }
        vdin_devp_viu->para.cap_addr = amviuin_dec_info.pbufAddr +
                (amviuin_dec_info.decbuf_size * canvas_id) + VIUIN_ANCI_DATA_SIZE ;
        vdin_devp_viu->para.cap_size = amviuin_dec_info.decbuf_size;
        vdin_devp_viu->para.canvas_index =VDIN_START_CANVAS+ canvas_id;
        return 0;
    }

    if(amviuin_dec_info.para.port == TVIN_PORT_VIU_ENCT)
    {
        viu_in_dec_run(info);
    }
    return 0;
}


static int viuin_check_callback(struct notifier_block *block, unsigned long cmd , void *para)
{
    int ret = 0;
    switch(cmd)
    {
        case TVIN_EVENT_INFO_CHECK:
            break;
        default:
            break;
    }
    return ret;
}


struct tvin_dec_ops_s viuin_op = {
        .dec_run = amvdec_viu_in_run,        
    };


static struct notifier_block viuin_check_notifier = {
    .notifier_call  = viuin_check_callback,
};


static int viuin_notifier_callback(struct notifier_block *block, unsigned long cmd , void *para)
{
    int ret = 0;
    vdin_dev_t *p = NULL;
    switch(cmd)
    {
        case TVIN_EVENT_DEC_START:
            if(para != NULL)
            {
                p = (vdin_dev_t*)para;
                if (p->para.port != TVIN_PORT_VIU_ENCT)
                {
                    pr_info("viuin: ignore TVIN_EVENT_DEC_START (port=%d)\n", p->para.port);
                    return ret;
                }
                pr_dbg("viuin_notifier_callback, para = %x ,mem_start = %x, port = %d, format = %d, ca-_flag = %d.\n" ,
                        (unsigned int)para, p->mem_start, p->para.port, p->para.fmt_info.fmt, p->para.flag);
                vdin_devp_viu = p;
                start_amvdec_viu_in(p->mem_start,p->mem_size, p->para.port, p->para.fmt_info);
                amviuin_dec_info.para.status = TVIN_SIG_STATUS_STABLE;
                vdin_info_update(p, &amviuin_dec_info.para);
                tvin_dec_register(p, &viuin_op);
                tvin_check_notifier_register(&viuin_check_notifier);
                ret = NOTIFY_STOP_MASK;
            }
            break;

        case TVIN_EVENT_DEC_STOP:
            if(para != NULL)
            {
                p = (vdin_dev_t*)para;
                if(p->para.port != TVIN_PORT_VIU_ENCT)
                {
                    pr_info("viuin: ignore TVIN_EVENT_DEC_STOP, port=%d \n", p->para.port);
                    return ret;
                }
                stop_amvdec_viu_in();
                tvin_dec_unregister(vdin_devp_viu);
                tvin_check_notifier_unregister(&viuin_check_notifier);
                vdin_devp_viu = NULL;
                ret = NOTIFY_STOP_MASK;
            }
            break;

        default:
            break;
    }
    return ret;
}


static struct notifier_block viuin_notifier = {
    .notifier_call  = viuin_notifier_callback,
};

static int amvdec_viuin_probe(struct platform_device *pdev)
{
    int r = 0;

    pr_dbg("amvdec_viuin probe start.\n");

    tvin_dec_notifier_register(&viuin_notifier);

    pr_dbg("amvdec_viuin probe end.\n");
    return r;
}

static int amvdec_viuin_remove(struct platform_device *pdev)
{
    /* Remove the cdev */
    tvin_dec_notifier_unregister(&viuin_notifier);
    return 0;
}



static struct platform_driver amvdec_viuin_driver = {
    .probe      = amvdec_viuin_probe,
    .remove     = amvdec_viuin_remove,
    .driver     = {
        .name   = DEVICE_NAME,
    }
};

static struct platform_device* viuin_device = NULL;

static int __init amvdec_viuin_init_module(void)
{
    pr_dbg("amvdec_viuin module init\n");
#if 1
	  viuin_device = platform_device_alloc(DEVICE_NAME,0);
    if (!viuin_device) {
        pr_error("failed to alloc viuin_device\n");
        return -ENOMEM;
    }

    if(platform_device_add(viuin_device)){
        platform_device_put(viuin_device);
        pr_error("failed to add viuin_device\n");
        return -ENODEV;
    }
    if (platform_driver_register(&amvdec_viuin_driver)) {
        pr_error("failed to register amvdec_viuin driver\n");

        platform_device_del(viuin_device);
        platform_device_put(viuin_device);
        return -ENODEV;
    }

#else
    if (platform_driver_register(&amvdec_viuin_driver)) {
        pr_error("failed to register amvdec_viuin driver\n");
        return -ENODEV;
    }
#endif
    return 0;
}

static void __exit amvdec_viuin_exit_module(void)
{
    pr_dbg("amvdec_viuin module remove.\n");
    platform_driver_unregister(&amvdec_viuin_driver);
    return ;
}



module_init(amvdec_viuin_init_module);
module_exit(amvdec_viuin_exit_module);


