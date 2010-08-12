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

#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>
#include <linux/fs.h>
#include <mach/am_regs.h>

#include <linux/string.h>
#include <linux/io.h>
#include <linux/mm.h>
#include <linux/major.h>
#include <linux/err.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/ctype.h>
#include <linux/amports/ptsserv.h>
#include <linux/amports/timestamp.h>
#include <linux/amports/tsync.h>
#include <linux/amports/canvas.h>
#include <linux/amports/amstream.h>
#include <linux/vout/vout_notify.h>
#include <linux/sched.h>
#include <linux/poll.h>

#include <asm/fiq.h>
#include <asm/uaccess.h>

#include "vframe.h"
#include "vframe_provider.h"
#include "video.h"
#include "vpp.h"

#include "deinterlace.h"

#define DRIVER_NAME "amvideo"
#define MODULE_NAME "amvideo"
#define DEVICE_NAME "amvideo"

//#define FIQ_VSYNC 

//#define SLOW_SYNC_REPEAT
#define DEBUG

#define BRIDGE_IRQ	INT_TIMER_D
#define BRIDGE_IRQ_SET() WRITE_CBUS_REG(ISA_TIMERD, 1)

#define RESERVE_CLR_FRAME

#define pr_dbg(fmt, args...) printk(KERN_DEBUG "apollo video: " fmt, ## args)
#define pr_error(fmt, args...) printk(KERN_ERR "apollo video: " fmt, ## args)

#define EnableVideoLayer()  \
    do { SET_MPEG_REG_MASK(VPP_MISC, \
         VPP_VD1_PREBLEND | VPP_PREBLEND_EN | VPP_VD1_POSTBLEND); \
    } while (0)

#define DisableVideoLayer() \
    do { CLEAR_MPEG_REG_MASK(VPP_MISC, \
         VPP_VD1_PREBLEND | VPP_VD1_POSTBLEND); \
    } while (0)

/*********************************************************/

#define VOUT_TYPE_TOP_FIELD 0
#define VOUT_TYPE_BOT_FIELD 1
#define VOUT_TYPE_PROG      2

#define DUR2PTS(x) ((x) - ((x) >> 4))
#define DUR2PTS_RM(x) ((x) & 0xf)

const char video_dev_id[] = "amvideo-dev";


static spinlock_t lock = SPIN_LOCK_UNLOCKED;
static u32 frame_par_ready_to_set, frame_par_force_to_set;
static u32 vpts_remainder;
static bool video_property_changed = false;

/* display canvas */
static u32 disp_canvas_index[3] = {
    DISPLAY_CANVAS_BASE_INDEX,
    DISPLAY_CANVAS_BASE_INDEX+1,
    DISPLAY_CANVAS_BASE_INDEX+2};
static u32 disp_canvas;
static u32 post_canvas = 0;
static ulong keep_y_addr = 0, *keep_y_addr_remap = NULL;
static ulong keep_u_addr = 0, *keep_u_addr_remap = NULL;
static ulong keep_v_addr = 0, *keep_v_addr_remap = NULL;
#define Y_BUFFER_SIZE   0x200000 // for 1920*1088
#define U_BUFFER_SIZE   0x80000
#define V_BUFFER_SIZE   0x80000

/* zoom information */
static u32 zoom_start_x_lines;
static u32 zoom_end_x_lines;
static u32 zoom_start_y_lines;
static u32 zoom_end_y_lines;

/* wide settings */
static u32 wide_setting; // 1  fill content .

/* black out policy */
#if defined(CONFIG_JPEGLOGO)
static u32 blackout = 0;
#else
static u32 blackout = 1;
#endif

/* disable video */
static u32 disable_video = 0;

#ifdef SLOW_SYNC_REPEAT
/* video frame repeat count */
static u32 frame_repeat_count = 0;
#endif

/* vout */
static const vinfo_t *vinfo = NULL;

/* config */
static vframe_t *cur_dispbuf = NULL;
static vframe_t vf_local;
static u32 vsync_pts_inc;

static vpp_frame_par_t *cur_frame_par, *next_frame_par;
static vpp_frame_par_t frame_parms[2];

/* vsync pass flag */
static u32 wait_sync;
static const vframe_provider_t *vfp;

// 0 - off
// 1 - pre-post link
// 2 - pre-post separate, only post in vsync
static int deinterlace_mode = 0;

/* trickmode i frame*/
u32 trickmode_i = 0;

/* trickmode ff/fb */
u32 trickmode_fffb = 0;
atomic_t trickmode_framedone = ATOMIC_INIT(0);

static const f2v_vphase_type_t vpp_phase_table[4][3] = {
    {F2V_P2IT,  F2V_P2IB,  F2V_P2P },   /* VIDTYPE_PROGRESSIVE */
    {F2V_IT2IT, F2V_IT2IB, F2V_IT2P},   /* VIDTYPE_INTERLACE_TOP */
    {F2V_P2IT,  F2V_P2IB,  F2V_P2P },
    {F2V_IB2IT, F2V_IB2IB, F2V_IB2P}    /* VIDTYPE_INTERLACE_BOTTOM */
};

static const u8 skip_tab[6] = { 0x24, 0x04, 0x68, 0x48, 0x28, 0x08 };
/* wait queue for poll */
static wait_queue_head_t amvideo_trick_wait;

int get_deinterlace_mode(void)
{
	return deinterlace_mode;
}

const vframe_provider_t * get_vfp(void)
{
	return vfp;
}

/*********************************************************/
static inline vframe_t *vf_peek(void)
{
	if ( deinterlace_mode == 2 )
	{
		return peek_di_out_buf();
	}
	else
	{
    	if (vfp)
        	return vfp->peek();
	}

    return NULL;
}

static inline vframe_t *vf_get(void)
{
	if ( deinterlace_mode == 2 )
	{
		vframe_t *disp_buf = peek_di_out_buf();

		if ( disp_buf )
		{
	    	if ( (disp_buf->duration > 0)
#ifdef DI_SD_ONLY
				&& (disp_buf->width <= 720)
#endif
	    		)
			{
				set_post_di_mem(disp_buf->blend_mode);
			}

		   	inc_field_counter();
		}

		return disp_buf;
	}
	else
	{
    	if (vfp)
        	return vfp->get();
	}

    return NULL;
}

static inline void vf_put(vframe_t *vf)
{
	if ( deinterlace_mode == 2 )
	{
		if ( vfp && (vf->recycle_by_di_pre == 0) )
        	vfp->put(vf);
	}
	else
	{
    	if (vfp)
        	vfp->put(vf);
	}
}

static void vpp_settings_h(vpp_frame_par_t *framePtr)
{
    vppfilter_mode_t *vpp_filter = &framePtr->vpp_filter;
    u32 r1, r2, r3;

    r1 = framePtr->VPP_hsc_linear_startp - framePtr->VPP_hsc_startp;
    r2 = framePtr->VPP_hsc_linear_endp   - framePtr->VPP_hsc_startp;
    r3 = framePtr->VPP_hsc_endp          - framePtr->VPP_hsc_startp;

    WRITE_MPEG_REG(VPP_POSTBLEND_VD1_H_START_END,
        ((framePtr->VPP_hsc_startp & VPP_VD_SIZE_MASK) << VPP_VD1_START_BIT) |
        ((framePtr->VPP_hsc_endp   & VPP_VD_SIZE_MASK) << VPP_VD1_END_BIT));

    WRITE_MPEG_REG(VPP_HSC_REGION12_STARTP,
                   (0 << VPP_REGION1_BIT) |
                   ((r1 & VPP_REGION_MASK) << VPP_REGION2_BIT));

    WRITE_MPEG_REG(VPP_HSC_REGION34_STARTP,
                   ((r2 & VPP_REGION_MASK) << VPP_REGION3_BIT) |
                   ((r3 & VPP_REGION_MASK) << VPP_REGION4_BIT));
    WRITE_MPEG_REG(VPP_HSC_REGION4_ENDP, r3);

    WRITE_MPEG_REG(VPP_HSC_START_PHASE_STEP,
                   vpp_filter->vpp_hf_start_phase_step);

    WRITE_MPEG_REG(VPP_LINE_IN_LENGTH, framePtr->VPP_line_in_length_);
    WRITE_MPEG_REG(VPP_PREBLEND_H_SIZE, framePtr->VPP_line_in_length_);
}

static void vpp_settings_v(vpp_frame_par_t *framePtr)
{
    vppfilter_mode_t *vpp_filter = &framePtr->vpp_filter;
    u32 r;

    r = framePtr->VPP_vsc_endp - framePtr->VPP_vsc_startp;

    WRITE_MPEG_REG(VPP_POSTBLEND_VD1_V_START_END,
        ((framePtr->VPP_vsc_startp & VPP_VD_SIZE_MASK) << VPP_VD1_START_BIT) |
        ((framePtr->VPP_vsc_endp   & VPP_VD_SIZE_MASK) << VPP_VD1_END_BIT));

    WRITE_MPEG_REG(VPP_VSC_REGION12_STARTP, 0);
    WRITE_MPEG_REG(VPP_VSC_REGION34_STARTP,
                   ((r & VPP_REGION_MASK) << VPP_REGION3_BIT) |
                   ((r & VPP_REGION_MASK) << VPP_REGION4_BIT));
    WRITE_MPEG_REG(VPP_VSC_REGION4_ENDP, r);

    WRITE_MPEG_REG(VPP_VSC_START_PHASE_STEP,
                   vpp_filter->vpp_vsc_start_phase_step);
}

static void zoom_display_horz(void)
{
    WRITE_MPEG_REG(VD1_IF0_LUMA_X0,
               (zoom_start_x_lines << VDIF_PIC_START_BIT) |
               (zoom_end_x_lines   << VDIF_PIC_END_BIT) );

    WRITE_MPEG_REG(VD1_IF0_CHROMA_X0,
               (zoom_start_x_lines/2 << VDIF_PIC_START_BIT) |
               (zoom_end_x_lines/2   << VDIF_PIC_END_BIT) );

    WRITE_MPEG_REG(VD1_IF0_LUMA_X1,
               (zoom_start_x_lines << VDIF_PIC_START_BIT) |
               (zoom_end_x_lines   << VDIF_PIC_END_BIT) );

    WRITE_MPEG_REG(VD1_IF0_CHROMA_X1,
               (zoom_start_x_lines/2 << VDIF_PIC_START_BIT) |
               (zoom_end_x_lines/2   << VDIF_PIC_END_BIT) );

    WRITE_MPEG_REG(VIU_VD1_FMT_W,
        ((zoom_end_x_lines - zoom_start_x_lines + 1) << VD1_FMT_LUMA_WIDTH_BIT) |
         ((zoom_end_x_lines/2 - zoom_start_x_lines/2 + 1) << VD1_FMT_CHROMA_WIDTH_BIT));
}

static void zoom_display_vert(void)
{
    WRITE_MPEG_REG(VD1_IF0_LUMA_Y0,
                   (zoom_start_y_lines << VDIF_PIC_START_BIT) |
                   (zoom_end_y_lines   << VDIF_PIC_END_BIT));

    WRITE_MPEG_REG(VD1_IF0_CHROMA_Y0,
                   ((zoom_start_y_lines/2) << VDIF_PIC_START_BIT) |
                   ((zoom_end_y_lines/2)   << VDIF_PIC_END_BIT));

    WRITE_MPEG_REG(VD1_IF0_LUMA_Y1,
                   (zoom_start_y_lines << VDIF_PIC_START_BIT) |
                   (zoom_end_y_lines << VDIF_PIC_END_BIT));

    WRITE_MPEG_REG(VD1_IF0_CHROMA_Y1,
                   ((zoom_start_y_lines / 2) << VDIF_PIC_START_BIT) |
                   ((zoom_end_y_lines / 2) << VDIF_PIC_END_BIT));
}

static void vsync_toggle_frame(vframe_t *vf)
{
    u32 first_picture = 0;

    if ((vf->width == 0) && (vf->height == 0)) {
        printk("Video: invalid frame dimension\n");
        return;
    }

    if ((cur_dispbuf) && (cur_dispbuf != &vf_local) && (cur_dispbuf != vf)) {
        vf_put(cur_dispbuf);

    } else {
        first_picture = 1;
    }

    if (video_property_changed) {
        video_property_changed = false;
        first_picture = 1;
    }

    /* switch buffer */
    post_canvas = vf->canvas0Addr;
    canvas_copy(vf->canvas0Addr & 0xff, disp_canvas_index[0]);
    canvas_copy((vf->canvas0Addr >> 8) & 0xff, disp_canvas_index[1]);
    canvas_copy((vf->canvas0Addr >> 16)& 0xff, disp_canvas_index[2]);

    WRITE_MPEG_REG(VD1_IF0_CANVAS0, disp_canvas);
    WRITE_MPEG_REG(VD1_IF0_CANVAS1, disp_canvas);
    WRITE_MPEG_REG(VD2_IF0_CANVAS0, disp_canvas);
    WRITE_MPEG_REG(VD2_IF0_CANVAS1, disp_canvas);

    /* set video PTS */
	if (cur_dispbuf != vf) {
	    if (vf->pts != 0) {
#ifdef DEBUG
        pr_dbg("vpts to vf->pts: 0x%x, scr: 0x%x, abs_scr: 0x%x\n",
            vf->pts, timestamp_pcrscr_get(), READ_MPEG_REG(SCR_HIU));
#endif
        timestamp_vpts_set(vf->pts);
    	}
    	else if (cur_dispbuf) {
#ifdef DEBUG
        	pr_dbg("vpts inc: 0x%x, scr: 0x%x, abs_scr: 0x%x\n",
            	timestamp_vpts_get() + DUR2PTS(cur_dispbuf->duration),
            	timestamp_pcrscr_get(), READ_MPEG_REG(SCR_HIU));
#endif
        	timestamp_vpts_inc(DUR2PTS(cur_dispbuf->duration));

        	vpts_remainder += DUR2PTS_RM(cur_dispbuf->duration);
        	if (vpts_remainder >= 0xf) {
            	vpts_remainder -= 0xf;
            	timestamp_vpts_inc(-1);
        	}
    	}
    }

	vf->type_backup = vf->type;

    /* enable new config on the new frames */
    if ((first_picture) ||
        (cur_dispbuf->bufWidth != vf->bufWidth) ||
        (cur_dispbuf->width != vf->width) ||
        (cur_dispbuf->height != vf->height) ||
        (cur_dispbuf->ratio_control != vf->ratio_control) ||
        ((cur_dispbuf->type_backup & VIDTYPE_INTERLACE) !=
         (vf->type_backup & VIDTYPE_INTERLACE))) {
#ifdef DEBUG
        pr_dbg("%s %dx%d ar=0x%x\n",
               ((vf->type & VIDTYPE_TYPEMASK) == VIDTYPE_INTERLACE_TOP) ?
               "interlace-top" :
               ((vf->type & VIDTYPE_TYPEMASK) == VIDTYPE_INTERLACE_BOTTOM) ?
               "interlace-bottom" :
               "progressive",
               vf->width,
               vf->height,
               vf->ratio_control);
#endif
        next_frame_par = (&frame_parms[0] == next_frame_par) ?
            &frame_parms[1] : &frame_parms[0];

        if ( (deinterlace_mode != 0) 
        	&& (vf->type & VIDTYPE_INTERLACE)
#ifdef DI_SD_ONLY
            && (vf->width <= 720)
#endif
                ) 
        {
            vf->type &= ~VIDTYPE_TYPEMASK;

           	if ( deinterlace_mode == 1 )
           	{
            	inc_field_counter();
           	}
        } 

        vpp_set_filters(wide_setting, vf, next_frame_par, vinfo);

        /* apply new vpp settings */
        frame_par_ready_to_set = 1;
    }
    else
    {
        if ( (deinterlace_mode != 0) 
        	&& (vf->type & VIDTYPE_INTERLACE)
#ifdef DI_SD_ONLY
            && (vf->width <= 720)
#endif
                ) 
		{
            vf->type &= ~VIDTYPE_TYPEMASK;

           	if ( deinterlace_mode == 1 )
           	{
            	inc_field_counter();
           	}
		}
    }

    cur_dispbuf = vf;

    if (first_picture && (disable_video==0)) {
        EnableVideoLayer();

        frame_par_ready_to_set = 1;

        if ( deinterlace_mode == 1 )
    		disable_deinterlace();
        else if ( deinterlace_mode == 2 )
        	disable_post_deinterlace();
    }
}

static void viu_set_dcu(vpp_frame_par_t *frame_par, vframe_t *vf)
{
    u32 r;

    r = (3 << VDIF_URGENT_BIT) |
        (10 << VDIF_HOLD_LINES_BIT) |
        VDIF_FORMAT_SPLIT  |
        VDIF_CHRO_RPT_LAST |
        VDIF_ENABLE;

    if ((vf->type & VIDTYPE_VIU_SINGLE_PLANE) == 0) {
        r |= VDIF_SEPARATE_EN;
    } else {
        if (vf->type & VIDTYPE_VIU_422) {
            r |= VDIF_FORMAT_422;
        } else {
            r |= VDIF_FORMAT_RGB888_YUV444;
        }
    }

    WRITE_MPEG_REG(VD1_IF0_GEN_REG, r);
    WRITE_MPEG_REG(VD2_IF0_GEN_REG, r);

    if ((vf->type & VIDTYPE_VIU_FIELD) &&
        (frame_par->VPP_prog_as_interlace == 0)) {
        WRITE_MPEG_REG(VD1_IF0_RPT_LOOP,        0);
        WRITE_MPEG_REG(VD1_IF0_LUMA0_RPT_PAT,   0);
        WRITE_MPEG_REG(VD1_IF0_CHROMA0_RPT_PAT, 0);
        WRITE_MPEG_REG(VD1_IF0_LUMA1_RPT_PAT,   0);
        WRITE_MPEG_REG(VD1_IF0_CHROMA1_RPT_PAT, 0);

        WRITE_MPEG_REG(VD2_IF0_RPT_LOOP,        0);
        WRITE_MPEG_REG(VD2_IF0_LUMA0_RPT_PAT,   0);
        WRITE_MPEG_REG(VD2_IF0_CHROMA0_RPT_PAT, 0);
        WRITE_MPEG_REG(VD2_IF0_LUMA1_RPT_PAT,   0);
        WRITE_MPEG_REG(VD2_IF0_CHROMA1_RPT_PAT, 0);

        WRITE_MPEG_REG(VIU_VD1_FMT_CTRL,
            HFORMATTER_YC_RATIO_2_1 |
            HFORMATTER_EN |
            VFORMATTER_RPTLINE0_EN |
            (0xc << VFORMATTER_INIPHASE_BIT) |
            (((vf->type & VIDTYPE_VIU_422) ? 0x10 : 0x08) << VFORMATTER_PHASE_BIT) |
            VFORMATTER_EN);

        WRITE_MPEG_REG(VIU_VD2_FMT_CTRL,
            HFORMATTER_YC_RATIO_2_1 |
            HFORMATTER_EN |
            VFORMATTER_RPTLINE0_EN |
            (0xc << VFORMATTER_INIPHASE_BIT) |
            (((vf->type & VIDTYPE_VIU_422) ? 0x10 : 0x08) << VFORMATTER_PHASE_BIT) |
            VFORMATTER_EN);
    }
    else if ((vf->type & VIDTYPE_INTERLACE) ||
             (frame_par->VPP_prog_as_interlace == 1)) {
        if (((vf->type & VIDTYPE_TYPEMASK) == VIDTYPE_INTERLACE_TOP) &&
            (frame_par->VPP_prog_as_interlace == 0)) {
            WRITE_MPEG_REG(VD1_IF0_RPT_LOOP,
                (0x11 << VDIF_CHROMA_LOOP1_BIT) |
                (0x11 << VDIF_LUMA_LOOP1_BIT)   |
                (0x11 << VDIF_CHROMA_LOOP0_BIT) |
                (0x11 << VDIF_LUMA_LOOP0_BIT));
            WRITE_MPEG_REG(VD1_IF0_LUMA0_RPT_PAT,   0x80);
            WRITE_MPEG_REG(VD1_IF0_CHROMA0_RPT_PAT, 0x80);
            WRITE_MPEG_REG(VD1_IF0_LUMA1_RPT_PAT,   0x80);
            WRITE_MPEG_REG(VD1_IF0_CHROMA1_RPT_PAT, 0x80);

            WRITE_MPEG_REG(VD2_IF0_RPT_LOOP,
                (0x11 << VDIF_CHROMA_LOOP1_BIT) |
                (0x11 << VDIF_LUMA_LOOP1_BIT)   |
                (0x11 << VDIF_CHROMA_LOOP0_BIT) |
                (0x11 << VDIF_LUMA_LOOP0_BIT));
            WRITE_MPEG_REG(VD2_IF0_LUMA0_RPT_PAT,   0x80);
            WRITE_MPEG_REG(VD2_IF0_CHROMA0_RPT_PAT, 0x80);
            WRITE_MPEG_REG(VD2_IF0_LUMA1_RPT_PAT,   0x80);
            WRITE_MPEG_REG(VD2_IF0_CHROMA1_RPT_PAT, 0x80);

            WRITE_MPEG_REG(VIU_VD1_FMT_CTRL,
                HFORMATTER_YC_RATIO_2_1 |
                HFORMATTER_EN |
                VFORMATTER_RPTLINE0_EN |
                (0xe << VFORMATTER_INIPHASE_BIT) |
                (((vf->type & VIDTYPE_VIU_422) ? 0x10 : 0x08) << VFORMATTER_PHASE_BIT) |
                VFORMATTER_EN);

            WRITE_MPEG_REG(VIU_VD2_FMT_CTRL,
                HFORMATTER_YC_RATIO_2_1 |
                HFORMATTER_EN |
                VFORMATTER_RPTLINE0_EN |
                (0xe << VFORMATTER_INIPHASE_BIT) |
                (((vf->type & VIDTYPE_VIU_422) ? 0x10 : 0x08) << VFORMATTER_PHASE_BIT) |
                VFORMATTER_EN);
        } else {
            WRITE_MPEG_REG(VD1_IF0_RPT_LOOP,
                (0x00 << VDIF_CHROMA_LOOP1_BIT) |
                (0x00 << VDIF_LUMA_LOOP1_BIT)   |
                (0x00 << VDIF_CHROMA_LOOP0_BIT) |
                (0x00 << VDIF_LUMA_LOOP0_BIT));
            WRITE_MPEG_REG(VD1_IF0_LUMA0_RPT_PAT,   0x08);
            WRITE_MPEG_REG(VD1_IF0_CHROMA0_RPT_PAT, 0x08);
            WRITE_MPEG_REG(VD1_IF0_LUMA1_RPT_PAT,   0x08);
            WRITE_MPEG_REG(VD1_IF0_CHROMA1_RPT_PAT, 0x08);

            WRITE_MPEG_REG(VD2_IF0_RPT_LOOP,
                (0x00 << VDIF_CHROMA_LOOP1_BIT) |
                (0x00 << VDIF_LUMA_LOOP1_BIT)   |
                (0x00 << VDIF_CHROMA_LOOP0_BIT) |
                (0x00 << VDIF_LUMA_LOOP0_BIT));
            WRITE_MPEG_REG(VD2_IF0_LUMA0_RPT_PAT,   0x08);
            WRITE_MPEG_REG(VD2_IF0_CHROMA0_RPT_PAT, 0x08);
            WRITE_MPEG_REG(VD2_IF0_LUMA1_RPT_PAT,   0x08);
            WRITE_MPEG_REG(VD2_IF0_CHROMA1_RPT_PAT, 0x08);

            WRITE_MPEG_REG(VIU_VD1_FMT_CTRL,
                HFORMATTER_YC_RATIO_2_1 |
                HFORMATTER_EN |
                VFORMATTER_RPTLINE0_EN |
                (0xa << VFORMATTER_INIPHASE_BIT) |
                (((vf->type & VIDTYPE_VIU_422) ? 0x10 : 0x08) << VFORMATTER_PHASE_BIT) |
                VFORMATTER_EN);

            WRITE_MPEG_REG(VIU_VD2_FMT_CTRL,
                HFORMATTER_YC_RATIO_2_1 |
                HFORMATTER_EN |
                VFORMATTER_RPTLINE0_EN |
                (0xa << VFORMATTER_INIPHASE_BIT) |
                (((vf->type & VIDTYPE_VIU_422) ? 0x10 : 0x08) << VFORMATTER_PHASE_BIT) |
                VFORMATTER_EN);
        }
    }
    else {
        /* progressive frame in two canvases, unsupported */
    }
}

static int detect_vout_type(void)
{
#if defined(CONFIG_AM_TCON_OUTPUT)
    return VOUT_TYPE_PROG;
#else
    int vout_type;
    int encp_enable = READ_MPEG_REG(ENCP_VIDEO_EN) & 1;

    if (encp_enable) {
        if (READ_MPEG_REG(ENCP_VIDEO_MODE) & (1 << 12)) {
            /* 1080I */
            if (READ_MPEG_REG(VENC_ENCP_LINE) < 562) {
                vout_type = VOUT_TYPE_TOP_FIELD;

            } else {
                vout_type = VOUT_TYPE_BOT_FIELD;
            }

        } else {
            vout_type = VOUT_TYPE_PROG;
        }

    } else {
        vout_type = (READ_MPEG_REG(VENC_STATA) & 1) ?
            VOUT_TYPE_BOT_FIELD : VOUT_TYPE_TOP_FIELD;
    }

    return vout_type;
#endif
}

static int calc_hold_line(void)
{
	if ( (READ_MPEG_REG(ENCI_VIDEO_EN) & 1) == 0 )
    	return READ_MPEG_REG(ENCP_VIDEO_VAVON_BLINE) >> 1;
	else
    	return READ_MPEG_REG(VFIFO2VD_LINE_TOP_START) >> 1;
}

#ifdef SLOW_SYNC_REPEAT
/* add a new function to check if current display frame has been
displayed for its duration */
static inline bool duration_expire(vframe_t *cur_vf, vframe_t *next_vf, u32 dur)
{
    u32 pts;
    s32 dur_disp;
    static s32 rpt_tab_idx = 0;
    static const u32 rpt_tab[4] = {0x100, 0x100, 0x300, 0x300};

    if ((cur_vf == NULL) || (cur_dispbuf == &vf_local))
        return true;

    pts = next_vf->pts;
    if (pts == 0)
        dur_disp = DUR2PTS(cur_vf->duration);
    else
        dur_disp = pts - timestamp_vpts_get();

    if ((dur << 8) >= (dur_disp * rpt_tab[rpt_tab_idx & 3])) {
        rpt_tab_idx = (rpt_tab_idx + 1) & 3;
        return true;
    }
    else
        return false;
}
#endif

static inline bool vpts_expire(vframe_t *cur_vf, vframe_t *next_vf)
{
    u32 pts = next_vf->pts;
    u32 systime;

    if ((trickmode_i == 1)
        || ((trickmode_fffb == 1) && (0 == atomic_read(&trickmode_framedone))))
        return true;

    if (next_vf->duration == 0)
        return true;

    systime = timestamp_pcrscr_get();

    if (pts == 0)
        pts = timestamp_vpts_get() + (cur_vf ? DUR2PTS(cur_vf->duration) : 0);
    /* check video PTS discontinuity */
    else if (abs(systime - pts) > tsync_vpts_discontinuity_margin()) {
        pts = timestamp_vpts_get() + (cur_vf ? DUR2PTS(cur_vf->duration) : 0);

        if ((systime - pts) >= 0) {
            tsync_avevent(VIDEO_TSTAMP_DISCONTINUITY, next_vf->pts);

            return true;
        }
    }

    return ((int)(timestamp_pcrscr_get() - pts) >= 0);
}

static irqreturn_t vsync_isr(int irq, void *dev_id)
{
	wake_up_interruptible(&amvideo_trick_wait);

	return IRQ_HANDLED;
}
#ifdef FIQ_VSYNC
static void __attribute__ ((naked)) vsync_fiq_isr(void)
#else
static irqreturn_t vsync_isr0(int irq, void *dev_id)
#endif
{
	int hold_line;
    s32 i, vout_type;
    vframe_t *vf;
#ifdef DEBUG
    int toggle_cnt;
#endif
#ifdef FIQ_VSYNC
	asm __volatile__ (
		"mov    ip, sp;\n"
		"stmfd	sp!, {r0-r12, lr};\n"
		"sub    sp, sp, #256;\n"
		"sub    fp, sp, #256;\n");
#endif			

#ifdef DEBUG
    toggle_cnt = 0;
#endif

    vout_type = detect_vout_type();
	hold_line = calc_hold_line();

	//di_pre_isr();

    timestamp_pcrscr_inc(vsync_pts_inc);

#ifdef SLOW_SYNC_REPEAT
    frame_repeat_count++;
#endif

    if ((!cur_dispbuf) || (cur_dispbuf == &vf_local)) {
        vf = vf_peek();
        if (vf) 
        	{
            tsync_avevent(VIDEO_START,
                (vf->pts) ? vf->pts : timestamp_vpts_get());

#ifdef SLOW_SYNC_REPEAT
            frame_repeat_count = 0;
#endif

        } 
        else if ((cur_dispbuf == &vf_local) && (video_property_changed))
        {
            if (!blackout)
            {
                /* setting video display property in unregister mode */
                u32 cur_index = cur_dispbuf->canvas0Addr;
                canvas_update_addr(cur_index & 0xff, (u32)keep_y_addr);
                canvas_update_addr((cur_index >> 8)& 0xff, (u32)keep_u_addr);
                canvas_update_addr((cur_index >> 16)&0xff, (u32)keep_v_addr);

                vsync_toggle_frame(cur_dispbuf);
            }
            else
                video_property_changed = false;
        }
		else
			return IRQ_HANDLED;
    }
    
    /* buffer switch management */
    vf = vf_peek();

    while (vf) {
        if (vpts_expire(cur_dispbuf, vf)) {
#ifdef DEBUG
            pr_dbg("VIDEO_PTS = 0x%x, cur_dur=0x%x, next_pts=0x%x, scr = 0x%x\n",
                timestamp_vpts_get(),
                (cur_dispbuf) ? cur_dispbuf->duration : 0,
                vf->pts,
                timestamp_pcrscr_get());

            if (toggle_cnt > 0) pr_dbg("skipped\n");
#endif
            vf = vf_get();

            vsync_toggle_frame(vf);

            if (trickmode_fffb == 1)
            {
                atomic_set(&trickmode_framedone, 1);

				/* bridge to dummy IRQ */
				BRIDGE_IRQ_SET();
                break;
            }

#ifdef SLOW_SYNC_REPEAT
            frame_repeat_count = 0;
#endif
            vf = vf_peek();

        } else {
#ifdef SLOW_SYNC_REPEAT
            /* check if current frame's duration has expired, in this example
             * it compares current frame display duration with 1/1/1/1.5 frame duration
             * every 4 frames there will be one frame play longer than usual.
             * you can adjust this array for any slow sync control as you want.
             * The playback can be smoother than previous method.
             */
            if (duration_expire(cur_dispbuf, vf, frame_repeat_count * vsync_pts_inc))
            {
#ifdef DEBUG
                pr_dbg("slow sync toggle, frame_repeat_count = %d\n", frame_repeat_count);
                pr_dbg("system time = 0x%x, video time = 0x%x\n", timestamp_pcrscr_get(), timestamp_vpts_get());
#endif
                vf = vf_get();
                vsync_toggle_frame(vf);
                frame_repeat_count = 0;

                vf = vf_peek();
            }
			else
#endif            
			/* setting video display property in pause mode */
			if (video_property_changed && cur_dispbuf)
				vsync_toggle_frame(cur_dispbuf);

            break;
        }
#ifdef DEBUG
        toggle_cnt++;
#endif
    }

    /* filter setting management */
    if ((frame_par_ready_to_set) || (frame_par_force_to_set)) {
        cur_frame_par = next_frame_par;
    }

    if (cur_dispbuf) {
        f2v_vphase_t *vphase;
        u32 vin_type = cur_dispbuf->type & VIDTYPE_TYPEMASK;

		if ( deinterlace_mode == 0 )
        	viu_set_dcu(cur_frame_par, cur_dispbuf);

        /* vertical phase */
        vphase = &cur_frame_par->VPP_vf_ini_phase_[vpp_phase_table[vin_type][vout_type]];
        WRITE_MPEG_REG(VPP_VSC_INI_PHASE, ((u32)(vphase->phase) << 8));

        if (vphase->repeat_skip >= 0) {
            /* skip lines */
            WRITE_MPEG_REG_BITS(VPP_VSC_PHASE_CTRL,
                                skip_tab[vphase->repeat_skip],
                                VPP_PHASECTL_INIRCVNUMT_BIT,
                                VPP_PHASECTL_INIRCVNUM_WID +
                                VPP_PHASECTL_INIRPTNUM_WID);

        } else {
            /* repeat first line */
            WRITE_MPEG_REG_BITS(VPP_VSC_PHASE_CTRL, 4,
                                VPP_PHASECTL_INIRCVNUMT_BIT,
                                VPP_PHASECTL_INIRCVNUM_WID);
            WRITE_MPEG_REG_BITS(VPP_VSC_PHASE_CTRL,
                                1 - vphase->repeat_skip,
                                VPP_PHASECTL_INIRPTNUMT_BIT,
                                VPP_PHASECTL_INIRPTNUM_WID);
        }
    }

    if (((frame_par_ready_to_set) || (frame_par_force_to_set)) &&
        (cur_frame_par)) {
        vppfilter_mode_t *vpp_filter = &cur_frame_par->vpp_filter;

        if (cur_dispbuf) {
            u32 zoom_start_y;

            if ((cur_dispbuf->type & VIDTYPE_VIU_FIELD) == 0) {
                zoom_start_y = cur_frame_par->VPP_vd_start_lines_ >> 1;

            } else {
                zoom_start_y = cur_frame_par->VPP_vd_start_lines_;
            }

            zoom_start_x_lines = cur_frame_par->VPP_hd_start_lines_;
            zoom_end_x_lines   = cur_frame_par->VPP_hd_end_lines_;
            zoom_display_horz();

            zoom_start_y_lines = zoom_start_y;
            zoom_end_y_lines   = cur_frame_par->VPP_vd_end_lines_;
            zoom_display_vert();
        }

        /* vpp filters */
        SET_MPEG_REG_MASK(VPP_SC_MISC,
                          VPP_SC_TOP_EN | VPP_SC_VERT_EN | VPP_SC_HORZ_EN);

        /* horitontal filter settings */
        WRITE_MPEG_REG_BITS(VPP_SC_MISC,
                            vpp_filter->vpp_horz_coeff[0],
                            VPP_SC_HBANK_LENGTH_BIT,
                            VPP_SC_BANK_LENGTH_WID);

        if (vpp_filter->vpp_horz_coeff[1] & 0x8000)
            WRITE_MPEG_REG(VPP_SCALE_COEF_IDX, VPP_COEF_HORZ | VPP_COEF_9BIT);
        else
            WRITE_MPEG_REG(VPP_SCALE_COEF_IDX, VPP_COEF_HORZ);

        for (i=0; i < (vpp_filter->vpp_horz_coeff[1] & 0xff); i++) {
            WRITE_MPEG_REG(VPP_SCALE_COEF, vpp_filter->vpp_horz_coeff[i+2]);
        }

        /* vertical filter settings */
        WRITE_MPEG_REG_BITS(VPP_SC_MISC,
                            vpp_filter->vpp_vert_coeff[0],
                            VPP_SC_VBANK_LENGTH_BIT,
                            VPP_SC_BANK_LENGTH_WID);

        WRITE_MPEG_REG(VPP_SCALE_COEF_IDX, VPP_COEF_VERT);
        for (i = 0; i < vpp_filter->vpp_vert_coeff[1]; i++) {
            WRITE_MPEG_REG(VPP_SCALE_COEF,
                           vpp_filter->vpp_vert_coeff[i + 2]);
        }

        WRITE_MPEG_REG(VPP_PIC_IN_HEIGHT,
                       cur_frame_par->VPP_pic_in_height_);

        WRITE_MPEG_REG_BITS(VPP_HSC_PHASE_CTRL,
                            cur_frame_par->VPP_hf_ini_phase_,
                            VPP_HSC_TOP_INI_PHASE_BIT,
                            VPP_HSC_TOP_INI_PHASE_WID);
        WRITE_MPEG_REG(VPP_POSTBLEND_VD1_H_START_END,
                       ((cur_frame_par->VPP_post_blend_vd_h_start_ & VPP_VD_SIZE_MASK) << VPP_VD1_START_BIT) |
                       ((cur_frame_par->VPP_post_blend_vd_h_end_   & VPP_VD_SIZE_MASK) << VPP_VD1_END_BIT));
        WRITE_MPEG_REG(VPP_POSTBLEND_VD1_V_START_END,
                       ((cur_frame_par->VPP_post_blend_vd_v_start_ & VPP_VD_SIZE_MASK) << VPP_VD1_START_BIT) |
                       ((cur_frame_par->VPP_post_blend_vd_v_end_   & VPP_VD_SIZE_MASK) << VPP_VD1_END_BIT));
        WRITE_MPEG_REG(VPP_POSTBLEND_H_SIZE, cur_frame_par->VPP_post_blend_h_size_);

        vpp_settings_h(cur_frame_par);
        vpp_settings_v(cur_frame_par);

        frame_par_ready_to_set = 0;
        frame_par_force_to_set = 0;
    } /* VPP one time settings */

    wait_sync = 0;

    if ( deinterlace_mode != 0 )
    	run_deinterlace(zoom_start_x_lines, zoom_end_x_lines, zoom_start_y_lines, zoom_end_y_lines, cur_dispbuf->type_backup, cur_dispbuf->blend_mode, hold_line);

#ifdef FIQ_VSYNC
exit:
	WRITE_MPEG_REG(IRQ_CLR_REG(INT_VIU_VSYNC), 1 << IRQ_BIT(INT_VIU_VSYNC));

	dsb();

	asm __volatile__ (
		"add	sp, sp, #256 ;\n"
		"ldmia	sp!, {r0-r12, lr};\n"
		"subs	pc, lr, #4;\n");
#else
	return IRQ_HANDLED;
#endif
	
}

static int alloc_keep_buffer(void)
{
    printk("alloc_keep_buffer\n");
    keep_y_addr = __get_free_pages(GFP_KERNEL, get_order(Y_BUFFER_SIZE));
    if (!keep_y_addr)
    {
        printk("%s: failed to alloc y addr\n", __FUNCTION__);
        goto err1;
    }

    keep_y_addr_remap = ioremap_nocache(virt_to_phys((u8 *)keep_y_addr), Y_BUFFER_SIZE);
    if (!keep_y_addr_remap)
    {
        printk("%s: failed to remap y addr\n", __FUNCTION__);
        goto err2;
    }

    keep_u_addr = __get_free_pages(GFP_KERNEL, get_order(U_BUFFER_SIZE));
    if (!keep_u_addr)
    {
        printk("%s: failed to alloc u addr\n", __FUNCTION__);
        goto err3;
    }

    keep_u_addr_remap = ioremap_nocache(virt_to_phys((u8 *)keep_u_addr), U_BUFFER_SIZE);
    if (!keep_u_addr_remap)
    {
        printk("%s: failed to remap u addr\n", __FUNCTION__);
        goto err4;
    }

    keep_v_addr = __get_free_pages(GFP_KERNEL, get_order(V_BUFFER_SIZE));
    if (!keep_v_addr)
    {
        printk("%s: failed to alloc v addr\n", __FUNCTION__);
        goto err5;
    }

    keep_v_addr_remap = ioremap_nocache(virt_to_phys((u8 *)keep_v_addr), U_BUFFER_SIZE);
    if (!keep_v_addr_remap)
    {
        printk("%s: failed to remap v addr\n", __FUNCTION__);
        goto err6;
    }

    return 0;

err6:
    free_pages(keep_v_addr, get_order(U_BUFFER_SIZE));
    keep_v_addr = 0;
err5:
    iounmap(keep_u_addr_remap);
    keep_u_addr_remap = NULL;
err4:
    free_pages(keep_u_addr, get_order(U_BUFFER_SIZE));
    keep_u_addr = 0;
err3:
    iounmap(keep_y_addr_remap);
    keep_y_addr_remap = NULL;
err2:
    free_pages(keep_y_addr, get_order(Y_BUFFER_SIZE));
    keep_y_addr = 0;
err1:
    return -ENOMEM;
}

/*********************************************************
 * FIQ Routines
 *********************************************************/
#ifdef FIQ_VSYNC
/* 4K size of FIQ stack size */
static u8 fiq_stack[4096];

static void __attribute__ ((naked)) fiq_vector(void)
{
	asm __volatile__ ("mov pc, r8 ;");
}
#endif

static void vsync_fiq_up(void)
{

#ifdef  FIQ_VSYNC
	struct pt_regs regs;
	unsigned int mask = 1 << IRQ_BIT(INT_VIU_VSYNC);

	/* prep the special FIQ mode regs */
	memset(&regs, 0, sizeof(regs));
	regs.ARM_r8 = (unsigned long)vsync_fiq_isr;
	regs.ARM_sp = (unsigned long)fiq_stack + sizeof(fiq_stack) - 4;
	set_fiq_regs(&regs);
	set_fiq_handler(fiq_vector, 8);

	SET_CBUS_REG_MASK(IRQ_FIQSEL_REG(INT_VIU_VSYNC), mask);
	enable_fiq(INT_VIU_VSYNC);
#else
   int r;
   r = request_irq(INT_VIU_VSYNC, &vsync_isr0,
                    IRQF_SHARED, "am_sync0",
                    (void *)video_dev_id);
   #endif
}

static void vsync_fiq_down(void)
{
	unsigned int mask = 1 << IRQ_BIT(INT_VIU_VSYNC);
	
	disable_fiq(INT_VIU_VSYNC);
	CLEAR_CBUS_REG_MASK(IRQ_FIQSEL_REG(INT_VIU_VSYNC), mask);
}

/*********************************************************
 * Exported routines
 *********************************************************/
void vf_reg_provider(const vframe_provider_t *p)
{
    ulong flags;

    spin_lock_irqsave(&lock, flags);

    if (vfp)
        vf_unreg_provider();

    vfp = p;

    spin_unlock_irqrestore(&lock, flags);
}

void vf_unreg_provider(void)
{
    ulong flags;

    spin_lock_irqsave(&lock, flags);

    if (cur_dispbuf) {
        vf_local = *cur_dispbuf;
        cur_dispbuf = &vf_local;
    }

    if (trickmode_fffb)
        atomic_set(&trickmode_framedone, 0);

    if (blackout)
        DisableVideoLayer();

    if (!trickmode_fffb)
    {
        vf_keep_current();
    }

    vfp = NULL;
    tsync_avevent(VIDEO_STOP, 0);

	if ( deinterlace_mode == 2 )
    	disable_pre_deinterlace();

    spin_unlock_irqrestore(&lock, flags);
}

void vf_light_unreg_provider(void)
{
    ulong flags;

    spin_lock_irqsave(&lock, flags);

    if (cur_dispbuf) {
        vf_local = *cur_dispbuf;
        cur_dispbuf = &vf_local;
    }

    vfp = NULL;

    spin_unlock_irqrestore(&lock, flags);
}

unsigned int get_post_canvas(void)
{
    return post_canvas;
}

static int canvas_dup(ulong *dst, ulong src_paddr, ulong size)
{
	void __iomem *p = ioremap_wc(src_paddr, size);
	
	if (p) {
		memcpy(dst, p, size);
		iounmap(p);
		
		return 1;
	}
	
	return 0;
}

unsigned int vf_keep_current(void)
{
    u32 cur_index;
    u32 y_index, u_index, v_index;

    if (blackout)
        return 0;

    if (0 == (READ_MPEG_REG(VPP_MISC) & VPP_VD1_POSTBLEND))
        return 0;

    if (!keep_y_addr_remap)
    {
        //if (alloc_keep_buffer())
            return -1;
    }

    cur_index = READ_MPEG_REG(VD1_IF0_CANVAS0);
    y_index = cur_index & 0xff;
    u_index = (cur_index >> 8) & 0xff;
    v_index = (cur_index >> 16) & 0xff;

	if (canvas_dup(keep_y_addr_remap, canvas_get_addr(y_index), Y_BUFFER_SIZE) &&
    	canvas_dup(keep_u_addr_remap, canvas_get_addr(u_index), U_BUFFER_SIZE) &&
    	canvas_dup(keep_v_addr_remap, canvas_get_addr(v_index), V_BUFFER_SIZE)) {
	    canvas_update_addr(y_index, (u32)keep_y_addr);
    	canvas_update_addr(u_index, (u32)keep_u_addr);
    	canvas_update_addr(v_index, (u32)keep_v_addr);
    }

    return 0;
}

EXPORT_SYMBOL(vf_reg_provider);
EXPORT_SYMBOL(vf_unreg_provider);
EXPORT_SYMBOL(vf_light_unreg_provider);
EXPORT_SYMBOL(get_post_canvas);
EXPORT_SYMBOL(vf_keep_current);

/*********************************************************
 * /dev/amvideo APIs
 *********************************************************/
static int amvideo_open(struct inode *inode, struct file *file)
{
    return 0;
}

static int amvideo_release(struct inode *inode, struct file *file)
{
    if (blackout)
        DisableVideoLayer();
    return 0;
}

static int amvideo_ioctl(struct inode *inode, struct file *file,
                        unsigned int cmd, ulong arg)
{
    switch (cmd)
    {
        case AMSTREAM_IOC_TRICKMODE:
            if (arg == TRICKMODE_I)
                trickmode_i = 1;
            else if (arg == TRICKMODE_FFFB)
                trickmode_fffb = 1;
            else
            {
                trickmode_i = 0;
                trickmode_fffb = 0;
            }
            atomic_set(&trickmode_framedone, 0);
            tsync_trick_mode(trickmode_fffb);
            break;

        case AMSTREAM_IOC_TRICK_STAT:
            *((u32 *)arg) = atomic_read(&trickmode_framedone);
            break;

        case AMSTREAM_IOC_VPAUSE:
            tsync_avevent(VIDEO_PAUSE, arg);
            break;

        case AMSTREAM_IOC_AVTHRESH:
            tsync_set_avthresh(arg);
            break;

        case AMSTREAM_IOC_SYNCTHRESH:
            tsync_set_syncthresh(arg);
            break;
            
        default:
            return -EINVAL;
    }

    return 0;
}

static unsigned int amvideo_poll(struct file *file, poll_table *wait_table)
{
    poll_wait(file, &amvideo_trick_wait, wait_table);

    if (atomic_read(&trickmode_framedone))
    {
        atomic_set(&trickmode_framedone, 0);
        return POLLOUT | POLLWRNORM;
    }

    return 0;
}

const static struct file_operations amvideo_fops = {
    .owner    = THIS_MODULE,
    .open     = amvideo_open,
    .release  = amvideo_release,
    .ioctl    = amvideo_ioctl,
    .poll     = amvideo_poll,
};

/*********************************************************
 * SYSFS property functions
 *********************************************************/
#define MAX_NUMBER_PARA 10
#define AMVIDEO_CLASS_NAME "video"

static DEFINE_MUTEX(video_module_mutex);
static char video_axis[40];
static char video_screen_mode[40];

static int *parse_para(char *para, char *para_num)
{
    static unsigned int buffer[MAX_NUMBER_PARA];
    char *endp;
    int *pt = NULL;
    int len = 0, count = 0;

    if (!para)
        return NULL;

    memset(buffer, 0, sizeof(buffer));

    pt = &buffer[0];

    len = strlen(para);

    endp = (char *)buffer;

    do {
        //filter space out
        while (para && (isspace(*para) || !isalnum(*para)) && len) {
            para++;
            len--;
        }

        if (len == 0)
            break;

        *pt++ = simple_strtoul(para, &endp, 0);

        para = endp;

        len = strlen(para);

    } while ((endp) && (++count < *para_num) && (count < MAX_NUMBER_PARA));

    *para_num = count;

    return buffer;
}

static void set_video_window(char *para)
{
    char count = 4;
    disp_rect_t disp_rect;
    int *pt = &disp_rect.x;

    //parse window para .
    memcpy(pt, parse_para(para, &count), sizeof(disp_rect_t));
#ifdef DEBUG
    pr_dbg("video=>x0:%d ,y0:%d,x1:%d,y1:%d\r\n ",
           *pt, *(pt + 1), *(pt + 2), *(pt + 3));
#endif
    vpp_set_video_layer_position(*pt, *(pt + 1), *(pt + 2), *(pt + 3));
}

static ssize_t video_axis_show(struct class *cla, struct class_attribute *attr, char *buf)
{
    disp_rect_t disp_rect;
    int *pt = &disp_rect.x;

    vpp_get_video_layer_position(pt, pt + 1, pt + 2, pt + 3);

    sprintf(video_axis, "%d %d %d %d",
            disp_rect.x, disp_rect.y, disp_rect.w, disp_rect.h);

    return snprintf(buf, 40, "%s\n", video_axis);
}

static ssize_t video_axis_store(struct class *cla, struct class_attribute *attr, const char *buf,
                                size_t count)
{
    mutex_lock(&video_module_mutex);

    snprintf(video_axis, 40, "%s", buf);

    mutex_unlock(&video_module_mutex);

    set_video_window(video_axis);

    video_property_changed = true;

    return strnlen(buf, count);
}

static ssize_t video_screen_mode_show(struct class *cla, struct class_attribute *attr, char *buf)
{
    sprintf(video_screen_mode, "%d:%s", wide_setting,wide_setting?"full screen":"normal");
    return snprintf(buf, 40, "%s\n", video_screen_mode);
}

static ssize_t video_screen_mode_store(struct class *cla, struct class_attribute *attr, const char *buf,
                                size_t count)
{
    mutex_lock(&video_module_mutex);

    snprintf(video_screen_mode, 40, "%s", buf);

    mutex_unlock(&video_module_mutex);

    if(strchr(video_screen_mode,'0'))
    {
        if (0 != wide_setting)
        {
			video_property_changed = true;
            wide_setting=0;
        }
    }
    else if(strchr(video_screen_mode,'1'))
    {
        if (1 != wide_setting)
        {
			video_property_changed = true;
            wide_setting=1;
        }
    }

    return strnlen(buf, count);
}

static ssize_t video_blackout_policy_show(struct class *cla, struct class_attribute *attr, char *buf)
{
    return sprintf(buf, "%d\n", blackout);
}

static ssize_t video_blackout_policy_store(struct class *cla, struct class_attribute *attr, const char *buf,
                                size_t count)
{
    size_t r;

    r = sscanf(buf, "%d", &blackout);
    if (r != 1)
        return -EINVAL;

    return count;
}

static ssize_t video_brightness_show(struct class *cla, struct class_attribute *attr, char *buf)
{
    s32 val = (READ_MPEG_REG(VPP_VADJ1_Y) >> 8) & 0x1ff;

    val = (val << 23) >> 23;

    return sprintf(buf, "%d\n", val);
}

static ssize_t video_brightness_store(struct class *cla, struct class_attribute *attr, const char *buf,
                                size_t count)
{
    size_t r;
    int val;

    r = sscanf(buf, "%d", &val);
    if ((r != 1) || (val < -255) || (val > 255))
        return -EINVAL;

    WRITE_MPEG_REG_BITS(VPP_VADJ1_Y, val, 8, 9);
    WRITE_MPEG_REG(VPP_VADJ_CTRL, VPP_VADJ1_EN);

    return count;
}

static ssize_t video_contrast_show(struct class *cla, struct class_attribute *attr, char *buf)
{
    return sprintf(buf, "%d\n", (int)(READ_MPEG_REG(VPP_VADJ1_Y) & 0xff) - 0x80);
}

static ssize_t video_contrast_store(struct class *cla, struct class_attribute *attr, const char *buf,
                                size_t count)
{
    size_t r;
    int val;

    r = sscanf(buf, "%d", &val);
    if ((r != 1) || (val < -127) || (val > 127))
        return -EINVAL;

    val += 0x80;

    WRITE_MPEG_REG_BITS(VPP_VADJ1_Y, val, 0, 8);
    WRITE_MPEG_REG(VPP_VADJ_CTRL, VPP_VADJ1_EN);

    return count;
}

static ssize_t video_saturation_show(struct class *cla, struct class_attribute *attr, char *buf)
{
    return sprintf(buf, "%d\n", READ_MPEG_REG(VPP_VADJ1_Y) &0xff);
}

static ssize_t video_saturation_store(struct class *cla, struct class_attribute *attr, const char *buf,
                                size_t count)
{
    size_t r;
    int val;

    r = sscanf(buf, "%d", &val);
    if ((r != 1) || (val < -127) || (val > 127))
        return -EINVAL;

    WRITE_MPEG_REG_BITS(VPP_VADJ1_Y, val, 0, 8);
    WRITE_MPEG_REG(VPP_VADJ_CTRL, VPP_VADJ1_EN);

    return count;
}

static ssize_t video_disable_show(struct class *cla, struct class_attribute *attr, char *buf)
{
    return sprintf(buf, "%d\n", disable_video);
}

static ssize_t video_disable_store(struct class *cla, struct class_attribute *attr, const char *buf,
                                size_t count)
{
    size_t r;

    r = sscanf(buf, "%d", &disable_video);
    if (r != 1)
        return -EINVAL;

    if (disable_video)
    {
        DisableVideoLayer();
    }
    else
    {
        if (cur_dispbuf && (cur_dispbuf != &vf_local))
            EnableVideoLayer();
    }

    return count;
}

static ssize_t frame_addr_show(struct class *cla, struct class_attribute *attr, char *buf)
{
    canvas_t canvas;
    u32 addr[3];

    if (cur_dispbuf) {
        canvas_read(cur_dispbuf->canvas0Addr & 0xff, &canvas);
        addr[0] = canvas.addr;
        canvas_read((cur_dispbuf->canvas0Addr >> 8) & 0xff, &canvas);
        addr[1] = canvas.addr;
        canvas_read((cur_dispbuf->canvas0Addr >> 16) & 0xff, &canvas);
        addr[2] = canvas.addr;

        return sprintf(buf, "0x%x-0x%x-0x%x\n", addr[0], addr[1], addr[2]);
    }

    return sprintf(buf, "NA\n");
}

static ssize_t frame_canvas_width_show(struct class *cla, struct class_attribute *attr, char *buf)
{
    canvas_t canvas;
    u32 width[3];

    if (cur_dispbuf) {
        canvas_read(cur_dispbuf->canvas0Addr & 0xff, &canvas);
        width[0] = canvas.width;
        canvas_read((cur_dispbuf->canvas0Addr >> 8) & 0xff, &canvas);
        width[1] = canvas.width;
        canvas_read((cur_dispbuf->canvas0Addr >> 16) & 0xff, &canvas);
        width[2] = canvas.width;

        return sprintf(buf, "%d-%d-%d\n", width[0], width[1], width[2]);
    }

    return sprintf(buf, "NA\n");
}

static ssize_t frame_canvas_height_show(struct class *cla, struct class_attribute *attr, char *buf)
{
    canvas_t canvas;
    u32 height[3];

    if (cur_dispbuf) {
        canvas_read(cur_dispbuf->canvas0Addr & 0xff, &canvas);
        height[0] = canvas.height;
        canvas_read((cur_dispbuf->canvas0Addr >> 8) & 0xff, &canvas);
        height[1] = canvas.height;
        canvas_read((cur_dispbuf->canvas0Addr >> 16) & 0xff, &canvas);
        height[2] = canvas.height;

        return sprintf(buf, "%d-%d-%d\n", height[0], height[1], height[2]);
    }

    return sprintf(buf, "NA\n");
}

static ssize_t frame_width_show(struct class *cla, struct class_attribute *attr, char *buf)
{
    if (cur_dispbuf)
        return sprintf(buf, "%d\n", cur_dispbuf->width);

    return sprintf(buf, "NA\n");
}

static ssize_t frame_height_show(struct class *cla, struct class_attribute *attr, char *buf)
{
    if (cur_dispbuf)
        return sprintf(buf, "%d\n", cur_dispbuf->height);

    return sprintf(buf, "NA\n");
}

static ssize_t frame_format_show(struct class *cla, struct class_attribute *attr, char *buf)
{
    if (cur_dispbuf) {
        if ((cur_dispbuf->type & VIDTYPE_TYPEMASK) == VIDTYPE_INTERLACE_TOP)
            return sprintf(buf, "interlace-top\n");
        else if ((cur_dispbuf->type & VIDTYPE_TYPEMASK) == VIDTYPE_INTERLACE_BOTTOM)
            return sprintf(buf, "interlace-bottom\n");
        else
            return sprintf(buf, "progressive\n");
    }

    return sprintf(buf, "NA\n");
}

static ssize_t frame_aspect_ratio_show(struct class *cla, struct class_attribute *attr, char *buf)
{
    if (cur_dispbuf) {
        u32 ar = (cur_dispbuf->ratio_control & DISP_RATIO_ASPECT_RATIO_MASK)
                 >> DISP_RATIO_ASPECT_RATIO_BIT;

        if (ar)
            return sprintf(buf, "0x%x\n", ar);
        else
            return sprintf(buf, "0x%x\n",
                (cur_dispbuf->width << 8) / cur_dispbuf->height);
    }

    return sprintf(buf, "NA\n");
}

static struct class_attribute amvideo_class_attrs[] = {
    __ATTR(axis,
           S_IRUGO | S_IWUSR,
           video_axis_show,
           video_axis_store),
    __ATTR(screen_mode,
           S_IRUGO | S_IWUSR,
           video_screen_mode_show,
           video_screen_mode_store),
    __ATTR(blackout_policy,
           S_IRUGO | S_IWUSR,
           video_blackout_policy_show,
           video_blackout_policy_store),
    __ATTR(disable_video,
           S_IRUGO | S_IWUSR,
           video_disable_show,
           video_disable_store),
    __ATTR(brightness,
           S_IRUGO | S_IWUSR,
           video_brightness_show,
           video_brightness_store),
    __ATTR(contrast,
           S_IRUGO | S_IWUSR,
           video_contrast_show,
           video_contrast_store),
    __ATTR(saturation,
           S_IRUGO | S_IWUSR,
           video_saturation_show,
           video_saturation_store),
    __ATTR_RO(frame_addr),
    __ATTR_RO(frame_canvas_width),
    __ATTR_RO(frame_canvas_height),
    __ATTR_RO(frame_width),
    __ATTR_RO(frame_height),
    __ATTR_RO(frame_format),
    __ATTR_RO(frame_aspect_ratio),
    __ATTR_NULL
};

static struct class amvideo_class = {
    .name = AMVIDEO_CLASS_NAME,
    .class_attrs = amvideo_class_attrs,
};

static struct device *amvideo_dev;

int vout_notify_callback(struct notifier_block *block, unsigned long cmd , void *para)
{
    const vinfo_t *info;
    ulong flags;

    if (cmd != VOUT_EVENT_MODE_CHANGE)
        return -1;

    info = get_current_vinfo();

    spin_lock_irqsave(&lock, flags);

    vinfo = info;

    /* pre-calculate vsync_pts_inc in 90k unit */
    vsync_pts_inc = 90000 * vinfo->sync_duration_den / vinfo->sync_duration_num;

    spin_unlock_irqrestore(&lock, flags);

    return 0;
}

static struct notifier_block vout_notifier = {
    .notifier_call  = vout_notify_callback,
};

static void vout_hook(void)
{
    vout_register_client(&vout_notifier);

    vinfo = get_current_vinfo();

	if (!vinfo) {
		set_current_vmode(VMODE_720P);

	    vinfo = get_current_vinfo();
	}

    if (vinfo)
        vsync_pts_inc = 90000 * vinfo->sync_duration_den / vinfo->sync_duration_num;

#ifdef DEBUG
    if (vinfo) {
        pr_dbg("vinfo = %p\n", vinfo);
        pr_dbg("display platform %s:\n", vinfo->name);
        pr_dbg("\tresolution %d x %d\n", vinfo->width, vinfo->height);
        pr_dbg("\taspect ratio %d : %d\n", vinfo->aspect_ratio_num, vinfo->aspect_ratio_den);
        pr_dbg("\tsync duration %d : %d\n", vinfo->sync_duration_num, vinfo->sync_duration_den);
    }
#endif
}

/*********************************************************/
static int __init video_init(void)
{
    int r = 0;

#ifdef RESERVE_CLR_FRAME
    alloc_keep_buffer();
#endif

    /* MALI clock settings */
    WRITE_CBUS_REG(HHI_MALI_CLK_CNTL,
		(2 << 9)	|	// select DDR clock as clock source
		(1 << 8)	|	// enable clock gating
		(1 << 0));		// DDR clk / 2

    DisableVideoLayer();

    WRITE_MPEG_REG_BITS(VPP_OFIFO_SIZE, 0x300,
                        VPP_OFIFO_SIZE_BIT, VPP_OFIFO_SIZE_WID);
    CLEAR_MPEG_REG_MASK(VPP_VSC_PHASE_CTRL, VPP_PHASECTL_TYPE_INTERLACE);
#ifndef CONFIG_FB_AML_TCON
    SET_MPEG_REG_MASK(VPP_MISC, VPP_OUT_SATURATE);
#endif
    WRITE_MPEG_REG(VPP_HOLD_LINES, 0x08080808);

    cur_dispbuf = NULL;

    /* hook vsync isr */
    r = request_irq(BRIDGE_IRQ, &vsync_isr,
                    IRQF_SHARED, "amvideo",
                    (void *)video_dev_id);

    if (r) {
        pr_error("video irq register error.\n");
        r = -ENOENT;
        goto err0;
    }
  

    /* sysfs node creation */
    r = class_register(&amvideo_class);
    if (r) {
        pr_error("create video class fail\r\n");
        free_irq(INT_VIU_VSYNC, (void *)video_dev_id);
        goto err1;
    }

    /* create video device */
    r = register_chrdev(AMVIDEO_MAJOR, "amvideo", &amvideo_fops);
    if (r < 0) {
        pr_error("Can't register major for amvideo device\n");
        goto err2;
    }

    amvideo_dev = device_create(&amvideo_class, NULL,
                              MKDEV(AMVIDEO_MAJOR, 0), NULL,
                              DEVICE_NAME);

    if (IS_ERR(amvideo_dev)) {
        pr_error("Can't create amvideo device\n");
        goto err3;
    }

    init_waitqueue_head(&amvideo_trick_wait);

    vout_hook();

    disp_canvas = (disp_canvas_index[2] << 16) | (disp_canvas_index[1] << 8) | disp_canvas_index[0];

	vsync_fiq_up();

	//deinterlace_init(disp_canvas_index);

    return (0);

err3:
    unregister_chrdev(AMVIDEO_MAJOR, DEVICE_NAME);

err2:
    free_irq(BRIDGE_IRQ, (void *)video_dev_id);

err1:
    class_unregister(&amvideo_class);

err0:
    return r;
}

static void __exit video_exit(void)
{
    DisableVideoLayer();
    
    vsync_fiq_down();

    device_destroy(&amvideo_class, MKDEV(AMVIDEO_MAJOR, 0));

    unregister_chrdev(AMVIDEO_MAJOR, DEVICE_NAME);

    free_irq(BRIDGE_IRQ, (void *)video_dev_id);

    class_unregister(&amvideo_class);
}

#ifdef CONFIG_JPEGLOGO
subsys_initcall(video_init);
#else
module_init(video_init);
#endif
module_exit(video_exit);

MODULE_DESCRIPTION("AMLOGIC video output driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Tim Yao <timyao@amlogic.com>");
