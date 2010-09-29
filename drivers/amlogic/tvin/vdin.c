/*
 * VDIN driver
 *
 * Author: Lin Xu <lin.xu@amlogic.com>
 *         Bobby Yang <bo.yang@amlogic.com>
 *
 * Copyright (C) 2010 Amlogic Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */


/* Standard Linux headers */
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/errno.h>
#include <asm/uaccess.h>

/* Amlogic headers */
#include <mach/am_regs.h>
#include "../amports/vframe.h"
#include "../amports/vframe_provider.h"

/* TVIN headers */
#include "tvin_global.h"
#include "vdin_regs.h"
#include "vdin.h"
#include "bt656_601_in.h"


#define VDIN_NAME               "vdin"
#define VDIN_DRIVER_NAME        "vdin"
#define VDIN_MODULE_NAME        "vdin"
#define VDIN_DEVICE_NAME        "vdin"
#define VDIN_CLASS_NAME         "vdin"

/* warning: "pr_dbg" redefined in tvin_global.h */
#if 0
#ifdef DEBUG
#define pr_dbg(fmt, args...) printk(KERN_DEBUG "amvdecvdin: " fmt, ## args)
#else
#define pr_dbg(fmt, args...)
#endif
#define pr_error(fmt, args...) printk(KERN_ERR "amvdecvdin: " fmt, ## args)
#endif


#define VDIN_COUNT              1
#define VDIN_CANVAS             70U
#define BT656IN_BUF_NUM         8
#define BT656IN_ANCI_DATA_SIZE  0x4000 //save anci data from bt656in

static dev_t vdin_devno;
static struct class *vdin_clsp;

typedef struct vdin_dev_s {
    int                         index;
    unsigned int                rd_canvas_index;
    unsigned int                wr_canvas_index;
    struct cdev                 cdev;
    unsigned int                flags;
    unsigned int                mem_start;
    unsigned int                mem_size;

    irqreturn_t (*vdin_isr) (int irq, void *dev_id);

    union vdin_hist_u           hist;
    struct vdin_bbar_info_s     bbar;
    enum vdin_src_e             src;
    enum tvin_sig_format_e      sig_fmt;
} vdin_dev_t;

const static char vdin_irq_id[] = "vdin_irq_id";

static vdin_dev_t *vdin_devp[VDIN_COUNT];

extern void canvas_config(u32 index, ulong addr, u32 width,
                          u32 height, u32 wrap, u32 blkmode);

void set_isr_func(vdin_dev_t * devp, irqreturn_t (*func)(int irq, void *dev_id))
{
    devp->vdin_isr = func;
    return;
}
EXPORT_SYMBOL(set_isr_func);


union vdin_hist_u           vdin_hist;
struct vdin_bbar_info_s     vdin_bbar;
enum   vdin_src_e           vdin_src;

static inline void vdin_set_clkgate( vdin_clkgate_cfg_t *clkgate_cfg)
{
    WRITE_CBUS_REG_BITS(VDIN_COM_GCLK_CTRL, (unsigned int)(clkgate_cfg->bbar),
                        COM_GCLK_BLKBAR_BIT, COM_GCLK_BLKBAR_WID);
    WRITE_CBUS_REG_BITS(VDIN_COM_GCLK_CTRL, (unsigned int)(clkgate_cfg->hist),
                        COM_GCLK_HIST_BIT, COM_GCLK_HIST_WID);
    WRITE_CBUS_REG_BITS(VDIN_COM_GCLK_CTRL, (unsigned int)(clkgate_cfg->lfifo),
                        COM_GCLK_LFIFO_BIT, COM_GCLK_LFIFO_WID);
    WRITE_CBUS_REG_BITS(VDIN_COM_GCLK_CTRL, (unsigned int)(clkgate_cfg->matrix),
                        COM_GCLK_MATRIX_BIT, COM_GCLK_MATRIX_WID);
    WRITE_CBUS_REG_BITS(VDIN_COM_GCLK_CTRL, (unsigned int)(clkgate_cfg->hscl),
                        COM_GCLK_HSCL_BIT, COM_GCLK_HSCL_WID);
    WRITE_CBUS_REG_BITS(VDIN_COM_GCLK_CTRL, (unsigned int)(clkgate_cfg->prehscl),
                        COM_GCLK_PRE_HSCL_BIT, COM_GCLK_PRE_HSCL_WID);
    WRITE_CBUS_REG_BITS(VDIN_COM_GCLK_CTRL, (unsigned int)(clkgate_cfg->top),
                        COM_GCLK_TOP_BIT, COM_GCLK_TOP_WID);
}

/* function collection - MPEG
static inline void vdin_set_mpeg(struct vdin_mpeg_cfg_s mpeg_cfg);
*/

static inline void vdin_set_mpeg( vdin_mpeg_cfg_t *mpeg_cfg)
{
    WRITE_CBUS_REG_BITS(VDIN_COM_CTRL0, mpeg_cfg->en,
        MPEG_TO_VDIN_SEL_BIT, MPEG_TO_VDIN_SEL_WID);
    WRITE_CBUS_REG_BITS(VDIN_COM_CTRL0, (unsigned int)(mpeg_cfg->field),
        MPEG_FID_BIT, MPEG_FID_WID);
}

static inline unsigned int vdin_get_afifo( enum vdin_src_e src)
{
    unsigned int ret = 0;

    switch (src)
    {
        case VDIN_SRC_BT656IN:
            ret = READ_CBUS_REG_BITS(VDIN_COM_STATUS1, FIFO1_OVFL_BIT, FIFO1_OVFL_WID);
            break;
        default:
            break;
    }

    return ret;
}

static inline void vdin_reset_afifo(enum vdin_src_e src)
{
    switch (src)
    {
        case VDIN_SRC_BT656IN:
            WRITE_CBUS_REG_BITS(VDIN_ASFIFO_CTRL0, 1, ASFIFO1_SOFT_RST_BIT,
                ASFIFO1_SOFT_RST_WID);
            WRITE_CBUS_REG_BITS(VDIN_ASFIFO_CTRL0, 0, ASFIFO1_SOFT_RST_BIT,
                ASFIFO1_SOFT_RST_WID);
            WRITE_CBUS_REG_BITS(VDIN_ASFIFO_CTRL0, 1, ASFIFO1_OVFL_STATUS_CLR_BIT,
                ASFIFO1_OVFL_STATUS_CLR_WID);
            WRITE_CBUS_REG_BITS(VDIN_ASFIFO_CTRL0, 0, ASFIFO1_OVFL_STATUS_CLR_BIT,
                ASFIFO1_OVFL_STATUS_CLR_WID);
            break;
        default:
            break;
    }
}

/*
    VDIN clock is system clock ranging 160~200MHz
    Frontend pixel clock must be lower the VDIN clock to avoid of AFIFO
    overflowe
    when frontend pixel clock is higher than VDIN clock, we need decimation
    to avoid of AFIFO overflowed
    decimation_number (2~16) means the down-sampling rate, that is to push
    only one out of decimation_number of pixels into AFIFO
    decimation_phase (0~15) means which one out of  decimation_number of pixels
    is pushed into AFIFO
    decimation_phase is temporarily fixed to 1
*/

static inline void vdin_set_afifo(enum vdin_src_e src)
{
    // set selected AFIFO
    switch (src)
    {
        case VDIN_SRC_BT656IN:
            WRITE_CBUS_REG_BITS(VDIN_ASFIFO_CTRL0, 1, ASFIFO1_DE_EN_BIT,
                ASFIFO1_DE_EN_WID);
            WRITE_CBUS_REG_BITS(VDIN_ASFIFO_CTRL0, 1, ASFIFO2_GO_FLD_EN_BIT,
                ASFIFO2_GO_FLD_EN_WID);
            WRITE_CBUS_REG_BITS(VDIN_ASFIFO_CTRL0, 1, ASFIFO2_GO_LN_EN_BIT,
                ASFIFO2_GO_LN_EN_WID);
            WRITE_CBUS_REG_BITS(VDIN_ASFIFO_CTRL0, 0, ASFIFO2_NEG_ACTIVE_IN_VS_BIT,
                ASFIFO2_NEG_ACTIVE_IN_VS_WID);
            WRITE_CBUS_REG_BITS(VDIN_ASFIFO_CTRL0, 0, ASFIFO2_NEG_ACTIVE_IN_HS_BIT,
                ASFIFO2_NEG_ACTIVE_IN_HS_WID);
            WRITE_CBUS_REG_BITS(VDIN_ASFIFO_CTRL0, 1, ASFIFO2_VS_SOFT_RST_FIFO_EN_BIT,
                ASFIFO2_VS_SOFT_RST_FIFO_EN_WID);
            break;
        default:
            break;
    }
    vdin_reset_afifo(src);
}

static inline void vdin_set_mux(enum vdin_mux_e mux)
{
    switch (mux)
    {
        case VDIN_MUX_YCBCR:
            WRITE_CBUS_REG_BITS(VDIN_COM_CTRL0, 2, COMP2_OUT_SWT_BIT, COMP2_OUT_SWT_WID);
            WRITE_CBUS_REG_BITS(VDIN_COM_CTRL0, 1, COMP1_OUT_SWT_BIT, COMP1_OUT_SWT_WID);
            WRITE_CBUS_REG_BITS(VDIN_COM_CTRL0, 0, COMP0_OUT_SWT_BIT, COMP0_OUT_SWT_WID);
            break;
        case VDIN_MUX_YCRCB:
            WRITE_CBUS_REG_BITS(VDIN_COM_CTRL0, 1, COMP2_OUT_SWT_BIT, COMP2_OUT_SWT_WID);
            WRITE_CBUS_REG_BITS(VDIN_COM_CTRL0, 2, COMP1_OUT_SWT_BIT, COMP1_OUT_SWT_WID);
            WRITE_CBUS_REG_BITS(VDIN_COM_CTRL0, 0, COMP0_OUT_SWT_BIT, COMP0_OUT_SWT_WID);
            break;
        case VDIN_MUX_CBCRY:
            WRITE_CBUS_REG_BITS(VDIN_COM_CTRL0, 1, COMP2_OUT_SWT_BIT, COMP2_OUT_SWT_WID);
            WRITE_CBUS_REG_BITS(VDIN_COM_CTRL0, 0, COMP1_OUT_SWT_BIT, COMP1_OUT_SWT_WID);
            WRITE_CBUS_REG_BITS(VDIN_COM_CTRL0, 2, COMP0_OUT_SWT_BIT, COMP0_OUT_SWT_WID);
            break;
        case VDIN_MUX_CBYCR:
            WRITE_CBUS_REG_BITS(VDIN_COM_CTRL0, 2, COMP2_OUT_SWT_BIT, COMP2_OUT_SWT_WID);
            WRITE_CBUS_REG_BITS(VDIN_COM_CTRL0, 0, COMP1_OUT_SWT_BIT, COMP1_OUT_SWT_WID);
            WRITE_CBUS_REG_BITS(VDIN_COM_CTRL0, 1, COMP0_OUT_SWT_BIT, COMP0_OUT_SWT_WID);
            break;
        case VDIN_MUX_CRYCB:
            WRITE_CBUS_REG_BITS(VDIN_COM_CTRL0, 0, COMP2_OUT_SWT_BIT, COMP2_OUT_SWT_WID);
            WRITE_CBUS_REG_BITS(VDIN_COM_CTRL0, 2, COMP1_OUT_SWT_BIT, COMP1_OUT_SWT_WID);
            WRITE_CBUS_REG_BITS(VDIN_COM_CTRL0, 1, COMP0_OUT_SWT_BIT, COMP0_OUT_SWT_WID);
            break;
        case VDIN_MUX_CRCBY:
            WRITE_CBUS_REG_BITS(VDIN_COM_CTRL0, 0, COMP2_OUT_SWT_BIT, COMP2_OUT_SWT_WID);
            WRITE_CBUS_REG_BITS(VDIN_COM_CTRL0, 1, COMP1_OUT_SWT_BIT, COMP1_OUT_SWT_WID);
            WRITE_CBUS_REG_BITS(VDIN_COM_CTRL0, 2, COMP0_OUT_SWT_BIT, COMP0_OUT_SWT_WID);
            break;
        default:
            break;
    }
}

static inline void vdin_set_src_mux( vdin_src_mux_cfg_t *src_mux_cfg, vdin_dev_t *devp)
{
    devp->src = src_mux_cfg->src;

    if (src_mux_cfg->src == VDIN_SRC_NULL)  // disable
    {
        WRITE_CBUS_REG_BITS(VDIN_COM_CTRL0, 0, COMMON_DATA_IN_EN_BIT,COMMON_DATA_IN_EN_WID);
    }
    else                                    // enable
    {
        WRITE_CBUS_REG_BITS(VDIN_COM_CTRL0, src_mux_cfg->go_field_delay,
                            DLY_GO_FLD_LN_NUM_BIT, DLY_GO_FLD_LN_NUM_WID);
        WRITE_CBUS_REG_BITS(VDIN_COM_CTRL0, (src_mux_cfg->go_field_delay ? 1 : 0),
                            DLY_GO_FLD_EN_BIT, DLY_GO_FLD_EN_WID);
        WRITE_CBUS_REG_BITS(VDIN_COM_CTRL0, src_mux_cfg->hold_lines,
                            HOLD_LN_BIT, HOLD_LN_WID);
        vdin_set_mux(src_mux_cfg->mux);

        WRITE_CBUS_REG_BITS(VDIN_COM_CTRL0, (unsigned int)(src_mux_cfg->src),
                            VDIN_SEL_BIT, VDIN_SEL_WID);
        WRITE_CBUS_REG_BITS(VDIN_COM_CTRL0, 1, COMMON_DATA_IN_EN_BIT, COMMON_DATA_IN_EN_WID);
        vdin_set_afifo(src_mux_cfg->src);
    }
    //add sourec mux here

}

static const unsigned int vdin_hscl_filter[3][33] =
{
    {
        0x00800000, 0x007f0100, 0xff7f0200, 0xfe7f0300,
        0xfd7e0500, 0xfc7e0600, 0xfb7d0800, 0xfb7c0900,
        0xfa7b0b00, 0xfa7a0dff, 0xf9790fff,
        0xf97711ff, 0xf87613ff, 0xf87416fe, 0xf87218fe,
        0xf8701afe, 0xf76f1dfd, 0xf76d1ffd, 0xf76b21fd,
        0xf76824fd, 0xf76627fc, 0xf76429fc,
        0xf7612cfc, 0xf75f2ffb, 0xf75d31fb, 0xf75a34fb,
        0xf75837fa, 0xf7553afa, 0xf8523cfa, 0xf8503ff9,
        0xf84d42f9, 0xf84a45f9, 0xf84848f8,
    },
    {
        0x00800000, 0x007e0200, 0x007c0400, 0x007a0600,
        0x00780800, 0x00760a00, 0x00740c00, 0x00720e00,
        0x00701000, 0x006e1200, 0x006c1400,
        0x006a1600, 0x00681800, 0x00661a00, 0x00641c00,
        0x00621e00, 0x00602000, 0x005e2200, 0x005c2400,
        0x005a2600, 0x00582800, 0x00562a00,
        0x00542c00, 0x00522e00, 0x00503000, 0x004e3200,
        0x004c3400, 0x004a3600, 0x00483800, 0x00463a00,
        0x00443c00, 0x00423e00, 0x00404000,
    },
    {
        0x80000000, 0x7e020000, 0x7c040000, 0x7a060000,
        0x78080000, 0x760a0000, 0x740c0000, 0x720e0000,
        0x70100000, 0x6e120000, 0x6c140000,
        0x6a160000, 0x68180000, 0x661a0000, 0x641c0000,
        0x621e0000, 0x60200000, 0x5e220000, 0x5c240000,
        0x5a260000, 0x58280000, 0x562a0000,
        0x542c0000, 0x522e0000, 0x50300000, 0x4e320000,
        0x4c340000, 0x4a360000, 0x48380000, 0x463a0000,
        0x443c0000, 0x423e0000, 0x40400000,
    },
};


static inline void vdin_set_hscaler( struct vdin_hscl_cfg_s *hsc_cfg)
{
    unsigned int i = 0;
    unsigned int phase_step_integer = 0, phase_step_fraction = 0;
    unsigned long long phase_step = 0;

    WRITE_MPEG_REG(VDIN_SCALE_COEF_IDX, 0x0100);
    for (i=0; i<33; i++)
    {
        WRITE_MPEG_REG(VDIN_SCALE_COEF,
            vdin_hscl_filter[(unsigned int)(hsc_cfg->filter)][i]);
    }
    phase_step = hsc_cfg->prehscl_en ?
        (((hsc_cfg->src_w+1)>>1)<<24)/hsc_cfg->dst_w :
        (hsc_cfg->src_w<<24)/hsc_cfg->dst_w;
    phase_step_integer  = (unsigned int)phase_step>>24;
    phase_step_fraction = (unsigned int)phase_step&0x00ffffff;
    if (hsc_cfg->init_pixi_ptr > 63)
        hsc_cfg->init_pixi_ptr = 63;
    if (hsc_cfg->init_pixi_ptr < -64)
        hsc_cfg->init_pixi_ptr = -64;
    hsc_cfg->init_pixi_ptr &= INIT_PIX_IN_PTR_MSK;
    WRITE_CBUS_REG_BITS(VDIN_WIDTHM1I_WIDTHM1O, hsc_cfg->src_w-1,
                        WIDTHM1I_BIT, WIDTHM1I_WID);
    WRITE_CBUS_REG_BITS(VDIN_WIDTHM1I_WIDTHM1O, hsc_cfg->dst_w-1,
                        WIDTHM1O_BIT, WIDTHM1O_WID);
    WRITE_CBUS_REG_BITS(VDIN_HSC_PHASE_STEP, phase_step_integer,
                        HSCL_PHASE_STEP_INT_BIT, HSCL_PHASE_STEP_INT_WID);
    WRITE_CBUS_REG_BITS(VDIN_HSC_PHASE_STEP, phase_step_fraction,
                        HSCL_PHASE_STEP_FRA_BIT, HSCL_PHASE_STEP_FRA_WID);
    WRITE_CBUS_REG_BITS(VDIN_HSC_INI_CTRL, hsc_cfg->hsc_rpt_p0_num,
                        HSCL_RPT_P0_NUM_BIT, HSCL_RPT_P0_NUM_WID);
    WRITE_CBUS_REG_BITS(VDIN_HSC_INI_CTRL, hsc_cfg->hsc_ini_rcv_num,
                        HSCL_INI_RCV_NUM_BIT, HSCL_INI_RCV_NUM_WID);
    WRITE_CBUS_REG_BITS(VDIN_HSC_INI_CTRL, hsc_cfg->hsc_ini_phase,
                        HSCL_INI_PHASE_BIT, HSCL_INI_PHASE_WID);
    WRITE_CBUS_REG_BITS(VDIN_SC_MISC_CTRL, (unsigned int)(hsc_cfg->init_pixi_ptr),
                        INIT_PIX_IN_PTR_BIT, INIT_PIX_IN_PTR_WID);
    WRITE_CBUS_REG_BITS(VDIN_SC_MISC_CTRL, hsc_cfg->prehscl_en,
                        PRE_HSCL_EN_BIT, PRE_HSCL_EN_WID);
    WRITE_CBUS_REG_BITS(VDIN_SC_MISC_CTRL, hsc_cfg->hscl_en,
                        HSCL_EN_BIT, HSCL_EN_WID);
    WRITE_CBUS_REG_BITS(VDIN_SC_MISC_CTRL, hsc_cfg->short_lineo_en,
                        SHORT_LN_OUT_EN_BIT, SHORT_LN_OUT_EN_WID);
    WRITE_CBUS_REG_BITS(VDIN_SC_MISC_CTRL, hsc_cfg->hsc_nearest_en,
                        HSCL_NEAREST_EN_BIT, HSCL_NEAREST_EN_WID);
    WRITE_CBUS_REG_BITS(VDIN_SC_MISC_CTRL, hsc_cfg->phase0_always_en,
                        PHASE0_ALWAYS_EN_BIT, PHASE0_ALWAYS_EN_WID);
    WRITE_CBUS_REG_BITS(VDIN_SC_MISC_CTRL, (unsigned int)(hsc_cfg->filter_depth),
                        HSCL_BANK_LEN_BIT, HSCL_BANK_LEN_WID);
}


static const struct vdin_matrix_lup_s vdin_matrix_lup[30] =
{
    // VDIN_MATRIX_RGB_YUV601
    //    0     0.257  0.504  0.098     16
    //    0    -0.148 -0.291  0.439    128
    //    0     0.439 -0.368 -0.071    128
    {0x00000000, 0x00000000, 0x01070204, 0x00641f68, 0x1ed601c2, 0x01c21e87,
                                        0x00001fb7, 0x00400200, 0x00000200,},
    // VDIN_MATRIX_YUV601_RGB
    //  -16     1.164  0      1.596      0
    // -128     1.164 -0.391 -0.813      0
    // -128     1.164  2.018  0          0
    {0x07c00600, 0x00000600, 0x04a80000, 0x066204a8, 0x1e701cbf, 0x04a80812,
                                        0x00000000, 0x00000000, 0x00000000,},
    // VDIN_MATRIX_RGB_YUV601F
    //    0     0.299  0.587  0.114      0
    //    0    -0.169 -0.331  0.5      128
    //    0     0.5   -0.419 -0.081    128
    {0x00000000, 0x00000000, 0x01320259, 0x00751f53, 0x1ead0200, 0x02001e53,
                                        0x00001fad, 0x00000200, 0x00000200,},
    // VDIN_MATRIX_YUV601F_RGB
    //    0     1      0      1.402      0
    // -128     1     -0.344 -0.714      0
    // -128     1      1.772  0          0
    {0x00000600, 0x00000600, 0x04000000, 0x059c0400, 0x1ea01d25, 0x04000717,
                                        0x00000000, 0x00000000, 0x00000000,},
    // VDIN_MATRIX_RGBS_YUV601
    //  -16     0.299  0.587  0.114     16
    //  -16    -0.173 -0.339  0.511    128
    //  -16     0.511 -0.429 -0.083    128
    {0x07c007c0, 0x000007c0, 0x01320259, 0x00751f4f, 0x1ea5020b, 0x020b1e49,
                                        0x00001fab, 0x00400200, 0x00000200,},
    // VDIN_MATRIX_YUV601_RGBS
    //  -16     1      0      1.371     16
    // -128     1     -0.336 -0.698     16
    // -128     1      1.733  0         16
    {0x07c00600, 0x00000600, 0x04000000, 0x057c0400, 0x1ea81d35, 0x040006ef,
                                        0x00000000, 0x00400040, 0x00000040,},
    // VDIN_MATRIX_RGBS_YUV601F
    //  -16     0.348  0.683  0.133      0
    //  -16    -0.197 -0.385  0.582    128
    //  -16     0.582 -0.488 -0.094    128
    {0x07c007c0, 0x000007c0, 0x016402bb, 0x00881f36, 0x1e760254, 0x02541e0c,
                                        0x00001fa0, 0x00000200, 0x00000200,},
    // VDIN_MATRIX_YUV601F_RGBS
    //    0     0.859  0      1.204     16
    // -128     0.859 -0.295 -0.613     16
    // -128     0.859  1.522  0         16
    {0x00000600, 0x00000600, 0x03700000, 0x04d10370, 0x1ed21d8c, 0x03700617,
                                        0x00000000, 0x00400040, 0x00000040,},
    // VDIN_MATRIX_YUV601F_YUV601
    //    0     0.859  0      0         16
    // -128     0      0.878  0        128
    // -128     0      0      0.878    128
    {0x00000600, 0x00000600, 0x03700000, 0x00000000, 0x03830000, 0x00000000,
                                        0x00000383, 0x00400200, 0x00000200,},
    // VDIN_MATRIX_YUV601_YUV601F
    //  -16     1.164  0      0          0
    // -128     0      1.138  0        128
    // -128     0      0      1.138    128
    {0x07c00600, 0x00000600, 0x04a80000, 0x00000000, 0x048d0000, 0x00000000,
                                        0x0000048d, 0x00000200, 0x00000200,},
    // VDIN_MATRIX_RGB_YUV709
    //    0     0.183  0.614  0.062     16
    //    0    -0.101 -0.338  0.439    128
    //    0     0.439 -0.399 -0.04     128
    {0x00000000, 0x00000000, 0x00bb0275, 0x003f1f99, 0x1ea601c2, 0x01c21e67,
                                        0x00001fd7, 0x00400200, 0x00000200,},
    // VDIN_MATRIX_YUV709_RGB
    //  -16     1.164  0      1.793      0
    // -128     1.164 -0.213 -0.534      0
    // -128     1.164  2.115  0          0
    {0x07c00600, 0x00000600, 0x04a80000, 0x072c04a8, 0x1f261ddd, 0x04a80876,
                                        0x00000000, 0x00000000, 0x00000000,},
    // VDIN_MATRIX_RGB_YUV709F
    //    0     0.213  0.715  0.072      0
    //    0    -0.115 -0.385  0.5      128
    //    0     0.5   -0.454 -0.046    128
    {0x00000000, 0x00000000, 0x00da02dc, 0x004a1f8a, 0x1e760200, 0x02001e2f,
                                        0x00001fd1, 0x00000200, 0x00000200,},
    // VDIN_MATRIX_YUV709F_RGB
    //    0     1      0      1.575      0
    // -128     1     -0.187 -0.468      0
    // -128     1      1.856  0          0
    {0x00000600, 0x00000600, 0x04000000, 0x064d0400, 0x1f411e21, 0x0400076d,
                                        0x00000000, 0x00000000, 0x00000000,},
    // VDIN_MATRIX_RGBS_YUV709
    //  -16     0.213  0.715  0.072     16
    //  -16    -0.118 -0.394  0.511    128
    //  -16     0.511 -0.464 -0.047    128
    {0x07c007c0, 0x000007c0, 0x00da02dc, 0x004a1f87, 0x1e6d020b, 0x020b1e25,
                                        0x00001fd0, 0x00400200, 0x00000200,},
    // VDIN_MATRIX_YUV709_RGBS
    //  -16     1      0      1.54      16
    // -128     1     -0.183 -0.459     16
    // -128     1      1.816  0         16
    {0x07c00600, 0x00000600, 0x04000000, 0x06290400, 0x1f451e2a, 0x04000744,
                                        0x00000000, 0x00400040, 0x00000040,},
    // VDIN_MATRIX_RGBS_YUV709F
    //  -16     0.248  0.833  0.084      0
    //  -16    -0.134 -0.448  0.582    128
    //  -16     0.582 -0.529 -0.054    128
    {0x07c007c0, 0x000007c0, 0x00fe0355, 0x00561f77, 0x1e350254, 0x02541de2,
                                        0x00001fc9, 0x00000200, 0x00000200,},
    // VDIN_MATRIX_YUV709F_RGBS
    //    0     0.859  0      1.353     16
    // -128     0.859 -0.161 -0.402     16
    // -128     0.859  1.594  0         16
    {0x00000600, 0x00000600, 0x03700000, 0x05690370, 0x1f5b1e64, 0x03700660,
                                        0x00000000, 0x00400040, 0x00000040,},
    // VDIN_MATRIX_YUV709F_YUV709
    //    0     0.859  0      0         16
    // -128     0      0.878  0        128
    // -128     0      0      0.878    128
    {0x00000600, 0x00000600, 0x03700000, 0x00000000, 0x03830000, 0x00000000,
                                        0x00000383, 0x00400200, 0x00000200,},
    // VDIN_MATRIX_YUV709_YUV709F
    //  -16     1.164  0      0          0
    // -128     0      1.138  0        128
    // -128     0      0      1.138    128
    {0x07c00600, 0x00000600, 0x04a80000, 0x00000000, 0x048d0000, 0x00000000,
                                        0x0000048d, 0x00000200, 0x00000200,},
    // VDIN_MATRIX_YUV601_YUV709
    //  -16     1     -0.115 -0.207     16
    // -128     0      1.018  0.114    128
    // -128     0      0.075  1.025    128
    {0x07c00600, 0x00000600, 0x04001f8a, 0x1f2c0000, 0x04120075, 0x0000004d,
                                        0x0000041a, 0x00400200, 0x00000200,},
    // VDIN_MATRIX_YUV709_YUV601
    //  -16     1      0.100  0.192     16
    // -128     0      0.990 -0.110    128
    // -128     0     -0.072  0.984    128
    {0x07c00600, 0x00000600, 0x04000066, 0x00c50000, 0x03f61f8f, 0x00001fb6,
                                        0x000003f0, 0x00400200, 0x00000200,},
    // VDIN_MATRIX_YUV601_YUV709F
    //  -16     1.164 -0.134 -0.241      0
    // -128     0      1.160  0.129    128
    // -128     0      0.085  1.167    128
    {0x07c00600, 0x00000600, 0x04a81f77, 0x1f090000, 0x04a40084, 0x00000057,
                                        0x000004ab, 0x00000200, 0x00000200,},
    // VDIN_MATRIX_YUV709F_YUV601
    //    0     0.859  0.088  0.169     16
    // -128     0      0.869 -0.097    128
    // -128     0     -0.063  0.864    128
    {0x00000600, 0x00000600, 0x0370005a, 0x00ad0000, 0x037a1f9d, 0x00001fbf,
                                        0x00000375, 0x00400200, 0x00000200,},
    // VDIN_MATRIX_YUV601F_YUV709
    //    0     0.859 -0.101 -0.182     16
    // -128     0      0.894  0.100    128
    // -128     0      0.066  0.900    128
    {0x00000600, 0x00000600, 0x03701f99, 0x1f460000, 0x03930066, 0x00000044,
                                        0x0000039a, 0x00400200, 0x00000200,},
    // VDIN_MATRIX_YUV709_YUV601F
    //  -16     1.164  0.116  0.223      0
    // -128     0      1.128 -0.126    128
    // -128     0     -0.082  1.120    128
    {0x07c00600, 0x00000600, 0x04a80077, 0x00e40000, 0x04831f7f, 0x00001fac,
                                        0x0000047b, 0x00000200, 0x00000200,},
    // VDIN_MATRIX_YUV601F_YUV709F
    //    0     1     -0.118 -0.212     16
    // -128     0      1.018  0.114    128
    // -128     0      0.075  1.025    128
    {0x00000600, 0x00000600, 0x04001f87, 0x1f270000, 0x04120075, 0x0000004d,
                                        0x0000041a, 0x00400200, 0x00000200,},
    // VDIN_MATRIX_YUV709F_YUV601F
    //    0     1      0.102  0.196      0
    // -128     0      0.990 -0.111    128
    // -128     0     -0.072  0.984    128
    {0x00000600, 0x00000600, 0x04000068, 0x00c90000, 0x03f61f8e, 0x00001fb6,
                                        0x000003f0, 0x00000200, 0x00000200,},
    // VDIN_MATRIX_RGBS_RGB
    //  -16     1.164  0      0          0
    //  -16     0      1.164  0          0
    //  -16     0      0      1.164      0
    {0x07c007c0, 0x000007c0, 0x04a80000, 0x00000000, 0x04a80000, 0x00000000,
                                        0x000004a8, 0x00000000, 0x00000000,},
    // VDIN_MATRIX_RGB_RGBS
    //    0     0.859  0      0         16
    //    0     0      0.859  0         16
    //    0     0      0      0.859     16
    {0x00000000, 0x00000000, 0x03700000, 0x00000000, 0x03700000, 0x00000000,
                                        0x00000370, 0x00400040, 0x00000040,},
};


/* function collection - MATRIX
static inline void vdin_set_matrix(enum vdin_matrix_csc_e matrix_csc);
*/

static inline void vdin_set_matrix(enum vdin_matrix_csc_e matrix_csc)
{
    const struct vdin_matrix_lup_s *matrix;
    if (matrix_csc == VDIN_MATRIX_NULL) // disable
    {
        WRITE_MPEG_REG_BITS(VDIN_MATRIX_CTRL, 0, VDIN_MATRIX_EN_BIT,
            VDIN_MATRIX_EN_WID);

    }
    else                                // enable
    {
        matrix = &vdin_matrix_lup[(unsigned int)matrix_csc - 1];
        WRITE_MPEG_REG(VDIN_MATRIX_PRE_OFFSET0_1,matrix->pre_offset0_1 );
        WRITE_MPEG_REG(VDIN_MATRIX_PRE_OFFSET2,  matrix->pre_offset2   );
        WRITE_MPEG_REG(VDIN_MATRIX_COEF00_01,    matrix->coef00_01     );
        WRITE_MPEG_REG(VDIN_MATRIX_COEF02_10,    matrix->coef02_10     );
        WRITE_MPEG_REG(VDIN_MATRIX_COEF11_12,    matrix->coef11_12     );
        WRITE_MPEG_REG(VDIN_MATRIX_COEF20_21,    matrix->coef20_21     );
        WRITE_MPEG_REG(VDIN_MATRIX_COEF22,       matrix->coef22        );
        WRITE_MPEG_REG(VDIN_MATRIX_OFFSET0_1,    matrix->post_offset0_1);
        WRITE_MPEG_REG(VDIN_MATRIX_OFFSET2,      matrix->post_offset2  );
        WRITE_MPEG_REG_BITS(VDIN_MATRIX_CTRL, 1, VDIN_MATRIX_EN_BIT,
            VDIN_MATRIX_EN_WID);
    }
}

/* function collection - LFIFO
static inline void vdin_set_lfifo(struct vdin_lfifo_cfg_s lfifo_cfg);
*/

static inline void vdin_set_lfifo(struct vdin_lfifo_cfg_s *lfifo_cfg)
{
    WRITE_MPEG_REG_BITS(VDIN_LFIFO_CTRL, lfifo_cfg->len,
        LFIFO_BUF_SIZE_BIT, LFIFO_BUF_SIZE_WID);
    WRITE_MPEG_REG_BITS(VDIN_WR_CTRL, lfifo_cfg->soft_reset_en,
        LFIFO_SOFT_RST_EN_BIT, LFIFO_SOFT_RST_EN_BIT);
}

/* function collection - OUTPUT
static inline void vdin_set_output(struct vdin_output_cfg_s output_cfg);
*/

static inline void vdin_set_output(struct vdin_output_cfg_s *output_cfg)
{
    unsigned temp_data;
    WRITE_MPEG_REG_BITS(VDIN_WR_CTRL, output_cfg->control,
                        WR_OUT_CTRL_BIT, WR_OUT_CTRL_WID);

    if((output_cfg->data_fmt == TVIN_CS_RGB444) ||
        (output_cfg->data_fmt == TVIN_CS_YUV444))
        temp_data = 1;
    else
        temp_data = 0;

    WRITE_MPEG_REG_BITS(VDIN_WR_CTRL, temp_data, WR_FMT_BIT, WR_FMT_BIT);
    WRITE_MPEG_REG_BITS(VDIN_WR_CTRL, output_cfg->canvas_shadow_en,
                        WR_CANVAS_BUF_EN_BIT, WR_CANVAS_BUF_EN_WID);
    WRITE_MPEG_REG_BITS(VDIN_WR_CTRL, output_cfg->req_urgent,
                        WR_REQ_URGENT_BIT, WR_REQ_URGENT_WID);
    WRITE_MPEG_REG_BITS(VDIN_WR_CTRL, output_cfg->req_en,
                        WR_REQ_EN_BIT, WR_REQ_EN_WID);
    WRITE_MPEG_REG_BITS(VDIN_WR_CTRL, output_cfg->canvas_id,
                        WR_CANVAS_BIT, WR_CANVAS_WID);
    WRITE_MPEG_REG_BITS(VDIN_WR_H_START_END, output_cfg->hstart,
                        WR_HSTART_BIT, WR_HSTART_WID);
    WRITE_MPEG_REG_BITS(VDIN_WR_H_START_END, output_cfg->hend,
                        WR_HEND_BIT, WR_HEND_WID);
    WRITE_MPEG_REG_BITS(VDIN_WR_V_START_END, output_cfg->vstart,
                        WR_VSTART_BIT, WR_VSTART_WID);
    WRITE_MPEG_REG_BITS(VDIN_WR_V_START_END, output_cfg->vend,
                        WR_VEND_BIT, WR_VEND_WID);
}

/* function collection - HISTGRAM
static inline void vdin_get_histgram(void);
static inline void vdin_set_histgram(struct vdin_histgram_cfg_s hist_cfg);
*/

static inline void vdin_get_histgram(void)
{
    vdin_hist.reg[ 0] = READ_MPEG_REG(VDIN_HIST_MAX_MIN);
    vdin_hist.reg[ 1] = READ_MPEG_REG(VDIN_HIST_SPL_VAL);
    vdin_hist.reg[ 2] = READ_MPEG_REG(VDIN_HIST_SPL_PIX_CNT);
    vdin_hist.reg[ 3] = READ_MPEG_REG(VDIN_HIST_CHROMA_SUM);
    vdin_hist.reg[ 4] = READ_MPEG_REG(VDIN_DNLP_HIST00);
    vdin_hist.reg[ 5] = READ_MPEG_REG(VDIN_DNLP_HIST01);
    vdin_hist.reg[ 6] = READ_MPEG_REG(VDIN_DNLP_HIST02);
    vdin_hist.reg[ 7] = READ_MPEG_REG(VDIN_DNLP_HIST03);
    vdin_hist.reg[ 8] = READ_MPEG_REG(VDIN_DNLP_HIST04);
    vdin_hist.reg[ 9] = READ_MPEG_REG(VDIN_DNLP_HIST05);
    vdin_hist.reg[10] = READ_MPEG_REG(VDIN_DNLP_HIST06);
    vdin_hist.reg[11] = READ_MPEG_REG(VDIN_DNLP_HIST07);
    vdin_hist.reg[12] = READ_MPEG_REG(VDIN_DNLP_HIST08);
    vdin_hist.reg[13] = READ_MPEG_REG(VDIN_DNLP_HIST09);
    vdin_hist.reg[14] = READ_MPEG_REG(VDIN_DNLP_HIST10);
    vdin_hist.reg[15] = READ_MPEG_REG(VDIN_DNLP_HIST11);
    vdin_hist.reg[16] = READ_MPEG_REG(VDIN_DNLP_HIST12);
    vdin_hist.reg[17] = READ_MPEG_REG(VDIN_DNLP_HIST13);
    vdin_hist.reg[18] = READ_MPEG_REG(VDIN_DNLP_HIST14);
    vdin_hist.reg[19] = READ_MPEG_REG(VDIN_DNLP_HIST15);
    vdin_hist.reg[20] = READ_MPEG_REG(VDIN_DNLP_HIST16);
    vdin_hist.reg[21] = READ_MPEG_REG(VDIN_DNLP_HIST17);
    vdin_hist.reg[22] = READ_MPEG_REG(VDIN_DNLP_HIST18);
    vdin_hist.reg[23] = READ_MPEG_REG(VDIN_DNLP_HIST19);
    vdin_hist.reg[24] = READ_MPEG_REG(VDIN_DNLP_HIST20);
    vdin_hist.reg[25] = READ_MPEG_REG(VDIN_DNLP_HIST21);
    vdin_hist.reg[26] = READ_MPEG_REG(VDIN_DNLP_HIST22);
    vdin_hist.reg[27] = READ_MPEG_REG(VDIN_DNLP_HIST23);
    vdin_hist.reg[28] = READ_MPEG_REG(VDIN_DNLP_HIST24);
    vdin_hist.reg[29] = READ_MPEG_REG(VDIN_DNLP_HIST25);
    vdin_hist.reg[30] = READ_MPEG_REG(VDIN_DNLP_HIST26);
    vdin_hist.reg[31] = READ_MPEG_REG(VDIN_DNLP_HIST27);
    vdin_hist.reg[32] = READ_MPEG_REG(VDIN_DNLP_HIST28);
    vdin_hist.reg[33] = READ_MPEG_REG(VDIN_DNLP_HIST29);
    vdin_hist.reg[34] = READ_MPEG_REG(VDIN_DNLP_HIST30);
    vdin_hist.reg[35] = READ_MPEG_REG(VDIN_DNLP_HIST31);
}

static inline void vdin_set_histgram(struct vdin_hist_cfg_s *hist_cfg)
{
    WRITE_MPEG_REG_BITS(VDIN_HIST_H_START_END, hist_cfg->hstart,
                        HIST_HSTART_BIT, HIST_HSTART_WID);
    WRITE_MPEG_REG_BITS(VDIN_HIST_H_START_END, hist_cfg->hend,
                        HIST_HEND_BIT, HIST_HEND_WID);
    WRITE_MPEG_REG_BITS(VDIN_HIST_V_START_END, hist_cfg->vstart,
                        HIST_VSTART_BIT, HIST_VSTART_WID);
    WRITE_MPEG_REG_BITS(VDIN_HIST_V_START_END, hist_cfg->vend,
                        HIST_VEND_BIT, HIST_VEND_WID);
    WRITE_MPEG_REG_BITS(VDIN_HIST_CTRL, hist_cfg->pow,
                        HIST_POW_BIT, HIST_POW_WID);
    WRITE_MPEG_REG_BITS(VDIN_HIST_CTRL, (unsigned int)(hist_cfg->mux),
                        HIST_MUX_BIT, HIST_MUX_WID);
    WRITE_MPEG_REG_BITS(VDIN_HIST_CTRL, hist_cfg->win_en,
                        HIST_WIN_EN_BIT, HIST_WIN_EN_WID);
    WRITE_MPEG_REG_BITS(VDIN_HIST_CTRL, hist_cfg->rd_en,
                        HIST_RD_EN_BIT, HIST_RD_EN_WID);
}

/* function collection - BLACKBAR
static inline void vdin_get_blackbar(void);
static inline void vdin_reset_blackbar(void);
static inline void vdin_set_blackbar(struct vdin_blackbar_cfg_s blackbar_cfg);
*/

static inline void vdin_get_blackbar(void)
{

    if (READ_MPEG_REG_BITS(VDIN_BLKBAR_STATUS0, BLKBAR_DET_DONE_BIT,
            BLKBAR_DET_DONE_WID)) // done
    {
        unsigned int val;

        val = READ_MPEG_REG_BITS(VDIN_BLKBAR_IND_LEFT_START_END,
                BLKBAR_LEFT_HSTART_BIT, BLKBAR_LEFT_HSTART_WID);
        vdin_bbar.left_hstart = val;
        val = READ_MPEG_REG_BITS(VDIN_BLKBAR_IND_LEFT_START_END,
                BLKBAR_LEFT_HEND_BIT, BLKBAR_LEFT_HEND_WID);
        vdin_bbar.left_hend = val;
        val = READ_MPEG_REG_BITS(VDIN_BLKBAR_IND_RIGHT_START_END,
                BLKBAR_RIGHT_HSTART_BIT, BLKBAR_RIGHT_HSTART_WID);
        vdin_bbar.right_hstart = val;
        val = READ_MPEG_REG_BITS(VDIN_BLKBAR_IND_RIGHT_START_END,
                BLKBAR_RIGHT_HEND_BIT, BLKBAR_RIGHT_HEND_WID);
        vdin_bbar.right_hend = val;
        val =READ_MPEG_REG_BITS(VDIN_BLKBAR_IND_LEFT1_CNT,
                BLKBAR_LEFT1_CNT_BIT, BLKBAR_LEFT1_CNT_WID);
        vdin_bbar.lleft_blk_pixs = val;
        val = READ_MPEG_REG_BITS(VDIN_BLKBAR_IND_LEFT2_CNT,
                BLKBAR_LEFT2_CNT_BIT, BLKBAR_LEFT2_CNT_WID);
        vdin_bbar.lright_blk_pixs = val;
        val = READ_MPEG_REG_BITS(VDIN_BLKBAR_IND_RIGHT1_CNT,
                BLKBAR_RIGHT1_CNT_BIT, BLKBAR_RIGHT1_CNT_BIT);
        vdin_bbar.rleft_blk_pixs = val;
        val = READ_MPEG_REG_BITS(VDIN_BLKBAR_IND_RIGHT2_CNT,
                BLKBAR_RIGHT2_CNT_BIT, BLKBAR_RIGHT2_CNT_BIT);
        vdin_bbar.rright_blk_pixs = val;
        val = READ_MPEG_REG_BITS(VDIN_BLKBAR_STATUS0, BLKBAR_TOP_POS_BIT,
            BLKBAR_TOP_POS_WID);
        vdin_bbar.top = val;
        val = READ_MPEG_REG_BITS(VDIN_BLKBAR_STATUS0, BLKBAR_BTM_POS_BIT,
            BLKBAR_BTM_POS_WID);
        vdin_bbar.bottom = val;
        val = READ_MPEG_REG_BITS(VDIN_BLKBAR_STATUS1, BLKBAR_LEFT_POS_BIT,
            BLKBAR_LEFT_POS_WID);
        vdin_bbar.left = val;
        val = READ_MPEG_REG_BITS(VDIN_BLKBAR_STATUS1, BLKBAR_RIGHT_POS_BIT,
            BLKBAR_RIGHT_POS_WID);
        vdin_bbar.right = val;

        vdin_bbar.valid = 1;
    }
    else  // not done
    {
        vdin_bbar.valid = 0;
    }
}

static inline void vdin_reset_blackbar(void)
{
    WRITE_MPEG_REG_BITS(VDIN_BLKBAR_CTRL0, 0, BLKBAR_DET_SOFT_RST_N_BIT,
        BLKBAR_DET_SOFT_RST_N_WID);
    WRITE_MPEG_REG_BITS(VDIN_BLKBAR_CTRL0, 1, BLKBAR_DET_SOFT_RST_N_BIT,
        BLKBAR_DET_SOFT_RST_N_WID);
}

static inline void vdin_set_blackbar(struct vdin_bbar_cfg_s  *blkbar_cfg)
{
    unsigned int val;

    val = blkbar_cfg->blkpix_thr;
    WRITE_MPEG_REG_BITS(VDIN_BLKBAR_CTRL0, val,
                        BLKBAR_BLK_LVL_BIT, BLKBAR_BLK_LVL_WID);
    val = (2<<(unsigned int)(blkbar_cfg->region_wid_pow))-1;
    WRITE_MPEG_REG_BITS(VDIN_BLKBAR_CTRL0, val,
                        BLKBAR_H_WIDTH_BIT, BLKBAR_H_WIDTH_BIT);
    val = (unsigned int)(blkbar_cfg->src);
    WRITE_MPEG_REG_BITS(VDIN_BLKBAR_CTRL0, val,
                        BLKBAR_COMP_SEL_BIT, BLKBAR_COMP_SEL_BIT);
    val = blkbar_cfg->stat_en;
    WRITE_MPEG_REG_BITS(VDIN_BLKBAR_CTRL0, val,
                        BLKBAR_SW_STAT_EN_BIT, BLKBAR_SW_STAT_EN_WID);
    val = (unsigned int)(blkbar_cfg->mux);
    WRITE_MPEG_REG_BITS(VDIN_BLKBAR_CTRL0, val,
                        BLKBAR_DIN_SEL_BIT, BLKBAR_DIN_SEL_BIT);
    val = blkbar_cfg->en;
    WRITE_MPEG_REG_BITS(VDIN_BLKBAR_CTRL0, val,
                        BLKBAR_DET_TOP_EN_BIT, BLKBAR_DET_TOP_EN_WID);
    val = blkbar_cfg->lhstart;
    WRITE_MPEG_REG_BITS(VDIN_BLKBAR_H_START_END, val,
                        BLKBAR_HSTART_BIT, BLKBAR_HSTART_BIT);
    val = blkbar_cfg->rhend;
    WRITE_MPEG_REG_BITS(VDIN_BLKBAR_H_START_END, val,
                        BLKBAR_HEND_BIT, BLKBAR_HEND_BIT);
    val = blkbar_cfg->vstart;
    WRITE_MPEG_REG_BITS(VDIN_BLKBAR_V_START_END, val,
                        BLKBAR_VSTART_BIT, BLKBAR_VSTART_BIT);
    val = blkbar_cfg->vend;
    WRITE_MPEG_REG_BITS(VDIN_BLKBAR_V_START_END, val,
                        BLKBAR_VEND_BIT, BLKBAR_VEND_BIT);
    val = blkbar_cfg->bbar_thr;
    WRITE_MPEG_REG_BITS(VDIN_BLKBAR_CNT_THRESHOLD, val,
                        BLKBAR_CNT_TH_BIT, BLKBAR_CNT_TH_BIT);
    val = blkbar_cfg->bline_thr_top;
    WRITE_MPEG_REG_BITS(VDIN_BLKBAR_ROW_TH1_TH2, val,
                        BLKBAR_ROW_TH1_BIT, BLKBAR_ROW_TH1_BIT);
    val = blkbar_cfg->bline_thr_btm;
    WRITE_MPEG_REG_BITS(VDIN_BLKBAR_ROW_TH1_TH2, val,
                        BLKBAR_ROW_TH2_BIT, BLKBAR_ROW_TH2_BIT);

    vdin_reset_blackbar();
}


static void vdin_start_dec(struct vdin_dev_s *devp)
{
    switch (devp->src)
    {
        case VDIN_SRC_MPEG:
            break;
        case VDIN_SRC_BT656IN:
            start_amvdec_656_601_camera_in(devp->sig_fmt);
            break;
        case VDIN_SRC_TVFE:
            break;
        case VDIN_SRC_CVD2:
            break;
        case VDIN_SRC_HDMIRX:
            break;
        default:
            break;
    }
}


static void vdin_stop_dec(struct vdin_dev_s *devp)
{
    switch (devp->src)
    {
        case VDIN_SRC_MPEG:
            break;
        case VDIN_SRC_BT656IN:
            stop_amvdec_656_601_camera_in(devp->sig_fmt);
            break;
        case VDIN_SRC_TVFE:
            break;
        case VDIN_SRC_CVD2:
            break;
        case VDIN_SRC_HDMIRX:
            break;
        default:
            break;
    }
}


static void vdin_bt656in_canvas_init(struct vdin_dev_s *devp)
{
    int i = 0;
    unsigned int canvas_width  = 1440;
    unsigned int canvas_height = 288;
    unsigned int decbuf_size   = 0x70000;
    unsigned int decbuf_start  = devp->mem_start + BT656IN_ANCI_DATA_SIZE;

    for ( i = 0; i < BT656IN_BUF_NUM; i++)
    {
        canvas_config(VDIN_CANVAS + i, decbuf_start + i * decbuf_size,
            canvas_width, canvas_height,
            CANVAS_ADDR_NOWRAP, CANVAS_BLKMODE_LINEAR);
    }
    devp->wr_canvas_index = VDIN_CANVAS;
}


static int vdin_canvas_init(struct vdin_dev_s *devp)
{
    int ret = 0;

    if(devp->index > VDIN_COUNT)
        return -1;

    switch (devp->src)
    {
        case VDIN_SRC_MPEG:
            break;
        case VDIN_SRC_BT656IN:
            vdin_bt656in_canvas_init(devp);
            break;
        case VDIN_SRC_TVFE:
            break;
        case VDIN_SRC_CVD2:
            break;
        case VDIN_SRC_HDMIRX:
            break;
        default:
            ret = -1;
            break;
    }
    return ret;
}


static irqreturn_t vdin_isr(int irq, void *dev_id)
{
    struct vdin_dev_s *devp = (struct vdin_dev_s *)dev_id;
    u32 reg, index;
    vframe_t info = {
            0xffffffff,         //type
            0xffffffff,         //type_backup
            0,                  //blend_mode
            0,                  //recycle_by_di_pre
            1600,               //duration
            0,                  //duration_pulldown
            0,                  //pts
            0xff,               //canvas0Addr
            0xff,               //canvas1Addr
            1440,               //bufWidth
            720,                //width
            480,                //height
            0,                  //ratio_control
    };
    vdin_output_cfg_t output_cfg = {
            0xb,    //unsigned int   control; default value
            0,      //enum tvin_color_space_e data_fmt; 0: 422, 1:444
            0,      //unsigned int  canvas_shadow_en; disable the function
            1,      //unsigned int            req_urgent;
            1,      //unsigned int            req_en;
            VDIN_CANVAS,    //unsigned int            canvas_id;
            0,        //unsigned int            hstart;
            719,    //unsigned int            hend;
            0,      //unsigned int            vstart;
            239,    //unsigned int            vend;

    };

    switch (devp->src)
    {
        case VDIN_SRC_MPEG:
            break;
        case VDIN_SRC_BT656IN:
            amvdec_656_601_camera_in_run(&info); //If info.type ( --reture value )is 0xffffffff, the current field is error
            break;
        case VDIN_SRC_TVFE:
            break;
        case VDIN_SRC_CVD2:
            break;
        case VDIN_SRC_HDMIRX:
            break;
        default:
            break;
    }
    //If info.type ( --reture value )is 0xffffffff, the current field is error
    if(info.type == 0xffffffff)
    {
        pr_error("decode data is error, skip the feild data \n");
    }
    else    //do buffer managerment, and send info into video display, please refer to vh264 decode
    {
            //set info.canvas0Addr for display
            //set info.canvas1Addr for display
            //others

            vdin_devp[0]->wr_canvas_index++;
            if(vdin_devp[0]->wr_canvas_index > VDIN_CANVAS + BT656IN_BUF_NUM)
                vdin_devp[0]->wr_canvas_index = VDIN_CANVAS;
            //set vdin setting for next field data
            output_cfg.canvas_id = vdin_devp[0]->wr_canvas_index;
            vdin_set_output(&output_cfg);
    }
    return IRQ_HANDLED;
}




static int vdin_open(struct inode *inode, struct file *file)
{
    int ret = 0;
    vdin_dev_t *devp;

    /* Get the per-device structure that contains this cdev */
    devp = container_of(inode->i_cdev, vdin_dev_t, cdev);
    file->private_data = devp;

    #if 0
    ret = request_irq(INT_VDIN_VSYNC, vdin_isr, IRQF_SHARED, "vdin-irq", (void *)devp);
    if (ret) {
        printk(KERN_ERR "vdin: irq register error.\n");
        return -ENOENT;
    }
    #endif

    return 0;
}

static int vdin_release(struct inode *inode, struct file *file)
{
    vdin_dev_t *devp = file->private_data;
    file->private_data = NULL;

    switch (vdin_devp[0]->src)
    {
        case VDIN_SRC_MPEG:
            break;
        case VDIN_SRC_BT656IN:
            stop_amvdec_656_601_camera_in(TVIN_SIG_FMT_COMPONENT_576I_50D000);
            break;
        case VDIN_SRC_TVFE:
            break;
        case VDIN_SRC_CVD2:
            break;
        case VDIN_SRC_HDMIRX:
            break;
        default:
            break;
    }

    return 0;
}



static int vdin_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
    int ret = 0;
    vdin_dev_t *devp;
    void __user *argp = (void __user *)arg;

	if (_IOC_TYPE(cmd) != VDIN_IOC_MAGIC) {
		return -EINVAL;
	}

    devp = container_of(inode->i_cdev, vdin_dev_t, cdev);

    switch (cmd)
    {
        case VDIN_IOCS_CLKGATE:
        {
            struct vdin_clkgate_cfg_s clkgate_cfg = {
                .bbar       = VDIN_CLK_AUTO,
                .hist       = VDIN_CLK_AUTO,
                .lfifo      = VDIN_CLK_AUTO,
                .matrix     = VDIN_CLK_AUTO,
                .hscl       = VDIN_CLK_AUTO,
                .prehscl    = VDIN_CLK_AUTO,
                .top        = VDIN_CLK_AUTO};
            if (copy_from_user(&clkgate_cfg, argp, sizeof(vdin_clkgate_cfg_t)))
            {
                ret = -EFAULT;
                break;
            }

            vdin_set_clkgate(&clkgate_cfg);
            break;
        }

        case VDIN_IOCS_MPEG:
        {
            vdin_mpeg_cfg_t mpeg_cfg = {0, 0};
            if (copy_from_user(&mpeg_cfg, argp, sizeof(vdin_mpeg_cfg_t)))
            {
                ret = -EFAULT;
                break;
            }

            vdin_set_mpeg(&mpeg_cfg);
            break;
        }

        case VDIN_IOCS_SRC_MUX:
        {
            vdin_src_mux_cfg_t src_mux_cfg = {0, 0, 0, 0};
            if (copy_from_user(&src_mux_cfg, argp, sizeof(vdin_src_mux_cfg_t)))
            {
                ret = -EFAULT;
                break;
            }

            vdin_set_src_mux(&src_mux_cfg, devp);
            break;
        }

        case VDIN_IOCS_HSCL:
        {
            vdin_hscl_cfg_t hsc_cfg = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
            if (copy_from_user(&hsc_cfg, argp, sizeof(vdin_hscl_cfg_t)))
            {
                ret = -EFAULT;
                break;
            }

            vdin_set_hscaler(&hsc_cfg);
            break;
        }

        case VDIN_IOCS_MATRIX:
        {
            enum vdin_matrix_csc_e matrix_csc = 0;
            if (copy_from_user(&matrix_csc, argp, sizeof(enum vdin_matrix_csc_e)))
            {
                ret = -EFAULT;
                break;
            }

            vdin_set_matrix(matrix_csc);
            break;
        }

        case VDIN_IOCS_LFIFO:
        {
            vdin_lfifo_cfg_t lfifo_cfg = {0, 0};
            if (copy_from_user(&lfifo_cfg, argp, sizeof(vdin_lfifo_cfg_t)))
            {
                ret = -EFAULT;
                break;
            }

            vdin_set_lfifo(&lfifo_cfg);
            break;
        }

        case VDIN_IOCS_OUTPUT:
        {
            vdin_output_cfg_t output_cfg = {
                .control            = 0xb,
                .data_fmt           = 0,
                .canvas_shadow_en   = 0,
                .req_urgent         = 1,
                .req_en             = 1,
                .canvas_id          = VDIN_CANVAS,
                .hstart             = 0,
                .hend               = 719,
                .vstart             = 0,
                .vend               = 239,
            };

            if (copy_from_user(&output_cfg, argp, sizeof(vdin_output_cfg_t)))
            {
                ret = -EFAULT;
                break;
            }

            vdin_set_output(&output_cfg);
            break;
        }

        case VDIN_IOCS_HIST:
        {
            vdin_hist_cfg_t hist_cfg = {0, 0, 0, 0, 0, 0, 0, 0};
            if (copy_from_user(&hist_cfg, argp, sizeof(vdin_hist_cfg_t)))
            {
                ret = -EFAULT;
                break;
            }

            vdin_set_histgram(&hist_cfg);
            break;
        }

        case VDIN_IOCS_BBAR:
        {
            vdin_bbar_cfg_t blkbar_cfg = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
            if (copy_from_user(&blkbar_cfg, argp, sizeof(vdin_bbar_cfg_t)))
            {
                ret = -EFAULT;
                break;
            }

            vdin_set_blackbar(&blkbar_cfg);
            break;
        }

        case VDIN_IOC_INIT:
            vdin_canvas_init(devp);
            break;

        case VDIN_IOC_START_DEC:
        {
            tvin_sig_format_t sig_fmt = TVIN_SIG_FMT_NULL;
            if (copy_from_user(&sig_fmt, argp, sizeof(tvin_sig_format_t)))
            {
                ret = -EFAULT;
                break;
            }
            devp->sig_fmt = sig_fmt;
            vdin_start_dec(devp);
            break;
        }

        case VDIN_IOC_STOP_DEC:
            vdin_stop_dec(devp);
            break;
        default:
            ret = -ENOIOCTLCMD;
            break;
    }

    return ret;
}


static struct file_operations vdin_fops = {
    .owner   = THIS_MODULE,
    .open    = vdin_open,
    .release = vdin_release,
    .ioctl   = vdin_ioctl,
};


static int vdin_probe(struct platform_device *pdev)
{
    int ret;
    int i;
    struct device *devp;
    struct resource *res;

    ret = alloc_chrdev_region(&vdin_devno, 0, VDIN_COUNT, VDIN_NAME);
	if (ret < 0) {
		printk(KERN_ERR "vdin: failed to allocate major number\n");
		return 0;
	}

    vdin_clsp = class_create(THIS_MODULE, VDIN_NAME);
    if (IS_ERR(vdin_clsp))
    {
        unregister_chrdev_region(vdin_devno, VDIN_COUNT);
        return PTR_ERR(vdin_clsp);
    }

    /* @todo do with resources */
    res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
    if (!res)
    {
        printk(KERN_ERR "vdin: can't get memory resource\n");
        return -EFAULT;
    }

    for (i = 0; i < VDIN_COUNT; ++i)
    {
        /* allocate memory for the per-device structure */
        vdin_devp[i] = kmalloc(sizeof(struct vdin_dev_s), GFP_KERNEL);
        if (!vdin_devp[i])
        {
            printk(KERN_ERR "vdin: failed to allocate memory for vdin device\n");
            return -ENOMEM;
        }
        vdin_devp[i]->index = i;

        /* connect the file operations with cdev */
        cdev_init(&vdin_devp[i]->cdev, &vdin_fops);
        vdin_devp[i]->cdev.owner = THIS_MODULE;
        /* connect the major/minor number to the cdev */
        ret = cdev_add(&vdin_devp[i]->cdev, (vdin_devno + i), 1);
    	if (ret) {
    		printk(KERN_ERR "vdin: failed to add device\n");
            /* @todo do with error */
    		return ret;
    	}
        /* create /dev nodes */
        devp = device_create(vdin_clsp, NULL, MKDEV(MAJOR(vdin_devno), i),
                            NULL, "vdin%d", i);
        if (IS_ERR(devp)) {
            printk(KERN_ERR "vdin: failed to create device node\n");
            class_destroy(vdin_clsp);
            /* @todo do with error */
            return PTR_ERR(devp);;
    	}

        vdin_devp[i]->mem_start = res->start;
        vdin_devp[i]->mem_size  = res->end - res->start + 1;
        vdin_devp[i]->src  = VDIN_SRC_BT656IN;
    }

    #if 1
    ret = request_irq(INT_VDIN_VSYNC, &vdin_isr, IRQF_SHARED, VDIN_NAME, (void *)vdin_irq_id);
    if (ret ) {
          printk(KERN_ERR "vdin: irq register error.\n");
          return -ENOENT;
    }
    #endif

    printk(KERN_INFO "vdin: driver initialized ok\n");
    return 0;
}

static int vdin_remove(struct platform_device *pdev)
{
    int i = 0;
    free_irq(INT_VDIN_VSYNC,(void *)vdin_irq_id);
    unregister_chrdev_region(vdin_devno, VDIN_COUNT);
    for (i = 0; i < VDIN_COUNT; ++i)
    {
        device_destroy(vdin_clsp, MKDEV(MAJOR(vdin_devno), i));
        cdev_del(&vdin_devp[i]->cdev);
        kfree(vdin_devp[i]);
    }
    class_destroy(vdin_clsp);
    return 0;
}

static struct platform_driver vdin_driver = {
    .probe      = vdin_probe,
    .remove     = vdin_remove,
    .driver     = {
        .name   = VDIN_DRIVER_NAME,
    }
};

static int __init vdin_init(void)
{
    int ret = 0;
    ret = platform_driver_register(&vdin_driver);
    if (ret != 0) {
        printk(KERN_ERR "failed to register vdin module, error %d\n", ret);
        return -ENODEV;
    }
    return ret;
}

static void __exit vdin_exit(void)
{
    platform_driver_unregister(&vdin_driver);
}

module_init(vdin_init);
module_exit(vdin_exit);

MODULE_DESCRIPTION("AMLOGIC VDIN driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Xu Lin <lin.xu@amlogic.com>");
