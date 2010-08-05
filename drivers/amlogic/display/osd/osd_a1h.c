/*
 * Amlogic Apollo
 * frame buffer driver
 *
 * Copyright (C) 2009 Amlogic, Inc.
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
#include <linux/spinlock.h>

#include <mach/am_regs.h>
#include <linux/irqreturn.h>
#include <linux/errno.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/osd/osd.h>
#include <linux/amports/canvas.h>

#define DEBUG
#define OSD_COUNT	2
#define MODULE_NAME "apollofb"

#ifdef  DEBUG
//#define pr_err(fmt, args...) printk(KERN_ERR MODULE_NAME ": " fmt, ## args)
#define pr_dbg(fmt, args...) printk(KERN_DEBUG MODULE_NAME ": " fmt, ## args)
#else
#define pr_err(fmt, args...)
#define pr_dbg(fmt, args...)
#endif

typedef struct {
	s32 x_start;
	s32 x_end;
	s32 y_start;
	s32 y_end;
} pandata_t;

static const u8 blkmode_reg[] = {5, 7, 0, 1, 2, 4, 4, 4, 4,/*32*/5,5,5,/*24*/7,7,7,7,7,/*16*/4,4,4,4};
static const u8 colormat_reg[] = {0, 0, 0, 0, 0, 0, 1, 2, 3,/*32*/1,2,3,/*24*/5,1,2,3,4,/*16*/4,5,6,7};
static const u8 modebits[] = {32,24,2,4,8,16,16,16,16,/*32*/32,32,32,/*24*/24,24,24,24,24,/*16*/16,16,16,16};
static char  tv_scan_mode;

static pandata_t pandata[2];

static spinlock_t osd_lock = SPIN_LOCK_UNLOCKED;

#define OSD1_CANVAS_INDEX 0x40
#define OSD2_CANVAS_INDEX 0x43

#define OSD_GLOBAL_ALPHA_DEF  0xff
#define OSD_DATA_BIG_ENDIAN 	0
#define OSD_DATA_LITTLE_ENDIAN 1
#define OSD_TC_ALPHA_ENABLE_DEF 0  //disable tc_alpha

#define   REG_OFFSET		0x20

int  tv_irq_release(void) ;
static irqreturn_t vsync_isr(int irq, void *dev_id) ;


static void
_init_osd_simple(u32 pix_x_start,
                  u32 pix_x_end,
                  u32 pix_y_start,
                  u32 pix_y_end,
                  u32 display_h_start,
                  u32 display_v_start,
                  u32 display_h_end,
                  u32 display_v_end,
                  u32 canvas_index,
                  enum osd_type_s type,
                  int  index)
{
	u32 data32;
 	u32  vmode ;
	static char   tv_irq_got=0; 	
	
    if(READ_MPEG_REG(ENCI_VIDEO_EN)&0x1)  //interlace
    {
    	vmode=1;
		//display_v_end = 	(display_v_start + display_v_end)/2 ;
    }
	else
	{
		vmode=0;
	}
      	// Program reg VIU_OSD1_TCOLOR_AG<0-3>
	WRITE_MPEG_REG(VIU_OSD1_TCOLOR_AG0 + REG_OFFSET*index, (unsigned)0xffffff00);

	// Program reg VIU_OSD1_BLK0_CFG_W3-4
	data32 = (display_h_start & 0xfff) | (display_h_end & 0xfff) <<16 ;
   	WRITE_MPEG_REG(VIU_OSD1_BLK0_CFG_W3 + REG_OFFSET*index, data32);
	data32 = (display_v_start & 0xfff) | (display_v_end & 0xfff) <<16 ;
    WRITE_MPEG_REG(index?VIU_OSD2_BLK0_CFG_W4:VIU_OSD1_BLK0_CFG_W4, data32);

    // Program reg VIU_OSD1_BLK0_CFG_W1
 	// data32 = (pix_x_start & 0x1fff) | (pix_x_end & 0x1fff) <<16 ;		
	// WRITE_MPEG_REG(VIU_OSD1_BLK0_CFG_W1 + REG_OFFSET*index, data32);

    // Program reg VIU_OSD1_BLK0_CFG_W2
    // data32 = (pix_y_start & 0x1fff) | (pix_y_end & 0x1fff) <<16 ;			
    //WRITE_MPEG_REG(VIU_OSD1_BLK0_CFG_W2 + REG_OFFSET*index, data32);
	pandata[index].x_start = pix_x_start;
	pandata[index].x_end   = pix_x_end;
	pandata[index].y_start = pix_y_start;
	pandata[index].y_end   = pix_y_end;

	WRITE_MPEG_REG(VIU_OSD1_BLK0_CFG_W1,
		(pandata[0].x_start & 0x1fff) | (pandata[0].x_end & 0x1fff) << 16);
	WRITE_MPEG_REG(VIU_OSD1_BLK0_CFG_W2,
		(pandata[0].y_start & 0x1fff) | (pandata[0].y_end & 0x1fff) << 16);

	WRITE_MPEG_REG(VIU_OSD2_BLK0_CFG_W1,
		(pandata[1].x_start & 0x1fff) | (pandata[1].x_end & 0x1fff) << 16);
	WRITE_MPEG_REG(VIU_OSD2_BLK0_CFG_W2,
		(pandata[1].y_start & 0x1fff) | (pandata[1].y_end & 0x1fff) << 16);

    // Program reg VIU_OSD1_BLK0_CFG_W3
 	if( tv_irq_got)
	{
    		tv_irq_release() ;
		tv_irq_got =0; 	 
    	}	
    	if (vmode == 1)  //interlaced
    	{
    		data32 = 3;
    		tv_scan_mode='i';
		if ( request_irq(INT_AMRISC_VIU_VSYNC, &vsync_isr,
                    IRQF_SHARED , "am_osd_tv", (void *)&tv_scan_mode))
    		{
    			  printk(KERN_ERR"can't request irq for vsync\r\n");
    		}
		else
		{
			tv_irq_got=1;
		}
	
    	}
    	else
    	{
    		//release irq  .
    		data32 = 0 ;
    	}
    	data32 |= canvas_index << 16 ;
	data32 |= OSD_DATA_LITTLE_ENDIAN	 <<15 ;
    	data32 |= colormat_reg[type]     << 2;	
	data32 |= OSD_TC_ALPHA_ENABLE_DEF   << 6;	
	data32 |= 1                      << 7; /* rgb enable */
	data32 |= blkmode_reg[type]      << 8; /* osd_blk_mode */
	WRITE_MPEG_REG(VIU_OSD1_BLK0_CFG_W0+ REG_OFFSET*index, data32);

    	// Program reg VIU_OSD1_FIFO_CTRL_STAT
    	data32  = 4   << 5;  // hold_fifo_lines
    	data32 |= 3   << 10; // burst_len_sel: 3=64
    	data32 |= 32  << 12; // fifo_depth_val: 32*8=256
    	WRITE_MPEG_REG(VIU_OSD1_FIFO_CTRL_STAT + REG_OFFSET*index, data32);

    	// Program reg VIU_OSD1_CTRL_STAT
    	data32  = 0x1          << 0; // osd_blk_enable
    	data32 |= OSD_GLOBAL_ALPHA_DEF << 12;
    	WRITE_MPEG_REG(VIU_OSD1_CTRL_STAT + REG_OFFSET*index, data32);
}
void  osd_set_colorkey(u32 index,u32 bpp,u32 colorkey )
{
	u8  r=0,g=0,b=0,a=(colorkey&0xff000000)>>24;
	
	u32  reg=VIU_OSD1_TCOLOR_AG0 + REG_OFFSET*index;

	colorkey&=0x00ffffff;
	 switch(bpp)
	  {
	 		case BPP_TYPE_16_655:
			r=(colorkey>>10&0x3f)<<2;
			g=(colorkey>>5&0x1f)<<3;
			b=(colorkey&0x1f)<<3;
			break;	
			case BPP_TYPE_16_844:
			r=colorkey>>8&0xff;
			g=(colorkey>>4&0xf)<<4;
			b=(colorkey&0xf)<<4;	
			break;	
			case BPP_TYPE_16_565:
			r=(colorkey>>11&0x1f)<<3;
			g=(colorkey>>5&0x3f)<<2;
			b=(colorkey&0x1f)<<3;		
			break;	
			case BPP_TYPE_24_888_B:
			b=colorkey>>16&0xff;
			g=colorkey>>8&0xff;
			r=colorkey&0xff;			
			break;
			case BPP_TYPE_24_RGB:
			r=colorkey>>16&0xff;
			g=colorkey>>8&0xff;
			b=colorkey&0xff;			
			break;	
	 }	
	printk("bpp:%d--r:0x%x g:0x%x b:0x%x ,a:0x%x\r\n",bpp,r,g,b,a);
	WRITE_MPEG_REG(reg,r<<24|g<<16|b<<8|a);
	return ;
}
void  osd_srckey_enable(u32  index,u8 enable)
{
	u32  reg=VIU_OSD1_BLK0_CFG_W0 + REG_OFFSET*index;
	u32  data=READ_MPEG_REG(reg);

	if(enable)
	{
		data|=(1<<6);
	}
	else
	{
		data&=~(1<<6);
	}
	
	WRITE_MPEG_REG(reg,data );
}

void osd_setup(struct osd_ctl_s *osd_ctl,
                u32 xoffset,
                u32 yoffset,
                u32 xres,
                u32 yres,
                u32 xres_virtual,
                u32 yres_virtual,
                u32 disp_start_x,
                u32 disp_start_y,
                u32 disp_end_x,
                u32 disp_end_y,
                u32 fbmem,
                enum osd_type_s type,
                int index 
                )
{
    u32 w = (modebits[type] * xres_virtual + 7) >> 3;

    osd_ctl->type = type;
    osd_ctl->xres = xres;
    osd_ctl->yres = yres;
    osd_ctl->xres_virtual = xres_virtual;
    osd_ctl->yres_virtual = yres_virtual;
    osd_ctl->addr = (u32)fbmem;
    osd_ctl->index = index ?OSD2_CANVAS_INDEX : OSD1_CANVAS_INDEX;

    /* configure canvas */
    canvas_config(osd_ctl->index, osd_ctl->addr,
	              w, osd_ctl->yres_virtual,
	              CANVAS_ADDR_NOWRAP, CANVAS_BLKMODE_LINEAR);

    /* configure OSD1 */
    _init_osd_simple(xoffset, xoffset + (disp_end_x-disp_start_x),
	                 yoffset, yoffset + (disp_end_y-disp_start_y),
	                 disp_start_x, disp_start_y,
	                 disp_end_x,disp_end_y, osd_ctl->index,
	                 type,index);
	
	SET_MPEG_REG_MASK(VIU_OSD1_CTRL_STAT, 1 << 21);
	SET_MPEG_REG_MASK(VIU_OSD2_CTRL_STAT, 1 << 21);	
	WRITE_MPEG_REG_BITS(VIU_OSD1_FIFO_CTRL_STAT, 0x2, 10, 2); //set burst size 48
	WRITE_MPEG_REG_BITS(VIU_OSD2_FIFO_CTRL_STAT, 0x2, 10, 2);
	SET_MPEG_REG_MASK(VPP_MISC, ((index == 0) ? VPP_OSD1_POSTBLEND :
	                  VPP_OSD2_POSTBLEND) | VPP_POSTBLEND_EN);

#ifndef FB_OSD2_ENABLE
    CLEAR_MPEG_REG_MASK(VPP_MISC, VPP_OSD2_POSTBLEND);
#endif

    CLEAR_MPEG_REG_MASK(VPP_MISC, VPP_PREBLEND_EN |
                                  VPP_PRE_FG_OSD2 |
                                  VPP_POST_FG_OSD2);
}

void osd_setpal(unsigned regno,
                 unsigned red,
                 unsigned green,
                 unsigned blue,
                 unsigned transp,
                 int index 
                 )
{

    if (regno < 256) {
        u32 pal;
        pal = ((red   & 0xff) << 24) |
              ((green & 0xff) << 16) |
              ((blue  & 0xff) <<  8) |
              (transp & 0xff);

        WRITE_MPEG_REG(VIU_OSD1_COLOR_ADDR+REG_OFFSET*index, regno);
        WRITE_MPEG_REG(VIU_OSD1_COLOR+REG_OFFSET*index, pal);
    }
}

void osd_enable(int enable ,int index )
{
    unsigned long flags;
    	
    int  enable_bit=index? (VPP_OSD2_POSTBLEND) :(VPP_OSD1_POSTBLEND) ;	//osd1 osd2 select bit.
    
    spin_lock_irqsave(&osd_lock, flags);
     printk(KERN_NOTICE"+++++++%s bit:0x%x \r\n",enable?"enable":"disable",enable_bit);
    	
		
    if (enable) {
        SET_MPEG_REG_MASK(VPP_MISC, enable_bit);
    }
    else {
        CLEAR_MPEG_REG_MASK(VPP_MISC, enable_bit);
    }

    spin_unlock_irqrestore(&osd_lock, flags);
}

void osd_pan_display(unsigned int xoffset, unsigned int yoffset,int index )
{
	ulong flags;
	
	if (index >= 2)
		return;

    spin_lock_irqsave(&osd_lock, flags);

	pandata[index].x_start += xoffset;
	pandata[index].x_end   += xoffset;
	pandata[index].y_start += yoffset;
	pandata[index].y_end   += yoffset;

    spin_unlock_irqrestore(&osd_lock, flags);
}

/********************************************************
*  request vsync irq                                                                        **
*********************************************************/
static irqreturn_t vsync_isr(int irq, void *dev_id)
{
	char*   scan_mode=(char*)dev_id ;
	unsigned  int  fb0_cfg_w0,fb1_cfg_w0;
	unsigned  int  current_field;
	
	WRITE_MPEG_REG(VIU_OSD1_BLK0_CFG_W1,
		(pandata[0].x_start & 0x1fff) | (pandata[0].x_end & 0x1fff) << 16);
	WRITE_MPEG_REG(VIU_OSD1_BLK0_CFG_W2,
		(pandata[0].y_start & 0x1fff) | (pandata[0].y_end & 0x1fff) << 16);

	WRITE_MPEG_REG(VIU_OSD2_BLK0_CFG_W1,
		(pandata[1].x_start & 0x1fff) | (pandata[1].x_end & 0x1fff) << 16);
	WRITE_MPEG_REG(VIU_OSD2_BLK0_CFG_W2,
		(pandata[1].y_start & 0x1fff) | (pandata[1].y_end & 0x1fff) << 16);

	if (*scan_mode=='i')
	{
		fb0_cfg_w0=READ_MPEG_REG(VIU_OSD1_BLK0_CFG_W0);
		fb1_cfg_w0=READ_MPEG_REG(VIU_OSD1_BLK0_CFG_W0+ REG_OFFSET);
		if ((READ_MPEG_REG(ENCP_VIDEO_MODE) & (1 << 12)) && 
        		((READ_MPEG_REG(ENCI_VIDEO_EN) & 1) == 0)) {
       		 /* 1080I */
        		if (READ_MPEG_REG(VENC_ENCP_LINE) < 562) {
           		 /* top field */
            			current_field = 0;
        		} else {
           			 current_field = 1;
        		}
    		} else {
        		current_field = READ_MPEG_REG(VENC_STATA) & 1;
    		}
		fb0_cfg_w0 &=~1;
		fb1_cfg_w0 &=~1;
		fb0_cfg_w0 |=current_field;
		fb1_cfg_w0 |=current_field;
		WRITE_MPEG_REG(VIU_OSD1_BLK0_CFG_W0, fb0_cfg_w0);
		WRITE_MPEG_REG(VIU_OSD1_BLK0_CFG_W0+ REG_OFFSET, fb1_cfg_w0);
		
	}
	//pr_dbg("trigger one vsync int  %c =>  0x%x\r\n",*scan_mode,fb0_cfg_w3);
	
	return  IRQ_HANDLED ;
}

int  tv_irq_release(void)
{
	free_irq(INT_AMRISC_VIU_VSYNC, (void *)&tv_scan_mode) ;
	return  0;
}

