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
 * Author:	Tim Yao <timyao@amlogic.com>
 *
 */

#ifndef OSD_H
#define OSD_H
#include  <linux/fb.h>
//do not change this order

enum osd_type_s {
    OSD_TYPE_32_RGBA    = 0,  // 0
    OSD_TYPE_24_RGB     = 1,   // 0
    OSD_TYPE_02_PAL4    = 2,  // 0
    OSD_TYPE_04_PAL16   = 3, // 0
    OSD_TYPE_08_PAL256  = 4,// 0
    OSD_TYPE_16_655     = 5,  // 0
    OSD_TYPE_16_844     = 6,  // 1
    OSD_TYPE_16_6442    = 7, // 2 
    OSD_TYPE_16_4444_R    = 8, // 3
  
    OSD_TYPE_32_ARGB =9, // 1 
    OSD_TYPE_32_ABGR =10,// 2
    OSD_TYPE_32_BGRA =11,// 3 

    OSD_TYPE_24_888_B=12, //RGB  5
    OSD_TYPE_24_5658=13,  //RGBA  1
    OSD_TYPE_24_8565=14,  // 2
    OSD_TYPE_24_6666_R=15, //RGBA 3 
    OSD_TYPE_24_6666_A=16, //ARGB 4 
    

    OSD_TYPE_16_565 = 17 , //RGB 4
    OSD_TYPE_16_4444_A   = 18, // 5
    OSD_TYPE_16_1555_A  = 19, // 6
    OSD_TYPE_16_4642_R    = 20, // 7

    //YUV  mode
    OSD_TYPE_YUV_422 =21 ,

    

    OSD_TYPE_INVALID=0xff
};
typedef  enum {
	BPP_TYPE_02_PAL4    = 2,  // 0
    	BPP_TYPE_04_PAL16   = 4, // 0
	BPP_TYPE_08_PAL256=8,
	BPP_TYPE_16_655 =9,
	BPP_TYPE_16_844 =10,
	BPP_TYPE_16_6442 =11 ,
	BPP_TYPE_16_4444_R = 12,
	BPP_TYPE_16_4642_R = 13,
	BPP_TYPE_16_1555_A=14,
	BPP_TYPE_16_4444_A = 15,
	BPP_TYPE_16_565 =16,
	
	BPP_TYPE_24_6666_A=19,
	BPP_TYPE_24_6666_R=20,
	BPP_TYPE_24_8565 =21,
	BPP_TYPE_24_5658 = 22,
	BPP_TYPE_24_888_B = 23,
	BPP_TYPE_24_RGB = 24,

	BPP_TYPE_32_BGRA=29,
	BPP_TYPE_32_ABGR = 30,
	BPP_TYPE_32_RGBA=31,
	BPP_TYPE_32_ARGB=32,

	BPP_TYPE_YUV_422=33,
	
}bpp_type_t;
typedef  struct {
	bpp_type_t	type_index;
	u8	red_offset ;
	u8	red_length;
	u8	red_msb_right;
	
	u8	green_offset;
	u8	green_length;
	u8	green_msb_right;

	u8	blue_offset;
	u8	blue_length;
	u8	blue_msb_right;

	u8	transp_offset;
	u8	transp_length;
	u8	transp_msb_right;

	u8	color_type;
	u8	bpp;
	
}bpp_color_bit_define_t;
typedef struct osd_ctl_s {
    enum osd_type_s type;
    u32  xres_virtual;
    u32  yres_virtual;
    u32  xres;
    u32  yres;
    u32  disp_start_x; //coordinate of screen 
    u32  disp_start_y;
    u32  disp_end_x;
    u32  disp_end_y;
    u32  addr;
    u32  index;
} osd_ctl_t;

static const  bpp_color_bit_define_t   default_color_format_array[]={
	{
		.type_index=BPP_TYPE_YUV_422,
		.red_offset=0,
		.red_length=0,
		.red_msb_right=0,
		.green_offset=0,
		.green_length=0,
		.green_msb_right=0,
		.blue_offset=0,
		.blue_length=0,
		.blue_msb_right=0,
		.transp_offset=0,
		.transp_length=0,
		.transp_msb_right=0,
		.color_type=FB_VISUAL_TRUECOLOR,
		.bpp=16,
	},
	{
		.type_index=BPP_TYPE_02_PAL4,
		.red_offset=0,
		.red_length=2,
		.red_msb_right=0,
		.green_offset=0,
		.green_length=2,
		.green_msb_right=0,
		.blue_offset=0,
		.blue_length=2,
		.blue_msb_right=0,
		.transp_offset=0,
		.transp_length=0,
		.transp_msb_right=0,
		.color_type=FB_VISUAL_PSEUDOCOLOR,
		.bpp=8,
	}, 
	{
		.type_index=BPP_TYPE_04_PAL16,
		.red_offset=0,
		.red_length=4,
		.red_msb_right=0,
		.green_offset=0,
		.green_length=4,
		.green_msb_right=0,
		.blue_offset=0,
		.blue_length=4,
		.blue_msb_right=0,
		.transp_offset=0,
		.transp_length=0,
		.transp_msb_right=0,
		.color_type=FB_VISUAL_PSEUDOCOLOR,
		.bpp=8,
	},
	{
		.type_index=BPP_TYPE_08_PAL256,
		.red_offset=0,
		.red_length=8,
		.red_msb_right=0,
		.green_offset=0,
		.green_length=8,
		.green_msb_right=0,
		.blue_offset=0,
		.blue_length=8,
		.blue_msb_right=0,
		.transp_offset=0,
		.transp_length=0,
		.transp_msb_right=0,
		.color_type=FB_VISUAL_PSEUDOCOLOR,
		.bpp=8,
	},
	{
		.type_index=BPP_TYPE_16_565,
		.red_offset=11,
		.red_length=5,
		.red_msb_right=0,
		.green_offset=6,
		.green_length=6,
		.green_msb_right=0,
		.blue_offset=0,
		.blue_length=5,
		.blue_msb_right=0,
		.transp_offset=0,
		.transp_length=0,
		.transp_msb_right=0,
		.color_type=FB_VISUAL_TRUECOLOR,
		.bpp=16,
	},
	{
		.type_index=BPP_TYPE_16_844,
		.red_offset=8,
		.red_length=8,
		.red_msb_right=0,
		.green_offset=5,
		.green_length=4,
		.green_msb_right=0,
		.blue_offset=0,
		.blue_length=4,
		.blue_msb_right=0,
		.transp_offset=0,
		.transp_length=0,
		.transp_msb_right=0,
		.color_type=FB_VISUAL_TRUECOLOR,
		.bpp=16,
	},
	{
		.type_index=BPP_TYPE_16_6442,
		.red_offset=10,
		.red_length=6,
		.red_msb_right=0,
		.green_offset=6,
		.green_length=4,
		.green_msb_right=0,
		.blue_offset=2,
		.blue_length=4,
		.blue_msb_right=0,
		.transp_offset=0,
		.transp_length=2,
		.transp_msb_right=0,
		.color_type=FB_VISUAL_TRUECOLOR,
		.bpp=16,
	},
	{
		.type_index=BPP_TYPE_16_4444_R,
		.red_offset=12,
		.red_length=4,
		.red_msb_right=0,
		.green_offset=8,
		.green_length=4,
		.green_msb_right=0,
		.blue_offset=4,
		.blue_length=4,
		.blue_msb_right=0,
		.transp_offset=0,
		.transp_length=4,
		.transp_msb_right=0,
		.color_type=FB_VISUAL_TRUECOLOR,
		.bpp=16,
	},
	{
		.type_index=BPP_TYPE_16_4642_R,
		.red_offset=12,
		.red_length=4,
		.red_msb_right=0,
		.green_offset=6,
		.green_length=6,
		.green_msb_right=0,
		.blue_offset=2,
		.blue_length=4,
		.blue_msb_right=0,
		.transp_offset=0,
		.transp_length=2,
		.transp_msb_right=0,
		.color_type=FB_VISUAL_TRUECOLOR,
		.bpp=16,
	},
	{
		.type_index=BPP_TYPE_16_1555_A,
		.red_offset=10,
		.red_length=5,
		.red_msb_right=0,
		.green_offset=5,
		.green_length=5,
		.green_msb_right=0,
		.blue_offset=0,
		.blue_length=5,
		.blue_msb_right=0,
		.transp_offset=15,
		.transp_length=1,
		.transp_msb_right=0,
		.color_type=FB_VISUAL_TRUECOLOR,
		.bpp=16,
	},
	{
		.type_index=BPP_TYPE_16_4444_A,
		.red_offset=8,
		.red_length=4,
		.red_msb_right=0,
		.green_offset=4,
		.green_length=4,
		.green_msb_right=0,
		.blue_offset=0,
		.blue_length=4,
		.blue_msb_right=0,
		.transp_offset=12,
		.transp_length=4,
		.transp_msb_right=0,
		.color_type=FB_VISUAL_TRUECOLOR,
		.bpp=16,
	},
	{
		.type_index=BPP_TYPE_16_655,
		.red_offset=10,
		.red_length=6,
		.red_msb_right=0,
		.green_offset=5,
		.green_length=5,
		.green_msb_right=0,
		.blue_offset=0,
		.blue_length=5,
		.blue_msb_right=0,
		.transp_offset=0,
		.transp_length=0,
		.transp_msb_right=0,
		.color_type=FB_VISUAL_TRUECOLOR,
		.bpp=16,
	},
	{
		.type_index=BPP_TYPE_24_6666_A,
		.red_offset=12,
		.red_length=6,
		.red_msb_right=0,
		.green_offset=6,
		.green_length=6,
		.green_msb_right=0,
		.blue_offset=0,
		.blue_length=6,
		.blue_msb_right=0,
		.transp_offset=18,
		.transp_length=6,
		.transp_msb_right=0,
		.color_type=FB_VISUAL_TRUECOLOR,
		.bpp=24,
	},
	{
		.type_index=BPP_TYPE_24_6666_R,
		.red_offset=18,
		.red_length=6,
		.red_msb_right=0,
		.green_offset=12,
		.green_length=6,
		.green_msb_right=0,
		.blue_offset=6,
		.blue_length=6,
		.blue_msb_right=0,
		.transp_offset=0,
		.transp_length=6,
		.transp_msb_right=0,
		.color_type=FB_VISUAL_TRUECOLOR,
		.bpp=24,
	},
	{
		.type_index=BPP_TYPE_24_8565,
		.red_offset=11,
		.red_length=5,
		.red_msb_right=0,
		.green_offset=5,
		.green_length=6,
		.green_msb_right=0,
		.blue_offset=0,
		.blue_length=5,
		.blue_msb_right=0,
		.transp_offset=16,
		.transp_length=8,
		.transp_msb_right=0,
		.color_type=FB_VISUAL_TRUECOLOR,
		.bpp=24,
	},
	{
		.type_index=BPP_TYPE_24_5658,
		.red_offset=19,
		.red_length=5,
		.red_msb_right=0,
		.green_offset=13,
		.green_length=6,
		.green_msb_right=0,
		.blue_offset=8,
		.blue_length=5,
		.blue_msb_right=0,
		.transp_offset=0,
		.transp_length=8,
		.transp_msb_right=0,
		.color_type=FB_VISUAL_TRUECOLOR,
		.bpp=24,
	},
	{
		.type_index=BPP_TYPE_24_888_B,
		.red_offset=0,
		.red_length=8,
		.red_msb_right=0,
		.green_offset=16,
		.green_length=6,
		.green_msb_right=0,
		.blue_offset=8,
		.blue_length=5,
		.blue_msb_right=0,
		.transp_offset=0,
		.transp_length=8,
		.transp_msb_right=0,
		.color_type=FB_VISUAL_TRUECOLOR,
		.bpp=24,
	},
	{
		.type_index=BPP_TYPE_24_RGB,
		.red_offset=16,
		.red_length=8,
		.red_msb_right=0,
		.green_offset=8,
		.green_length=8,
		.green_msb_right=0,
		.blue_offset=0,
		.blue_length=8,
		.blue_msb_right=0,
		.transp_offset=0,
		.transp_length=0,
		.transp_msb_right=0,
		.color_type=FB_VISUAL_TRUECOLOR,
		.bpp=24,
	},
	{
		.type_index=BPP_TYPE_32_BGRA,
		.red_offset=8,
		.red_length=8,
		.red_msb_right=0,
		.green_offset=16,
		.green_length=8,
		.green_msb_right=0,
		.blue_offset=24,
		.blue_length=8,
		.blue_msb_right=0,
		.transp_offset=0,
		.transp_length=8,
		.transp_msb_right=0,
		.color_type=FB_VISUAL_TRUECOLOR,
		.bpp=32,
	},
	{
		.type_index=BPP_TYPE_32_ABGR,
		.red_offset=0,
		.red_length=8,
		.red_msb_right=0,
		.green_offset=8,
		.green_length=8,
		.green_msb_right=0,
		.blue_offset=16,
		.blue_length=8,
		.blue_msb_right=0,
		.transp_offset=24,
		.transp_length=8,
		.transp_msb_right=0,
		.color_type=FB_VISUAL_TRUECOLOR,
		.bpp=32,
	},
	{
		.type_index=BPP_TYPE_32_ARGB,
		.red_offset=16,
		.red_length=8,
		.red_msb_right=0,
		.green_offset=8,
		.green_length=8,
		.green_msb_right=0,
		.blue_offset=0,
		.blue_length=8,
		.blue_msb_right=0,
		.transp_offset=24,
		.transp_length=8,
		.transp_msb_right=0,
		.color_type=FB_VISUAL_TRUECOLOR,
		.bpp=32,
	},
	{
		.type_index=BPP_TYPE_32_RGBA,
		.red_offset=24,
		.red_length=8,
		.red_msb_right=0,
		.green_offset=16,
		.green_length=8,
		.green_msb_right=0,
		.blue_offset=8,
		.blue_length=8,
		.blue_msb_right=0,
		.transp_offset=0,
		.transp_length=8,
		.transp_msb_right=0,
		.color_type=FB_VISUAL_TRUECOLOR,
		.bpp=32,
	},
	
		
};

int osd_setup_canvas(int index, unsigned long addr, unsigned w, unsigned h);

void osd_setup(struct osd_ctl_s *osd_ctl,
                u32 xoffset,
                u32 yoffset,
                u32 xres,
                u32 yres,
                u32 xres_virtual ,
                u32 yres_virtual,
                u32 disp_start_x,
                u32 disp_start_y,
                u32 disp_end_x,
                u32 disp_end_y,
                u32 fbmem,
                enum osd_type_s type,
                int index);

void osd_setpal(unsigned regno, unsigned red, unsigned green, unsigned blue, unsigned transp,int index);

void osd_enable(int enable,int index );

void osd_pan_display(unsigned int xoffset, unsigned int yoffset,int index );

#endif /* OSD1_H */
