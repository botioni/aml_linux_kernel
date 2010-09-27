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

#ifndef VFRAME_H
#define VFRAME_H

#include <linux/types.h>


#define VIDTYPE_PROGRESSIVE             0x0
#define VIDTYPE_INTERLACE_TOP           0x1
#define VIDTYPE_INTERLACE_BOTTOM        0x3
#define VIDTYPE_TYPEMASK                0x7
#define VIDTYPE_INTERLACE               0x1
#define VIDTYPE_INTERLACE_FIRST         0x8
#define VIDTYPE_PULLDOWN                0x10
#define VIDTYPE_VIU_422                 0x800
#define VIDTYPE_VIU_FIELD               0x1000
#define VIDTYPE_VIU_SINGLE_PLANE        0x2000
#define VIDTYPE_VIU_444                 0x4000

#define DISP_RATIO_FORCECONFIG          0x80000000
#define DISP_RATIO_CTRL_MASK            0x00000003
#define DISP_RATIO_NO_KEEPRATIO         0x00000000
#define DISP_RATIO_KEEPRATIO            0x00000001

#define DISP_RATIO_ASPECT_RATIO_MASK    0x0003ff00
#define DISP_RATIO_ASPECT_RATIO_BIT     8
#define DISP_RATIO_ASPECT_RATIO_MAX     0x3ff


/*
 * Histogram (36 words)
 * If pixel_sum == 0, then all Histogram information are invalid
 */
typedef struct vframe_hist_s
{
    unsigned int   luma_sum;
    unsigned int   chroma_sum;
    unsigned int   pixel_sum;
    unsigned char  luma_max;
    unsigned char  luma_min;
    unsigned short rsv_s;
    unsigned short region_cnt[64];
} vframe_hist_t;

/*
 * Blackbar (2 words)
 * If bottom == 0 or right == 0, then all Blackbar information are invalid
 */
typedef struct vframe_blkbar_s
{
    unsigned short top;
    unsigned short bottom;
    unsigned short left;
    unsigned short right;
} vfame_blkbar_t;

/*
 * Meas (M2 only, 1 word)
 * If vsin == 0, then all Measurement information are invalid
 */
typedef struct vframe_meas_s
{
    float          vsin;      // (Hz)
    // reserved (1 word)
    unsigned int   rsv_i;
} vframe_meas_t;


typedef struct vframe_s {
    u32 index;
    u32 type;
    u32 type_backup;
    u32 blend_mode;
    u32 duration;
    u32 duration_pulldown;
    u32 pts;

    u32 canvas0Addr;
    u32 canvas1Addr;

    u32 bufWidth;
    u32 width;
    u32 height;
    u32 ratio_control;

    /* vframe properties */
    struct vframe_hist_s hist;
    struct vframe_blkbar_s bbar;
    struct vframe_meas_s meas;
} vframe_t;

#endif /* VFRAME_H */

