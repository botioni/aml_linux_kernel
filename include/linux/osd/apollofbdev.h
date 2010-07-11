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

#ifndef APOLLOFBDEV_H
#define APOLLOFBDEV_H

#include "osd.h"
#include <linux/vout/vinfo.h>

typedef struct myfb_dev {
    struct mutex lock;

    struct fb_info *fb_info;
    struct platform_device *dev;

	u32 fb_mem_paddr;
	void __iomem *fb_mem_vaddr;
	u32 fb_len;

    vmode_t vmode;
	bpp_type_t   bpp_type;
    	
    struct osd_ctl_s osd_ctl;
		
} myfb_dev_t;
typedef  struct list_head   list_head_t ;

typedef   struct{
	list_head_t  list;
	struct myfb_dev *fbdev;
}fb_list_t,*pfb_list_t;

typedef  struct {
	int start ;
	int end ;
	int fix_line_length;
}osd_addr_t ;


typedef  struct {
struct osd_ctl_s osd_ctl;

}logo_osd_config_t,*plogo_osd_confit_t;

typedef  struct {
logo_osd_config_t  config;
vmode_t		vmode;
int  bpp;
}logo_osd_dev_t ;

#define fbdev_lock(dev) mutex_lock(&dev->lock);
#define fbdev_unlock(dev) mutex_unlock(&dev->lock);

extern logo_osd_dev_t*  get_init_fbdev(void); //export point


#endif /* APOLLOFBDEV_H */

