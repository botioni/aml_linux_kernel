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
 *		    jianfeng_wang : add ge2d support 09/05/21	
 */

#include <linux/version.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/io.h>
#include <linux/mm.h>
#include <linux/interrupt.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/fb.h>
#include <linux/spinlock.h>
#include <asm/cacheflush.h>
#include <mach/am_regs.h>
#include <linux/fs.h>
#include <linux/sysfs.h>
#include <linux/file.h>
#include <linux/fdtable.h>
#include <linux/osd/apollo_main.h>
#include <linux/osd/apollodev.h>
#include <linux/osd/apollofbdev.h>
#include <linux/slab.h>
#include <pr_dbg.h>
#include <asm/uaccess.h>


#define DRIVER_NAME "apollofb"
#define MODULE_NAME "apollofb"
#define  FBIOPUT_OSD_SRCCOLORKEY	0x46fb
#define  FBIOPUT_OSD_SRCKEY_ENABLE	0x46fa

#undef pr_dbg
#undef pr_err
static int  debug_enable;
static DEFINE_MUTEX(dbg_mutex);
static ulong def_xres;
static ulong def_yres;
static uint vsync_cnt = 0;
static list_head_t   fbdev_list_head;

#ifdef CONFIG_FB_AML_LOGO
extern logo_osd_dev_t*  get_init_fbdev(void); //export point
#endif


#if 0
static irqreturn_t
apollofb_vsync_irq(int irq, void *dev_instance)
{
    vsync_cnt++;

    return IRQ_HANDLED;
}
#endif


extern void  osd_set_colorkey(u32 index,u32 bpp,u32 colorkey ) ;
extern void  osd_srckey_enable(u32  index,u8 enable);

/*if memsize can't fit standard mode,we guess one w:h=4:3,32bpp window
* 4ax3a*4=memsize so we must get 'a' first,we have no fp sqrt,then  
*we make one ,and you can change  it by set_para  if you not prefer this 
*window size and w-h ratio*/
void  _construct_virtual_window(u32 mem_len,struct fb_var_screeninfo *var)
{//300k is the max value

	u32   a2,low,high,mid,tmp;
	pr_dbg("mem_len=%d\r\n",mem_len) ;
	if(mem_len < 48)  //  too small    
	{
		var->xres=0;
		var->yres=0;
		var->bits_per_pixel=0;
		return ;
	}
	
	a2=mem_len/48 ; // 4:3
	pr_dbg("a2=%d\r\n",a2) ;
	low=1 ,high=a2/2;
	while(low<high)
	{
		mid=(low+high)/2;
		if(0xffffffff/mid < mid)
		{
			tmp=0xffffffff;  //likely
		}
		else
		{
			tmp=mid*mid;
		}
		if(tmp>a2) high=mid-1;
		else
		{
			if(tmp==a2) 
			{
				low=high=mid;
				continue;
			}
			else
			{
				low=mid+1;
			}
		}
	}
	//high is the value ;
	var->xres= 4*high;
	var->yres= 3*high;
	var->xres_virtual=var->xres ;
	var->yres_virtual=var->yres ;
	var->bits_per_pixel=32;
	
}

#if 0
/*
*	--  --  --  --
*			--   --  --  --  --
*						--	--	--	--
*								    -- 	--	--	--
*    memsize from lower mode to higher mode will be overlay just like above slash line figure.
*	
*/
static void
_get_xyres_via_memsize(u32 mem_len,struct fb_var_screeninfo *var)
{
	int i,j,x,y;
	int mode_hit=0 ;
		
	for (i=0 ; i<TVMODE_MAX;i++)
	{
		x=valid_tv_mode[i].xres ;
		y=valid_tv_mode[i].yres ;
		for(j=1;j<=4;j++)/*8 , 16 ,24 ,32 bit */
		{
			/*x*y*j  memory requirment*/
			if(x*y*j>mem_len)
			{
				mode_hit=1;
				break;
			}
		}
		if(mode_hit)  break;

	}
	/*below the smallest mode*/
	if(mode_hit && 1==j && 0==i) 
	{
		pr_dbg("memory size below standard mode\r\n");
	}
	else
	{	/*bigger than the highest mode */
		if (!mode_hit)
		{
			pr_dbg("hd mode has been selected\r\n" );
			var->xres=valid_tv_mode[TVMODE_MAX-1].xres;
			var->yres=valid_tv_mode[TVMODE_MAX-1].yres;
			var->xres_virtual=var->xres ;
			var->yres_virtual=var->yres ;
			var->bits_per_pixel=32;
			
			return ;
		}
		else //fall in standard mode .
		{
			pr_dbg("hit mode :i=%d,j=%d\r\n",i,j);
			var->xres=valid_tv_mode[i].xres;
			var->yres=valid_tv_mode[i].yres;
			var->xres_virtual=var->xres ;
			var->yres_virtual=var->yres ;
			var->bits_per_pixel=8*(j-1);  //retrace to previous one 
			return ;
		}
	}

	_construct_virtual_window(mem_len,var);	
	
	
	
		
}
static  void 
_adjust_xyres(struct myfb_dev *fbdev)
{
	struct fb_var_screeninfo *var=&fbdev->fb_info->var;

	_get_xyres_via_memsize(fbdev->fb_len,var) ;
}
#endif

static void __init
_fbdev_set_default(struct myfb_dev *fbdev,int index ,int allow_auto_adj)
{
    	/* setup default value */
	fbdev->fb_info->var = mydef_var[index];
	fbdev->fb_info->fix = mydef_fix;
	fbdev->bpp_type=fbdev->fb_info->var.bits_per_pixel ;

	/* TODO:
	 * call vout server to find out the best output vmode
	 * based on memory size.
	 * Or just remove such features.
	 */
#if 0
	if(allow_auto_adj)
	_adjust_xyres(fbdev);	
#endif
}
bpp_color_bit_define_t*	
_find_color_format(int  bpp)
{
	int i ;

	for (i=0;i<ARRAY_SIZE(default_color_format_array);i++)
	{
		//pr_dbg("current color format value:%d\r\n",default_color_format_array[i].type_index);
		if(default_color_format_array[i].type_index==bpp)
		{
			return (bpp_color_bit_define_t*)&default_color_format_array[i];
		}
	}
	return NULL;
}
static int
apollofb_check_var(struct fb_var_screeninfo *var, struct fb_info *info)
{
	struct fb_fix_screeninfo *fix;
    	struct myfb_dev *fbdev=( struct myfb_dev*)info->par;
	bpp_color_bit_define_t   *color_format_pt;

	pr_dbg("apollofb_check_var\n");
	if(var->xres_virtual*var->yres_virtual*var->bits_per_pixel/8> fbdev->fb_len )
	{
		pr_dbg("no enough memory for %d*%d*%d\r\n",var->xres,var->yres,var->bits_per_pixel);
		return  -ENOMEM;
	}
	
    	fix = &info->fix;
	
		
	if ( (color_format_pt=_find_color_format(var->bits_per_pixel)) == NULL)
	{
		return -EFAULT ;
	}
	pr_dbg("select color format :index%d,bpp %d\r\n",color_format_pt->type_index, \
												color_format_pt->bpp) ;
	fbdev->bpp_type=var->bits_per_pixel ;
	var->red.offset = color_format_pt->red_offset;
	var->red.length = color_format_pt->red_length;
	var->red.msb_right= color_format_pt->red_msb_right ;
	var->green.offset  = color_format_pt->green_offset;
	var->green.length  = color_format_pt->green_length;
	var->green.msb_right = color_format_pt->green_msb_right;
	var->blue.offset   = color_format_pt->blue_offset;
	var->blue.length   = color_format_pt->blue_length;
	var->blue.msb_right = color_format_pt->blue_msb_right;
	var->transp.offset= color_format_pt->transp_offset ;
	var->transp.length = color_format_pt->transp_length ;
	var->transp.msb_right = color_format_pt->transp_msb_right ;
	var->bits_per_pixel=color_format_pt->bpp ;
	fix->visual=color_format_pt->color_type ;
	//adjust memory length.	
 	//fix->smem_len = var->xres_virtual*var->yres_virtual*var->bits_per_pixel/8;	
	fix->line_length = var->xres_virtual*var->bits_per_pixel/8;

    if (var->xres_virtual < var->xres)
        var->xres_virtual = var->xres;

    if (var->yres_virtual < var->yres)
        var->yres_virtual = var->yres;

    var->left_margin = var->right_margin = var->upper_margin = var->lower_margin = 0;
    

	if (var->xres + var->xoffset > var->xres_virtual)
		var->xoffset = var->xres_virtual - var->xres;
	if (var->yres + var->yoffset > var->yres_virtual)
		var->yoffset = var->yres_virtual - var->yres;
    
    return 0;
}

static int
apollofb_set_par(struct fb_info *info)
{
	struct myfb_dev *fbdev = (struct myfb_dev *)info->par;
	osd_ctl_t *osd_ctrl; 
	const vinfo_t *vinfo ;
	u32  end;
	 
	pr_dbg("++++++++apollofb_set_par\n");
	
	osd_ctrl=&fbdev->osd_ctl;
	if ( (vinfo=get_current_vinfo()) == NULL )
    	{
    	   return -EPERM;
    	}
	 
	if(info->var.xres  > vinfo->width)
	{
		osd_ctrl->disp_end_x= vinfo->width - 1 ;
	}
	else
	{
		end=osd_ctrl->disp_start_x+info->var.xres  ;
		end=(end>vinfo->width?vinfo->width:end); 
		osd_ctrl->disp_end_x=end -1;
	}
	if(info->var.yres  >vinfo->height)
	{
		osd_ctrl->disp_end_y=vinfo->height - 1;
	}
	else
	{     
		end=osd_ctrl->disp_start_y+info->var.yres;
		end=(end>vinfo->height?vinfo->height:end); 
		osd_ctrl->disp_end_y=end - 1; 
	}
	pr_dbg("start:end(x:%d-%d y:%d-%d)\n",osd_ctrl->disp_start_x,osd_ctrl->disp_end_x, \
										osd_ctrl->disp_start_y,osd_ctrl->disp_end_y);
	apollodev_set((struct myfb_dev *)info->par);
       return  0;
}

static int
apollofb_setcolreg(unsigned regno, unsigned red, unsigned green, unsigned blue,
        unsigned transp, struct fb_info *info)
{
    return apollodev_setcolreg(regno, red, green, blue,
                        transp, (struct myfb_dev *)info->par);
}

static int
apollofb_setcmap(struct fb_cmap *cmap, struct fb_info *info)
{
	int count, index, r;
	u16 *red, *green, *blue, *transp;
	u16 trans = 0xffff;

	red     = cmap->red;
	green   = cmap->green;
	blue    = cmap->blue;
	transp  = cmap->transp;
	index   = cmap->start;

	for (count = 0; count < cmap->len; count++) {
		if (transp)
			trans = *transp++;
		r = apollodev_setcolreg(index++, *red++, *green++, *blue++, trans,
				(struct myfb_dev *)info->par);
		if (r != 0)
			return r;
	}

	return 0;
}

static  bool   check_cmd_support(unsigned int cmd)
{
    return    	(cmd == FBIOPUT_OSD_SRCCOLORKEY) ||
           		(cmd == FBIOPUT_OSD_SRCKEY_ENABLE) ;
       
}
static int
apollofb_ioctl(struct fb_info *info, unsigned int cmd,
               unsigned long arg)
{
	 struct myfb_dev *fbdev = (struct myfb_dev *)info->par;
	 void __user *argp = (void __user *)arg;
   	 u32  src_colorkey;//16 bit or 24 bit 
   	 u32    srckey_enable;


	if (! check_cmd_support(cmd))
    	{
     	   pr_err("command not supported\r\n ");
     	   return -1;
    	}
    	switch (cmd)
  	 {
   		case  FBIOPUT_OSD_SRCKEY_ENABLE:
			copy_from_user(&srckey_enable,argp,sizeof(u32));
			break;
   		case  FBIOPUT_OSD_SRCCOLORKEY:
			copy_from_user(&src_colorkey,argp,sizeof(u32));
			break ;
		default :
			return -1;
	  }
	 mutex_lock(&fbdev->lock);

  	  switch (cmd)
    	{
    	  case FBIOPUT_OSD_SRCCOLORKEY:
	    switch(fbdev->bpp_type)
	  	{
	 		case BPP_TYPE_16_655:
			case BPP_TYPE_16_844:
			case BPP_TYPE_16_565:
			case BPP_TYPE_24_888_B:
			case BPP_TYPE_24_RGB:
	  	   	pr_dbg("set osd color key 0x%x\r\n",src_colorkey);
	  	 	osd_set_colorkey(info->node,fbdev->bpp_type,src_colorkey);
			break;
			default: break;
	  	 }
	   break ;
	  case FBIOPUT_OSD_SRCKEY_ENABLE:
		  switch(fbdev->bpp_type)
	  	{
	 		case BPP_TYPE_16_655:
			case BPP_TYPE_16_844:
			case BPP_TYPE_16_565:
			case BPP_TYPE_24_888_B:
			case BPP_TYPE_24_RGB:
			pr_dbg("set osd color key %s\r\n",srckey_enable?"enable":"disable");
		   	osd_srckey_enable(info->node,srckey_enable!=0?1:0);	
			break;
			default:break;
	 	 }
	   break;
	 
    }

    mutex_unlock(&fbdev->lock);
	
	return  0;
}
static int apollofb_open(struct fb_info *info, int arg)
{
	return 0;
	
}


static int
apollofb_blank(int blank_mode, struct fb_info *info)
{
      pr_dbg("osd%d\r\n"	,info->node);
    apollodev_enable((blank_mode != 0) ? 0 : 1,info->node);

    return 0;
}

static int apollofb_pan_display(struct fb_var_screeninfo *var,
                        struct fb_info *fbi)
{
	
    	apollodev_pan_display((struct myfb_dev *)fbi->par);
	pr_dbg("apollofb_pan_display:=>osd%d\r\n"	,fbi->node);
	return 0;
}

static int apollofb_sync(struct fb_info *info)
{
     	
     return 0;
}


/* fb_ops structures */
static struct fb_ops apollofb_ops = {
    .owner          = THIS_MODULE,
    .fb_check_var   = apollofb_check_var,
    .fb_set_par     = apollofb_set_par,
    .fb_setcolreg   = apollofb_setcolreg,
    .fb_setcmap     = apollofb_setcmap,
    .fb_fillrect    = cfb_fillrect,
    .fb_copyarea    =  cfb_copyarea,
    .fb_imageblit   = cfb_imageblit,
#ifdef CONFIG_FB_SOFT_CURSOR
    .fb_cursor      = soft_cursor,
#endif
    .fb_ioctl       = apollofb_ioctl,
    .fb_open      = apollofb_open,
    .fb_blank       = apollofb_blank,
    .fb_pan_display = apollofb_pan_display,
	.fb_sync	    = apollofb_sync,
};

void  set_default_display_axis(struct fb_var_screeninfo *var,osd_ctl_t *osd_ctrl, const vinfo_t *vinfo)
{
	int i ,found=0;
	u32  end;
	
	for(i=0;i<ARRAY_SIZE(disp_offset);i++)
	{
		if(disp_offset[i].vmode==vinfo->mode)
		{
			osd_ctrl->disp_start_x=disp_offset[i].disp_start_x;
			osd_ctrl->disp_start_y=disp_offset[i].disp_start_y;
			found=1;
			break ;
		}
	}
	if (!found)  
	{//set  unadjust value.
		osd_ctrl->disp_start_x=0;
		osd_ctrl->disp_start_y=0;
		if(var->xres > vinfo->width)
		{
			osd_ctrl->disp_end_x=vinfo->width- 1 ;//screen axis 
		}
		else
		{
			osd_ctrl->disp_end_x=var->xres- 1 ;//screen axis 
		}
		if(var->yres > vinfo->height)
		{
			osd_ctrl->disp_end_y=vinfo->height- 1 ;
		}
		else
		{
			osd_ctrl->disp_end_y=var->yres- 1 ;//screen axis 
		}
		return ;
	}
	//if found default offset value
	end = osd_ctrl->disp_start_x+var->xres;
	end=(end >vinfo->width?vinfo->width:end);
	osd_ctrl->disp_end_x=end -1;

	end = osd_ctrl->disp_start_y+var->yres;
	end=(end >vinfo->height?vinfo->height:end);
	osd_ctrl->disp_end_y=end -1;

}

int osd_notify_callback(struct notifier_block *block, unsigned long cmd , void *para)
{
	const vinfo_t *vinfo;
	int  		   blank;
	pfb_list_t  d;
	rectangle_t  *disp_rect;
	list_head_t   *l ;
	osd_ctl_t	    *osd_ctrl;
	struct fb_var_screeninfo *var ;
	myfb_dev_t *fb_dev[]={NULL,NULL,NULL,NULL} ;
	int  i;

	vinfo = get_current_vinfo();
	pr_dbg("tv_server:vmode=%s\r\n", vinfo->name);
	
	if(cmd != VOUT_EVENT_MODE_CHANGE &&
	   cmd != VOUT_EVENT_OSD_BLANK &&
	   cmd != VOUT_EVENT_OSD_DISP_AXIS)  return  -1;
	else
	{
		list_for_each(l,&fbdev_list_head)
		{
			d = list_entry(l, fb_list_t, list);
			if (d==NULL || d->fbdev==NULL ) continue ;
			fb_dev[d->fbdev->fb_info->node]=d->fbdev ;
		}
		switch (cmd)
		{
			case  VOUT_EVENT_MODE_CHANGE:
			pr_dbg("recevie change mode  message \r\n");
			for(i=0;i<OSD_COUNT;i++)
			{
				osd_ctrl=&fb_dev[i]->osd_ctl;
				var=&fb_dev[i]->fb_info->var ;
				set_default_display_axis(var,osd_ctrl,vinfo);
				pr_dbg("disp axis: x:%d y:%d w:%d h:%d\r\n"  , \
						osd_ctrl->disp_start_x, osd_ctrl->disp_start_y,\
						osd_ctrl->disp_end_x,osd_ctrl->disp_end_y);
				apollodev_set(fb_dev[i]);
			}
			break ;
			case  VOUT_EVENT_OSD_BLANK:
			blank=*(int*)para ;	
			for(i=0;i<OSD_COUNT;i++)
			{
				apollofb_blank(blank,fb_dev[i]->fb_info);
			}
			break ;
			case   VOUT_EVENT_OSD_DISP_AXIS:
			disp_rect=(rectangle_t*)para;	
			
			for(i=0;i<OSD_COUNT;i++)
			{	
				if(!disp_rect)  break;
				osd_ctrl=&fb_dev[i]->osd_ctl;
				osd_ctrl->disp_start_x=disp_rect->x  ;
				osd_ctrl->disp_start_y=disp_rect->y  ;
				pr_dbg("set disp axis: x:%d y:%d w:%d h:%d\r\n"  , \
						osd_ctrl->disp_start_x, osd_ctrl->disp_start_y,\
						disp_rect->w, disp_rect->h );
				
				if(osd_ctrl->disp_start_x+disp_rect->w>vinfo->width)
				{
					osd_ctrl->disp_end_x=vinfo->width - 1;
				}
				else
				{
					osd_ctrl->disp_end_x=osd_ctrl->disp_start_x+disp_rect->w -1 ; 
				}
				if(osd_ctrl->disp_start_y + disp_rect->h>vinfo->height)
				{
					osd_ctrl->disp_end_y=vinfo->height- 1;
				}
				else
				{
					osd_ctrl->disp_end_y=osd_ctrl->disp_start_y + disp_rect->h - 1 ;
				}
			
				disp_rect ++;
				pr_dbg("new disp axis: startx:%d starty:%d endx:%d endy:%d\r\n"  , \
						osd_ctrl->disp_start_x, osd_ctrl->disp_start_y,\
						osd_ctrl->disp_end_x,osd_ctrl->disp_end_y);
				apollodev_set(fb_dev[i]);
			}
			
			break;
		}
		return  0;
	}
}
static struct notifier_block osd_notifier_nb = {
	.notifier_call	= osd_notify_callback,
};
static ssize_t show_debug(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", debug_enable);
}

//control var by sysfs .
static ssize_t store_debug(struct device *dev, struct device_attribute *attr,
				    const char *buf, size_t count)
{
	int state;

	if (sscanf(buf, "%u", &state) != 1)
		return -EINVAL;

	if ((state != 1) && (state != 0))
		return -EINVAL;

	mutex_lock(&dbg_mutex);
	if (state != debug_enable) {
		debug_enable= state;
		if(debug_enable)
		{
			pr_dbg=normal_printk;
			pr_err=normal_printk;
		}	
		else
		{
			pr_dbg=mute_printk;
			pr_err=mute_printk;
		}
	}
	mutex_unlock(&dbg_mutex);

	return strnlen(buf, count);
}
static struct device_attribute device_attrs[] = {
	__ATTR(debug, S_IRUGO|S_IWUSR, show_debug, store_debug),
	__ATTR_NULL,	
};

static int __init
apollofb_probe(struct platform_device *pdev)
{
	int r;
    	struct myfb_dev *fbdev = NULL;
    	struct fb_info *fbi=NULL;
    	vmode_t vmode;
    	struct fb_var_screeninfo *var;
    	struct fb_fix_screeninfo *fix;
	struct resource *mem;
	int  index,Bpp;
	pfb_list_t  fbdev_item;
	logo_osd_dev_t  *init_fbdev=NULL;
	int  allow_auto_adj;
	int  logo_osd_index=0;
	
    	INIT_LIST_HEAD(&fbdev_list_head);

    	vout_register_client(&osd_notifier_nb);

#ifdef CONFIG_FB_AML_LOGO
	init_fbdev = get_init_fbdev();
	logo_osd_index= init_fbdev->config.osd_ctl.index ;
#endif

   	if (init_fbdev)
    	{
		BUG_ON(init_fbdev->vmode >= VMODE_MAX);

    		vmode=init_fbdev->vmode  ;
    	}
    	else
    	{
    		vmode=VMODE_720P;
    	}

	set_current_vmode(vmode);
	
    	for (index=0;index<OSD_COUNT;index++)
    	{
    		//platform resource 
		if (!(mem = platform_get_resource(pdev, IORESOURCE_MEM, index))) {
			pr_err("No frame buffer memory define.\n");
			r = -EFAULT;
			goto failed2;
		}
		//if we have no resource then no need to create this device.
		if (!mem || mem->start== 0 || mem->end==0 || mem->start==mem->end)
		{
			continue ;
		}
	
    		fbi = framebuffer_alloc(sizeof(struct myfb_dev), &pdev->dev);
    		if(!fbi) {
        		r = -ENOMEM;
        		goto failed1;
    		}
	
	fbdev = (struct myfb_dev *)fbi->par;
	fbdev->fb_info = fbi;
	fbdev->dev = pdev;
 	
	mutex_init(&fbdev->lock);

    	var = &fbi->var;
    	fix = &fbi->fix;

	fbdev_item=(pfb_list_t)kmalloc(sizeof(fb_list_t),GFP_KERNEL);
	fbdev_item->fbdev=fbdev;
	list_add(&fbdev_item->list,&fbdev_list_head);
	
	
	fbdev->fb_mem = mem->start;
	fbdev->fb_len = mem->end - mem->start + 1;
	//clear framebuffer memory
	pr_dbg("Frame buffer memory assigned at 0x%08x, size=%dK\n",
	    fbdev->fb_mem, fbdev->fb_len >> 10);
	allow_auto_adj=1;
	if(init_fbdev && index==logo_osd_index ) //adjust default var info
	{
		int  bpp=init_fbdev->bpp;//bytes per pixel
		
		Bpp=(bpp >8?(bpp>16?(bpp>24?4:3):2):1);
		if(init_fbdev->config.osd_ctl.xres_virtual*init_fbdev->config.osd_ctl.yres_virtual*Bpp <= fbdev->fb_len )
		{
			mydef_var[index].xres=init_fbdev->config.osd_ctl.xres;
			mydef_var[index].yres=init_fbdev->config.osd_ctl.yres;	
			mydef_var[index].xres_virtual=init_fbdev->config.osd_ctl.xres_virtual;
			mydef_var[index].yres_virtual=init_fbdev->config.osd_ctl.yres_virtual;
			mydef_var[index].bits_per_pixel=bpp ;
			allow_auto_adj=0;
			pr_dbg("init fbdev bpp is :%d\r\n",mydef_var[index].bits_per_pixel);
		}	
		if(mydef_var[index].bits_per_pixel>32) 
		{
			mydef_var[index].bits_per_pixel=32;
		}
		
	}else{
		
		memset((char*)fbdev->fb_mem,0,fbdev->fb_len);	
	}
	
	_fbdev_set_default(fbdev,index,0);
	Bpp=(fbdev->bpp_type >8?(fbdev->bpp_type>16?(fbdev->bpp_type>24?4:3):2):1);
	fix->line_length=var->xres_virtual*Bpp;
	fix->smem_start = fbdev->fb_mem;
	fix->smem_len = fbdev->fb_len;


	set_default_display_axis(var,&fbdev->osd_ctl,get_current_vinfo());
	
	
	  
	if (fb_alloc_cmap(&fbi->cmap, 16, 0) != 0) {
		pr_err("unable to allocate color map memory\n");
      		r = -ENOMEM;
        	goto failed3;
    	}

	if (!(fbi->pseudo_palette = kmalloc(sizeof(u32) * 16, GFP_KERNEL))) {
		pr_err("unable to allocate pseudo palette memory\n");
        	r = -ENOMEM;
        	goto failed4;
	}
	memset(fbi->pseudo_palette, 0, sizeof(u32) * 16);

   	fbi->fbops = &apollofb_ops;
    	fbi->screen_base = (char __iomem *)fix->smem_start ;
	fbi->screen_size = fix->smem_len;
	

	
    	apollofb_check_var(var, fbi);
    	register_framebuffer(fbi);
	if(NULL==init_fbdev )//to see logo ,dont change this line 
	{
		apollodev_set(fbdev);
	}
		
    	}	
 	dev_set_drvdata(&pdev->dev, &fbdev_list_head);
	index=0;
	while(attr_name(device_attrs[index]))
	{
		r= device_create_file(&pdev->dev, &device_attrs[index]);

		if (r)// remove all device file which already been created.
		{
			while(--index>=0)
			{
				device_remove_file(&pdev->dev, &device_attrs[index]);
			}
			device_attrs[0].attr.name=NULL;
			break;
		}
			
		index++;
	}

	
       pr_dbg("apollofb probe ok  \r\n");
	return 0;

failed4:
	fb_dealloc_cmap(&fbi->cmap);
failed3:
	dev_set_drvdata(&pdev->dev, NULL);
failed2:
    unregister_framebuffer(fbi);
failed1:

    pr_err("Driver module insert failed.\n");

    return r;
}

static int
apollofb_remove(struct platform_device *pdev)
{
#if 0
    free_irq(pdev->resource[0].start, pdev);
#endif
   	struct fb_info *fbi;
	pfb_list_t  d;
	list_head_t   *l ,*t;	
	list_head_t    *head;
	int i=0;

    	pr_dbg("apollofb_remove.\n");
	if (!pdev)
		return -ENODEV;
	
	while(attr_name(device_attrs[i]))
	{
		device_remove_file(&pdev->dev, &device_attrs[i++]);	
	}
	head = (list_head_t*) dev_get_drvdata(&pdev->dev);
	vout_unregister_client(&osd_notifier_nb);
	
	
	list_for_each_safe(l,t,head)
	{
		d = list_entry(l, fb_list_t, list);
		if(d->fbdev!=NULL)
		{
		  fbi = d->fbdev->fb_info;
      		  kfree(fbi->pseudo_palette);
     		  fb_dealloc_cmap(&fbi->cmap);
         	  unregister_framebuffer(fbi);
		  framebuffer_release(fbi);
		}
		list_del(l);
		if(d)
		kfree(d) ;
	}
	 

    	return 0;
}

#ifndef MODULE

/* Process kernel command line parameters */
static int __init
apollofb_setup(char *options)
{
    char *this_opt = NULL;
    int r = 0;

    if (!options || !*options)
        goto exit;

    while (!r && (this_opt = strsep(&options, ",")) != NULL) {
        if (!strncmp(this_opt, "xres:", 5))
            mydef_var[0].xres = mydef_var[0].xres_virtual = simple_strtoul(this_opt + 5, NULL, 0);
        else if (!strncmp(this_opt, "yres:", 5))
            mydef_var[0].yres = mydef_var[0].yres_virtual = simple_strtoul(this_opt + 5, NULL, 0);
        else {
            pr_err("invalid option\n");
            r = -1;
        }
    }
exit:
    return r;
}

#endif

/****************************************/

static struct platform_driver
apollofb_driver = {
    .probe      = apollofb_probe,
    .remove     = apollofb_remove,
    .driver     = {
        .name   = "apollofb",
    }
};

static int __init
apollofb_init_module(void)
{
	debug_enable=0;
	pr_dbg=mute_printk;
	pr_err=mute_printk;
	pr_dbg("apollofb_init\n");

#ifndef MODULE
    {
      	char *option;
      if (fb_get_options("apollofb", &option)) {
           return -ENODEV;
   	}
	  
   	apollofb_setup(option);
    }
#endif

    if (platform_driver_register(&apollofb_driver)) {
        pr_err("failed to register apollofb driver\n");
        return -ENODEV;
    }

    return 0;
}

static void __exit
apollofb_remove_module(void)
{
    pr_err("apollofb_remove_module.\n");

    platform_driver_unregister(&apollofb_driver);
}

/****************************************/

module_param(vsync_cnt, uint, 0664);
MODULE_PARM_DESC(vsync_cnt,
    "\n vsync_cnt desc \n"
      "VSYNC count \n");

module_param_named(xres, def_xres, long, 0664);
MODULE_PARM_DESC(xres, "\n X resolution \n");

module_param_named(yres, def_yres, long, 0664);
MODULE_PARM_DESC(yres, "\n Y resolution \n");

module_init(apollofb_init_module);
module_exit(apollofb_remove_module);

MODULE_DESCRIPTION("AMLOGIC APOLLO framebuffer driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Tim Yao <timyao@amlogic.com>");
