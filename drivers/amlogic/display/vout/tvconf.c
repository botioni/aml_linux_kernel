/*
 * Amlogic Apollo
 * tv display control driver
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
 * Author:   jianfeng_wang@amlogic
 *		   
 *		   
 */

#include <linux/version.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/io.h>
#include <linux/mm.h>
#include <linux/interrupt.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/ctype.h>
#include <linux/vout/vinfo.h>
#include <mach/am_regs.h>
#include <asm/uaccess.h>
#include <linux/major.h>
#include "tvconf.h"
#include "tvmode.h"
#include "tv_log.h"
#include <linux/amlog.h>

MODULE_AMLOG(AMLOG_DEFAULT_LEVEL, 0, LOG_LEVEL_DESC, LOG_MASK_DESC);
static  DEFINE_MUTEX(tvconf_module_mutex)  ;

#define DEFAULT_VMODE	VMODE_720P
static    disp_module_info_t    *info;

//deivce  attribute
SET_DISP_DEVICE_ATTR(tv_enable,func_default_null)
SET_DISP_DEVICE_ATTR(tv_mode,func_default_null)
SET_DISP_DEVICE_ATTR(hdmi_enable,func_default_null)
SET_DISP_DEVICE_ATTR(hdmi_mode,func_default_null)
//class attribute
SET_DISP_CLASS_ATTR(enable,func_default_null)
SET_DISP_CLASS_ATTR(mode,set_disp_mode)
SET_DISP_CLASS_ATTR(axis,set_disp_window)
SET_DISP_CLASS_ATTR(vdac_setting,parse_vdac_setting)
SET_DISP_CLASS_ATTR(wr_reg,write_reg)
SET_DISP_CLASS_ATTR(rd_reg,read_reg)

/*****************************
*	default settings :
*	Y    -----  DAC1
*	PB  -----  DAC2
*	PR  -----  DAC0
*
*	CVBS  	---- DAC1
*	S-LUMA    ---- DAC2
*	S-CHRO	----  DAC0
******************************/
static  unsigned  int  vdac_sequence=0x120120;
static const tvmode_t vmode_tvmode_tab[] =
{
	TVMODE_480I, TVMODE_480CVBS,TVMODE_480P, TVMODE_576I,TVMODE_576CVBS, TVMODE_576P, TVMODE_720P, TVMODE_1080I, TVMODE_1080P
};


static const vinfo_t tv_info[] = 
{
    { /* VMODE_480I */
		.name              = "480i",
		.mode              = VMODE_480I,
        .width             = 720,
        .height            = 480,
        .field_height      = 240,
        .aspect_ratio_num  = 4,
        .aspect_ratio_den  = 3,
        .sync_duration_num = 60,
        .sync_duration_den = 1,
    },
     { /* VMODE_480CVBS*/
		.name              = "480cvbs",
		.mode              = VMODE_480CVBS,
        .width             = 720,
        .height            = 480,
        .field_height      = 240,
        .aspect_ratio_num  = 4,
        .aspect_ratio_den  = 3,
        .sync_duration_num = 60,
        .sync_duration_den = 1,
    },
    { /* VMODE_480P */
		.name              = "480p",
		.mode              = VMODE_480P,
        .width             = 720,
        .height            = 480,
        .field_height      = 480,
        .aspect_ratio_num  = 4,
        .aspect_ratio_den  = 3,
        .sync_duration_num = 60,
        .sync_duration_den = 1,
    },
    { /* VMODE_576I */
		.name              = "576i",
		.mode              = VMODE_576I,
        .width             = 720,
        .height            = 576,
        .field_height      = 288,
        .aspect_ratio_num  = 4,
        .aspect_ratio_den  = 3,
        .sync_duration_num = 50,
        .sync_duration_den = 1,
    },
    { /* VMODE_576I */
		.name              = "576cvbs",
		.mode              = VMODE_576CVBS,
        .width             = 720,
        .height            = 576,
        .field_height      = 288,
        .aspect_ratio_num  = 4,
        .aspect_ratio_den  = 3,
        .sync_duration_num = 50,
        .sync_duration_den = 1,
    },
    { /* VMODE_576P */
		.name              = "576p",
		.mode              = VMODE_576P,
        .width             = 720,
        .height            = 576,
        .field_height      = 576,
        .aspect_ratio_num  = 4,
        .aspect_ratio_den  = 3,
        .sync_duration_num = 50,
        .sync_duration_den = 1,
    },
    { /* VMODE_720P */
		.name              = "720p",
		.mode              = VMODE_720P,
        .width             = 1280,
        .height            = 720,
        .field_height      = 720,
        .aspect_ratio_num  = 16,
        .aspect_ratio_den  = 9,
        .sync_duration_num = 60,
        .sync_duration_den = 1,
    },
    { /* VMODE_1080I */
		.name              = "1080i",
		.mode              = VMODE_1080I,
        .width             = 1920,
        .height            = 1080,
        .field_height      = 540,
        .aspect_ratio_num  = 16,
        .aspect_ratio_den  = 9,
        .sync_duration_num = 60,
        .sync_duration_den = 1,
    },
    { /* VMODE_1080P */
		.name              = "1080p",
		.mode              = VMODE_1080P,
        .width             = 1920,
        .height            = 1080,
        .field_height      = 1080,
        .aspect_ratio_num  = 16,
        .aspect_ratio_den  = 9,
        .sync_duration_num = 60,
        .sync_duration_den = 1,
    },
};

static  struct  class_attribute   *disp_attr[]={
&class_display_attr_enable,
&class_display_attr_mode,	
&class_display_attr_axis ,
&class_display_attr_vdac_setting,
&class_display_attr_wr_reg,
&class_display_attr_rd_reg,
};

#define    OSD_OFF     \
		blank=1;		\
		vout_notifier_call_chain(VOUT_EVENT_OSD_BLANK,&blank) ;	

#define    OSD_ON	\
		 blank=0;	\
		vout_notifier_call_chain(VOUT_EVENT_OSD_BLANK,&blank) ;	

static const struct file_operations am_tv_fops = {
	.open	= NULL,  
	.read	= NULL,//am_tv_read, 
	.write	= NULL, 
	.ioctl	= NULL,//am_tv_ioctl, 
	.release	= NULL, 	
	.poll		= NULL,
};

static const vinfo_t *get_valid_vinfo(char  *mode)
{
	int  i,count=ARRAY_SIZE(tv_info);
	
	for(i=0;i<count;i++)
	{
		if(strncmp(tv_info[i].name,mode,strlen(tv_info[i].name))==0)
		{
			return &tv_info[i];
		}
	}
	return NULL;
}

static const vinfo_t *tv_get_current_info(void)
{
	return info->vinfo;
}

static int tv_set_current_vmode(vmode_t mod)
{
	if (mod > VMODE_1080P)
		return -EINVAL;

	tvoutc_setmode(vmode_tvmode_tab[mod]);
	change_vdac_setting(vdac_sequence,mod);
	info->vinfo = &tv_info[mod];
	strcpy(mode,tv_info[mod].name);
	return 0;
}

static vmode_t tv_validate_vmode(char *mode)
{
	const vinfo_t *info = get_valid_vinfo(mode);
	
	if (info)
		return info->mode;
	
	return VMODE_MAX;
}

static vout_server_t tv_server={
	.name = "vout_tv_server",
	.op = {	
		.get_vinfo=tv_get_current_info,
		.set_vmode=tv_set_current_vmode,
		.validate_vmode=tv_validate_vmode,
	},
};

static  void   func_default_null(char  *str)
{
	return ;
}

static   int* parse_para(char *para,char   *para_num)
{
	 static unsigned   int  buffer[MAX_NUMBER_PARA] ; 
	 char  *endp ;
	 int *pt=NULL;
	 int len=0,count=0;

	if(!para) return NULL;
	memset(buffer,0,sizeof(int)*MAX_NUMBER_PARA);
	pt=&buffer[0];
	len=strlen(para);
	endp=(char*)buffer;
	do
	{
		//filter space out 
		while(para && ( isspace(*para) || !isalnum(*para)) && len)
		{
			para++;
			len --; 
		}
		if(len==0) break;
		*pt++=simple_strtoul(para,&endp,0);
		
		para=endp;
		len=strlen(para);
	}while(endp && ++count<*para_num&&count<MAX_NUMBER_PARA) ;
	*para_num=count;
	
	return  buffer;
}

static  void  read_reg(char *para)
{
	char  count=1;
	tv_reg_t  reg;

	memcpy(&reg.addr,parse_para(para+1,&count),sizeof(unsigned int));

	if (((*para) == 'm') || ((*para) == 'M'))
	{
		amlog_level(LOG_LEVEL_HIGH,"[0x%x] : 0x%x\r\n", reg.addr, READ_MPEG_REG(reg.addr));
	}else if (((*para) == 'p') || ((*para) == 'P')) {
		if (APB_REG_ADDR_VALID(reg.addr))
		amlog_level(LOG_LEVEL_HIGH,"[0x%x] : 0x%x\r\n", reg.addr, READ_APB_REG(reg.addr));
	}
}

static  void  write_reg(char *para)
{
	char  count=2;
	tv_reg_t  reg;

	memcpy(&reg,parse_para(para+1,&count),sizeof(tv_reg_t));

	if (((*para) == 'm') || ((*para) == 'M'))
		WRITE_MPEG_REG(reg.value, reg.addr);
	else if (((*para) == 'p') || ((*para) == 'P')) {
		if (APB_REG_ADDR_VALID(reg.addr))
			WRITE_APB_REG(reg.value, reg.addr);
	}		
}

/***************************************************
*
*	The first digit control component Y output DAC number
*	The 2nd digit control component U output DAC number
*	The 3rd digit control component V output DAC number
*	The 4th digit control composite CVBS output DAC number
*	The 5th digit control s-video Luma output DAC number
*	The 6th digit control s-video chroma output DAC number
* 	examble :
*		echo  120120 > /sys/class/display/vdac_setting
*		the first digit from the left side .	
******************************************************/
static void  parse_vdac_setting(char *para) 
{
	int  i;
	char  *pt=strstrip(para);
	int len=strlen(pt);

	amlog_mask_level(LOG_MASK_PARA,LOG_LEVEL_LOW,"origin vdac setting:0x%x,strlen:%d\n",vdac_sequence,len);
	if(len!=6)
	{
		amlog_mask_level(LOG_MASK_PARA,LOG_LEVEL_HIGH,"can't parse vdac settings\n");
		return ;
	}
	vdac_sequence=0;
	for(i=0;i<6;i++)
	{
		vdac_sequence<<=4;
		vdac_sequence|=*pt -'0';
		pt++;
	}
	amlog_mask_level(LOG_MASK_PARA,LOG_LEVEL_LOW,"current vdac setting:0x%x\n",vdac_sequence);
	change_vdac_setting(vdac_sequence,info->vinfo->mode);
}
	


static  void  set_disp_mode(char *mode)
{
	const vinfo_t *tvinfo;
	int  		  blank ;

	amlog_mask_level(LOG_MASK_PARA,LOG_LEVEL_LOW,"tvmode set to %s\r\n",mode);
	if( (tvinfo=get_valid_vinfo(mode))== NULL)
	{
		amlog_mask_level(LOG_MASK_PARA,LOG_LEVEL_HIGH,"invalid tvmode \r\n");
		return  ;
	}
	if(tvinfo==info->vinfo)
	{
		amlog_mask_level(LOG_MASK_PARA,LOG_LEVEL_HIGH,"don't set the same mode as current.\r\n");
	}
	else
	{
		info->vinfo=tvinfo;
		OSD_OFF  ;
		tvoutc_setmode(vmode_tvmode_tab[tvinfo->mode]);
		change_vdac_setting(vdac_sequence,tvinfo->mode);
		amlog_mask_level(LOG_MASK_PARA,LOG_LEVEL_LOW,"new mode %s set ok\r\n",mode);
		vout_notifier_call_chain(VOUT_EVENT_MODE_CHANGE,(vmode_t *)&tvinfo->mode) ;
		OSD_ON   ;
	}
		
}
//axis type : 0x12  0x100 0x120 0x130
static void  set_disp_window(char *para) 
{
#define   OSD_COUNT   2
	char  count=OSD_COUNT*4;	
	int  blank;
	int   *pt=&info->disp_rect[0].x;
	

	//parse window para .
	memcpy(pt,parse_para(para,&count),sizeof(disp_rect_t)*OSD_COUNT);
	
	if(count >=4 && count <8 )
	{
		info->disp_rect[1]=info->disp_rect[0] ;
	}
	amlog_mask_level(LOG_MASK_PARA,LOG_LEVEL_LOW,"osd0=>x:%d ,y:%d,w:%d,h:%d\r\n osd1=> x:%d,y:%d,w:%d,h:%d \r\n", \
			*pt,*(pt+1),*(pt+2),*(pt+3),*(pt+4),*(pt+5),*(pt+6),*(pt+7));
	OSD_OFF;
	vout_notifier_call_chain(VOUT_EVENT_OSD_DISP_AXIS,&info->disp_rect) ;
	OSD_ON;
}

static int  create_disp_attr(disp_module_info_t* info)
{
	//create base class for display
	int  i;
	const char  *device_name[]={"tv","hdmi"};
	info->base_class=class_create(THIS_MODULE,info->name);
	if(IS_ERR(info->base_class))
	{
		amlog_mask_level(LOG_MASK_INIT,LOG_LEVEL_HIGH,"create tv display class fail\r\n");
		return  -1 ;
	}
	//create  class attr
	for(i=0;i<DISP_ATTR_MAX;i++)
	{
		if ( class_create_file(info->base_class,disp_attr[i]))
		{
			amlog_mask_level(LOG_MASK_INIT,LOG_LEVEL_HIGH,"create disp attribute %s fail\r\n",disp_attr[i]->attr.name);
		}
	}
	//create  class device  ,for mdev
	for (i=0;i<ENC_TYPE_MAX;i++)
	{
		info->device[i]=device_create(info->base_class,NULL,MKDEV(info->major,i+1),NULL,device_name[i]);
	}
	sprintf(vdac_setting,"%x",vdac_sequence);
	return   0;
}

static int  disp_module_init(disp_module_info_t* info)
{
	//create tv  attr
	if (create_disp_attr(info))
	{
		return -1;
	}

	return 0;
}

static int __init display_init_module(void)
{
	int  ret ;

	info=(disp_module_info_t*)kmalloc(sizeof(disp_module_info_t),GFP_KERNEL) ;
	if (!info)
	{
		amlog_mask_level(LOG_MASK_INIT,LOG_LEVEL_HIGH,"can't alloc display info struct\r\n");
		return -ENOMEM;
	}
	
	memset(info, 0, sizeof(disp_module_info_t));

	sprintf(info->name,DISPLAY_CLASS_NAME) ;
	ret=register_chrdev(TV_CONF_MAJOR,info->name,&am_tv_fops);
	if(ret <0) 
	{
		amlog_mask_level(LOG_MASK_INIT,LOG_LEVEL_HIGH,"register char dev tv error\r\n");
		return  ret ;
	}
	info->major=TV_CONF_MAJOR;
	amlog_mask_level(LOG_MASK_INIT,LOG_LEVEL_HIGH,"major number %d for disp\r\n",ret);
	if ( ! disp_module_init(info) )
	{
		amlog_mask_level(LOG_MASK_INIT,LOG_LEVEL_HIGH,"display module init ok \r\n");
	}
	else
	{
		amlog_mask_level(LOG_MASK_INIT,LOG_LEVEL_HIGH,"display module init fail \r\n");
	}

	if(vout_register_server(&tv_server))
	{
		amlog_mask_level(LOG_MASK_INIT,LOG_LEVEL_HIGH,"register tv module server fail \r\n");
	}
	else
	{
		amlog_mask_level(LOG_MASK_INIT,LOG_LEVEL_HIGH,"register tv module server ok \r\n");
	}
	return 0;

}
static __exit void display_exit_module(void)
{
	int i;
	
	if(info)
	{
		for (i=0;i<ENC_TYPE_MAX;i++)
		{
			if(info->device[i])
			{
				device_destroy(info->base_class,MKDEV(info->major,i+1));
			}
		}
		for(i=0;i<DISP_ATTR_MAX;i++)
		{
			class_remove_file(info->base_class,disp_attr[i]) ;
		}
		class_destroy(info->base_class);
		unregister_chrdev(info->major,info->name)	;
		kfree(info);
	}
	vout_unregister_server();
	
	amlog_mask_level(LOG_MASK_INIT,LOG_LEVEL_HIGH,"exit tv module\r\n");
}

#if defined(CONFIG_JPEGLOGO) || defined(CONFIG_AM_LOGO)
subsys_initcall(display_init_module);
#else
module_init(display_init_module);
#endif
module_exit(display_exit_module);

MODULE_DESCRIPTION("display configure  module");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("jianfeng_wang <jianfeng.wang@amlogic.com>");

