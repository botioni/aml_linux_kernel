/*******************************************************************
 *
 *  Copyright C 2010 by Amlogic, Inc. All Rights Reserved.
 *
 *  Description:
 *
 *  Author: Amlogic Software
 *  Created: 2010/4/1   19:46
 *
 *******************************************************************/
#include "ge2d_main.h"


/***********************************************************************
*
* sysfs   section 
*
************************************************************************/

static ssize_t ge2d_debug_show(struct class *cla,struct class_attribute *attr, char *buf) 
{
	return sprintf(buf, "%u\n", ge2d_device.dbg_enable);
}

//control var by sysfs .

static ssize_t ge2d_debug_store(struct class *cla, struct class_attribute *attr,const char *buf,
                                size_t count)
{
	int state;

	if (sscanf(buf, "%u", &state) != 1)
		return -EINVAL;

	if ((state != 1) && (state != 0))
		return -EINVAL;

	mutex_lock(&ge2d_mutex);
	if (state != ge2d_device.dbg_enable) {
		ge2d_device.dbg_enable = state;
		if(ge2d_device.dbg_enable)
			pr_dbg=normal_printk;
		else
			pr_dbg=mute_printk;
	}
	mutex_unlock(&ge2d_mutex);

	return strnlen(buf, count);
}
/***********************************************************************
*
* file op section.
*
************************************************************************/
static  bool   command_valid(unsigned int cmd)
{
    return (cmd <= GE2D_STRETCHBLIT_NOALPHA && cmd >= GE2D_CONFIG );
}
static int 
ge2d_open(struct inode *inode, struct file *file) 
{
	 ge2d_context_t *context;
	 //we create one ge2d workqueue for this file handler.
	 
	 if(NULL==(context=create_ge2d_work_queue()))
	 {
	 	printk("can't create work queue \r\n");
		return -1;		
	 }
	 pr_dbg("open one ge2d device\n");
	 file->private_data=context;
	 ge2d_device.open_count++;
	 return 0;
}
static int
ge2d_ioctl(struct inode *inode, struct file *filp,
                 unsigned int cmd, unsigned long args)
{

	ge2d_context_t *context=(ge2d_context_t *)filp->private_data;
	void  __user* argp =(void __user*)args;
	config_para_t     ge2d_config;	
	ge2d_para_t  para ;
	int  ret=0;    	

	if(!command_valid(cmd))  return -1;
	switch (cmd)
   	{
		case  GE2D_CONFIG:
		copy_from_user(&ge2d_config,argp,sizeof(config_para_t));
		break;
		default :
		copy_from_user(&para,argp,sizeof(ge2d_para_t));	
		break;
		
   	}
	switch(cmd)
	{
		case GE2D_CONFIG:
		ret=ge2d_context_config(context,&ge2d_config) ;
	  	break;
    	  	case GE2D_SRCCOLORKEY:
		ge2dgen_src_key(context , 1,para.color, 0x0);  //RGBA MODE		
		break;
		case GE2D_FILLRECTANGLE:
		pr_dbg("fill rect...,x=%d,y=%d,w=%d,h=%d,color=0x%x\r\n",
                   para.src1_rect.x, para.src1_rect.y,
                   para.src1_rect.w, para.src1_rect.h,
                   para.color);

            fillrect(context,
                     para.src1_rect.x, para.src1_rect.y,
                     para.src1_rect.w, para.src1_rect.h,
                     para.color) ;	
		break;
		case GE2D_STRETCHBLIT:
		//stretch blit
            	pr_dbg("stretchblt...,x=%d,y=%d,w=%d,h=%d,dst.w=%d,dst.h=%d\r\n",
                   para.src1_rect.x, para.src1_rect.y,
                   para.src1_rect.w, para.src1_rect.h,
                   para.dst_rect.w, para.dst_rect.h);

            	stretchblt(context ,
                       para.src1_rect.x, para.src1_rect.y, para.src1_rect.w, para.src1_rect.h,
                       para.dst_rect.x,  para.dst_rect.y,  para.dst_rect.w,  para.dst_rect.h);	
		break;
		case GE2D_BLIT:
		//bitblt
            	pr_dbg("blit...\r\n");

            	bitblt(context ,
                   para.src1_rect.x, para.src1_rect.y,
                   para.src1_rect.w, para.src1_rect.h,
                   para.dst_rect.x, para.dst_rect.y);
           	break;
		case GE2D_BLEND:
		pr_dbg("blend ...\r\n");
		blend(context,
            		para.src1_rect.x, para.src1_rect.y,
            		para.src1_rect.w, para.src1_rect.h,
           		para.src2_rect.x, para.src2_rect.y,
           		para.src2_rect.w, para.src2_rect.h,
           		para.dst_rect.x, para.dst_rect.y,
           		para.dst_rect.w, para.dst_rect.h,
           		para.op) ;	
		break;
		case GE2D_BLIT_NOALPHA:
		//bitblt_noalpha
            	pr_dbg("blit_noalpha...\r\n");
            	bitblt_noalpha(context ,
                   para.src1_rect.x, para.src1_rect.y,
                   para.src1_rect.w, para.src1_rect.h,
                   para.dst_rect.x, para.dst_rect.y);	
		break;
		case GE2D_STRETCHBLIT_NOALPHA:
		//stretch blit
            	pr_dbg("stretchblt_noalpha...,x=%d,y=%d,w=%d,h=%d,dst.w=%d,dst.h=%d\r\n",
                   para.src1_rect.x, para.src1_rect.y,
                   para.src1_rect.w, para.src1_rect.h,
                   para.dst_rect.w, para.dst_rect.h);

            	stretchblt_noalpha(context ,
                       para.src1_rect.x, para.src1_rect.y, para.src1_rect.w, para.src1_rect.h,
                       para.dst_rect.x,  para.dst_rect.y,  para.dst_rect.w,  para.dst_rect.h);	
		break;
	}
 	return ret;
}
static int 
ge2d_release(struct inode *inode, struct file *file)
{
	ge2d_context_t *context=(ge2d_context_t *)file->private_data;
	
	if(context && (0==destroy_ge2d_work_queue(context)))
	{
		ge2d_device.open_count--;
		return 0;
	}
	pr_dbg("release one ge2d device\n");
	return -1;
}


/***********************************************************************
*
* module  section    (init&exit)
*
************************************************************************/
static int  
init_ge2d_device(void)
{
	int  ret=0;
	
	strcpy(ge2d_device.name,"ge2d");
	ret=register_chrdev(0,ge2d_device.name,&ge2d_fops);
	if(ret <=0) 
	{
		printk("register ge2d device error\r\n");
		return  ret ;
	}
	ge2d_device.major=ret;
	ge2d_device.dbg_enable=0;
	pr_dbg=mute_printk;
	printk("ge2d_dev major:%d\r\n",ret);
	ret = class_register(&ge2d_class);
	if(ret<0 )
	{
		printk("error create ge2d class\r\n");
		return ret;
	}
	ge2d_device.cla=&ge2d_class ;
	ge2d_device.dev=device_create(ge2d_device.cla,NULL,MKDEV(ge2d_device.major,0),NULL,ge2d_device.name);
	if (IS_ERR(ge2d_device.dev)) {
		printk("create ge2d device error\n");
		class_unregister(ge2d_device.cla);
		return -1 ;
	}
	return ge2d_setup();
	
}
static int remove_ge2d_device(void)
{
	if(ge2d_device.cla)
	{
		if(ge2d_device.dev)
		device_destroy(ge2d_device.cla, MKDEV(ge2d_device.major, 0));
	    	class_unregister(ge2d_device.cla);
	}
	
	unregister_chrdev(ge2d_device.major, ge2d_device.name);
	ge2d_deinit();
	return  0;
}

static int __init
ge2d_init_module(void)
{
   	printk("ge2d_init\n");
    	return init_ge2d_device();
    	
}

static void __exit
ge2d_remove_module(void)
{
	remove_ge2d_device();
    	printk("ge2d module removed.\n");
    
}

module_init(ge2d_init_module);
module_exit(ge2d_remove_module);

MODULE_DESCRIPTION("AMLOGIC APOLLO ge2d driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("jianfeng <jianfeng.wang@amlogic.com>");


