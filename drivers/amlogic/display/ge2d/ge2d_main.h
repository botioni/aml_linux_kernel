#ifndef  _GE2D_MAIN_H
#define  _GE2D_MAIN_H
#include "ge2d.h"
#include "pr_dbg.h"
#include <linux/interrupt.h>
#include <asm/arch/am_regs.h>
#include <linux/amports/canvas.h>
#include <linux/osd/apollofbdev.h>
#include <linux/fb.h>
#include <linux/list.h>
#include <asm/uaccess.h>
#include <linux/sysfs.h>
#include  <linux/spinlock.h>
#include <linux/kthread.h>


/**************************************************************
**																	 **
**	macro define		 												 **
**																	 **
***************************************************************/

#define    	GE2D_CLASS_NAME   				"ge2d"
#define  	 	GE2D_STRETCHBLIT_NOALPHA   	0x4702
#define  		GE2D_BLIT_NOALPHA	 			0x4701
#define  		GE2D_BLEND			 			0x4700
#define  		GE2D_BLIT    			 		0x46ff
#define  		GE2D_STRETCHBLIT   				0x46fe
#define  		GE2D_FILLRECTANGLE   			0x46fd
#define  		GE2D_SRCCOLORKEY   			0x46fc
#define  		GE2D_CONFIG					0x46f9

/**************************************************************
**																	 **
**	type  define		 												 **
**																	 **
***************************************************************/

typedef  struct {
	char  			name[20];
	unsigned int 		open_count;
	int	 			major;
	unsigned  int 		dbg_enable;
	struct class 		*cla;
	struct device		*dev;
}ge2d_device_t;

/**************************************************************
**																	 **
**function define & declare												 **
**																	 **
***************************************************************/

static int ge2d_open(struct inode *inode, struct file *file) ;
static int ge2d_ioctl(struct inode *inode, struct file *filp, unsigned int cmd, unsigned long args) ;
static int ge2d_release(struct inode *inode, struct file *file);

static ssize_t ge2d_debug_show(struct class *cla, char *buf) ;
static ssize_t ge2d_debug_store(struct class *cla, const char *buf,
                                size_t count);
extern ssize_t work_queue_status_show(struct class *cla, char *buf) ;
extern ssize_t free_queue_status_show(struct class *cla, char *buf);
/**************************************************************
**																	 **
**	varible define		 												 **
**																	 **
***************************************************************/

static  ge2d_device_t  ge2d_device;
static DEFINE_MUTEX(ge2d_mutex);
static const struct file_operations ge2d_fops = {
	.owner		= THIS_MODULE,
	.open		=ge2d_open,  
	.ioctl		= ge2d_ioctl,
	.release		= ge2d_release, 	
};
static struct class_attribute ge2d_class_attrs[] = {
    __ATTR(debug,
           S_IRUGO | S_IWUSR,
           ge2d_debug_show,
           ge2d_debug_store),
    __ATTR(wq_status,
           S_IRUGO | S_IWUSR,
           work_queue_status_show,
           NULL),
    __ATTR(fq_status,
           S_IRUGO | S_IWUSR,
           free_queue_status_show,
           NULL),       
    __ATTR_NULL
};
static struct class ge2d_class = {
    .name = GE2D_CLASS_NAME,
    .class_attrs = ge2d_class_attrs,
};

#endif
