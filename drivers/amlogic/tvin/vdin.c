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
//#include <linux/etherdevice.h>
#include <linux/interrupt.h>
#include <linux/errno.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/time.h>
#include <linux/mm.h>

/* Amlogic headers */
#include <linux/amports/canvas.h>
#include <mach/am_regs.h>
#include <linux/amports/vframe.h>
#include <linux/tvin/tvin.h>

/* TVIN headers */
#include "tvin_global.h"
#include "tvin_format_table.h"
#include "tvin_notifier.h"
#include "vdin_regs.h"
#include "vdin.h"
#include "vdin_vf.h"
#include "vdin_ctl.h"


#define VDIN_NAME               "vdin"
#define VDIN_DRIVER_NAME        "vdin"
#define VDIN_MODULE_NAME        "vdin"
#define VDIN_DEVICE_NAME        "vdin"
#define VDIN_CLASS_NAME         "vdin"

#if defined(CONFIG_ARCH_MESON)
#define VDIN_COUNT              1
#elif defined(CONFIG_ARCH_MESON2)
#define VDIN_COUNT              2
#endif

#define VDIN_PUT_INTERVAL       1    //(HZ/100)   //10ms, #define HZ 100

#define INVALID_VDIN_INPUT      0xffffffff

static dev_t vdin_devno;
static struct class *vdin_clsp;

static vdin_dev_t *vdin_devp[VDIN_COUNT];
//static vframe_t *cur_vdin_vf = NULL;

void tvin_dec_register(struct vdin_dev_s *devp, struct tvin_dec_ops_s *op)
{
    ulong flags;

    if (devp->decop)
        tvin_dec_unregister(devp);

    spin_lock_irqsave(&devp->declock, flags);

    devp->decop = op;

    spin_unlock_irqrestore(&devp->declock, flags);
}

void tvin_dec_unregister(struct vdin_dev_s *devp)
{
    ulong flags;
    spin_lock_irqsave(&devp->declock, flags);

    devp->decop = NULL;

    spin_unlock_irqrestore(&devp->declock, flags);
}

void vdin_info_update(struct vdin_dev_s *devp, struct tvin_parm_s *para)
{
    //check decoder signal status
    if((para->status != TVIN_SIG_STATUS_STABLE) || (para->fmt == TVIN_SIG_FMT_NULL))
        return;

    devp->para.status = para->status;
    devp->para.fmt = para->fmt;

    //write vdin registers
    vdin_set_all_regs(devp);

}

EXPORT_SYMBOL(vdin_info_update);

static void vdin_put_timer_func(unsigned long arg)
{
    struct timer_list *timer = (struct timer_list *)arg;

    while (!vfq_empty_recycle()) {
        vframe_t *vf = vfq_pop_recycle();
        vfq_push_newframe(vf);
    }

    tvin_check_notifier_call(TVIN_EVENT_INFO_CHECK, NULL);

    timer->expires = jiffies + VDIN_PUT_INTERVAL;
    add_timer(timer);
}

static void vdin_start_dec(struct vdin_dev_s *devp)
{
    vdin_vf_init();
    vdin_reg_vf_provider();


    vdin_set_default_regmap(devp->index);

    tvin_dec_notifier_call(TVIN_EVENT_DEC_START, devp);
	return;
}

static void vdin_stop_dec(struct vdin_dev_s *devp)
{
    vdin_unreg_vf_provider();
    //load default setting for vdin
    vdin_set_default_regmap(devp->index);
    tvin_dec_notifier_call(TVIN_EVENT_DEC_STOP, devp);
}

//static u32 vdin_isr_hard_counter = 0;
//static u32 vdin_isr_workueue_counter = 0;
/*as use the spin_lock,
 *1--there is no sleep,
 *2--it is better to shorter the time,
 *3--it is better to shorter the time,
*/
static irqreturn_t vdin_isr(int irq, void *dev_id)
{
    struct vdin_dev_s *devp = (struct vdin_dev_s *)dev_id;
    int vdin_irq_flag = 0;
    ulong flags;
    vframe_t *vf = NULL;
    spin_lock_irqsave(&devp->declock, flags);
//    struct timeval now;
//    do_gettimeofday(&now);
//    vdin_isr_hard_counter++;
//    if((vdin_isr_hard_counter % 3600) == 0)
//        pr_info("vdin_isr: vdin_isr_hard_counter = %d \n", vdin_isr_hard_counter);
    vf = vfq_pop_newframe();
    if(vf == NULL)
    {
//        pr_info("vdin_isr: don't get newframe \n");
        spin_unlock_irqrestore(&devp->declock, flags);
        return IRQ_HANDLED;
    }

    if(devp->decop && devp->decop->dec_run) {
        vf->type = INVALID_VDIN_INPUT;
        vf->pts = 0;
        vdin_irq_flag = devp->decop->dec_run(vf);
        if(vdin_irq_flag == 0)
        {
            if (devp->workqueue != NULL)
                queue_work(devp->workqueue, &devp->dec_work);
            else
                schedule_work(&devp->dec_work);
        }
    }
    else
       pr_err("vdin_isr:dec_run is NULL \n");
    spin_unlock_irqrestore(&devp->declock, flags);
    return IRQ_HANDLED;
}

/*as use the spin_lock,
 *1--there is no sleep,
 *2--it is better to shorter the time,
 *3--it is better to shorter the time,
*/
static void vdin_isr_wq(struct work_struct *work)
{
    struct vdin_dev_s *devp = container_of(work, struct vdin_dev_s, dec_work);
    vframe_t *cur_vdin_vf = NULL;

    spin_lock_bh(&devp->declock);

    /* pop out a new frame to be displayed */
//    struct timeval now;
//    unsigned long int temp_t = 0;
//    do_gettimeofday(&now);

//    vdin_irq_irq_list_time = (unsigned long int)((now.tv_sec  * 1000000)+ (now.tv_usec  ));

//    vdin_isr_workueue_counter++;
//    if((vdin_isr_workueue_counter % 3600) == 0)
//        pr_info("vdin_isr_wq: vdin_isr_workueue_counter = %d \n", vdin_isr_workueue_counter);
    cur_vdin_vf = vfq_get_curframe();
    if(cur_vdin_vf == NULL)
    {
//        pr_info("vdin_isr_wq: don't get cur_vdin_vf \n");
        spin_unlock_bh(&devp->declock);
        return;  //IRQ_HANDLED;
    }

    if(devp->decop && devp->decop->dec_run_bh){
        devp->decop->dec_run_bh(cur_vdin_vf);
    }
    else{
        pr_info("vdin: decop is null\n");
        spin_unlock_bh(&devp->declock);
        return;
    }


    //If cur_vdin_vf->type ( --reture value )is INVALID_VDIN_INPUT, the current field is error
    if(cur_vdin_vf->type == INVALID_VDIN_INPUT)
    {
        /* mpeg12 used spin_lock_irqsave(), @todo... */
        vfq_push_recycle(cur_vdin_vf);
    }
    else
    {
//        vdin_set_vframe_prop_info(cur_vdin_vf, devp->index);
        vfq_push_display(cur_vdin_vf); /* push to display */
    }
    spin_unlock_bh(&devp->declock);

    return;
}

static int vdin_open(struct inode *inode, struct file *file)
{
    vdin_dev_t *devp;

    /* Get the per-device structure that contains this cdev */
    devp = container_of(inode->i_cdev, vdin_dev_t, cdev);
    file->private_data = devp;

	if (devp->index >= VDIN_COUNT)
        return -ENXIO;

    return 0;
}

static int vdin_release(struct inode *inode, struct file *file)
{
    vdin_dev_t *devp = file->private_data;
    file->private_data = NULL;

    //printk(KERN_INFO "vdin: device %d release ok.\n", devp->index);
    return 0;
}

static inline bool vdin_port_valid(enum tvin_port_e port)
{
    bool ret = false;

#if defined(CONFIG_ARCH_MESON)
    switch (port>>8)
    {
        case 0x01: // mpeg
        case 0x02: // 656
        case 0x80: // dvin
            ret = true;
            break;
        default:
            break;
    }
#elif defined(CONFIG_ARCH_MESON2)
    switch (port>>8)
    {
        case 0x01: // mpeg
        case 0x02: // 656
        case 0x04: // VGA
        case 0x08: // COMPONENT
        case 0x10: // CVBS
        case 0x20: // SVIDEO
        case 0x40: // hdmi
        case 0x80: // dvin
            ret = true;
        default:
            break;
    }
#endif
    return ret;
}

static int vdin_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
    int ret = 0;
    vdin_dev_t *devp;
    void __user *argp = (void __user *)arg;

	if (_IOC_TYPE(cmd) != TVIN_IOC_MAGIC) {
		return -EINVAL;
	}

    devp = container_of(inode->i_cdev, vdin_dev_t, cdev);

    switch (cmd)
    {
        case TVIN_IOC_START_DEC:
        {
            struct tvin_parm_s para;
            pr_info("vdin%d: TVIN_IOC_START_DEC (0x%x)\n", devp->index, devp->flags);
            if(devp->flags & VDIN_FLAG_DEC_STARTED)
            {
                pr_err("vdin%d: decode started already\n", devp->index);
                ret = -EINVAL;
                break;
            }

            if (copy_from_user(&para, argp, sizeof(struct tvin_parm_s)))
            {
                pr_err("vdin%d: invalid paramenter\n", devp->index);
                ret = -EFAULT;
                break;
            }

            if (!vdin_port_valid(para.port))
            {
                pr_err("vdin%d: not supported port 0x%x\n", devp->index, para.port);
                ret = -EFAULT;
                break;
            }

            //init vdin signal info
            devp->para.port = para.port;
            devp->para.fmt = para.fmt;
            devp->para.status = TVIN_SIG_STATUS_NULL;
            devp->para.cap_addr = 0x85100000;
            devp->flags |= VDIN_FLAG_DEC_STARTED;
            vdin_start_dec(devp);
            pr_info("vdin%d: TVIN_IOC_START_DEC ok\n", devp->index);
#if defined(CONFIG_ARCH_MESON2)
            vdin_set_meas_mux(devp);
#endif
            msleep(10);
            enable_irq(devp->irq);
            pr_info("vdin%d: irq is enabled.\n", devp->index);

            break;
        }
        case TVIN_IOC_STOP_DEC:
        {
            pr_info("vdin%d: TVIN_IOC_STOP_DEC (flags:0x%x)\n", devp->index, devp->flags);
            if(!(devp->flags & VDIN_FLAG_DEC_STARTED))
            {
                pr_err("vdin%d: can't stop, decode havn't started\n", devp->index);
                ret = -EINVAL;
                break;
            }
            disable_irq_nosync(devp->irq);
            vdin_stop_dec(devp);
            devp->flags &= (~VDIN_FLAG_DEC_STARTED);

            break;
        }
        case TVIN_IOC_G_PARM:
        {
		    if (copy_to_user((void __user *)arg, &devp->para, sizeof(struct tvin_parm_s)))
		    {
               ret = -EFAULT;
		    }
            break;
        }

        case TVIN_IOC_S_PARM:
        {
            struct tvin_parm_s para = {TVIN_PORT_NULL, TVIN_SIG_FMT_NULL, TVIN_SIG_STATUS_NULL, 0, 0, 0};
            if (copy_from_user(&para, argp, sizeof(struct tvin_parm_s)))
		    {
                ret = -EFAULT;
                break;
            }
            //get tvin port selection and other setting
            devp->para.flag = para.flag;
            devp->para.cap_addr = 0;  //reset frame capture address 0 means null data
            if(devp->para.port != para.port)
            {
                //to do

		    }
            break;
        }
        default:
            ret = -ENOIOCTLCMD;
            break;
    }

    return ret;
}

static int vdin_mmap(struct file *file, struct vm_area_struct * vma)
{
    vdin_dev_t *devp = file->private_data;
	unsigned long off, pfn;
	unsigned long start, size;
    u32 len;

    unsigned long t1, t2, t3;

    if (!devp)
        return -ENODEV;
    if (vma->vm_pgoff > (~0UL >> PAGE_SHIFT))
        return -EINVAL;
        
 //   pr_info("%s \n", __func__);
    if(!(devp->para.flag &  TVIN_PARM_FLAG_CAP))
	{
    	pr_err("don't set capture flag to vdin \n");
    	return -EINVAL;
    }
    	
   // pr_info("cap_addr = 0x%x; cap_size = %d\n", devp->para.cap_addr, devp->para.cap_size);

	off = vma->vm_pgoff << PAGE_SHIFT;

//    pr_info("vm_pgoff = 0x%lx\n", vma->vm_pgoff);
//    pr_info("off      = 0x%lx = (vm_pgoff << PAGE_SHIFT)\n", off);

    mutex_lock(&devp->mm_lock);
	start = devp->para.cap_addr;
	len = PAGE_ALIGN((start & ~PAGE_MASK) + devp->para.cap_size);
    mutex_unlock(&devp->mm_lock);

//    pr_info("start    = 0x%lx\n", start);

    t1 = (start & ~PAGE_MASK);
//    pr_info("t1       = 0x%lx = (start & ~PAGE_MASK) \n", t1);

    t2 = t1 + devp->para.cap_size;
//    pr_info("t2       = 0x%lx = (t1 + cap_size)        \n", t2);
//    pr_info("len      = 0x%x = PAGE_ALIGN(t2)\n", len);

	start &= PAGE_MASK;
//    pr_info("start    = 0x%lx = (start & PAGE_MASK)\n", start);

//    pr_info("vm_start = 0x%lx; vm_end = 0x%lx\n", vma->vm_start, vma->vm_end);

	if ((vma->vm_end - vma->vm_start + off) > len)
		return -EINVAL;
	off += start;
//    pr_info("off      = 0x%lx = (off += start)\n", off);

	vma->vm_pgoff = off >> PAGE_SHIFT;
//    pr_info("vm_pgoff = 0x%lx = (off >> PAGE_SHIFT)\n", vma->vm_pgoff);

	vma->vm_flags |= VM_IO | VM_RESERVED;
    size = vma->vm_end - vma->vm_start;
    pfn  = off >> PAGE_SHIFT;

//    pr_info("size     = 0x%lx = (vm_end - vm_start)\n", size);
//    pr_info("pfn      = 0x%lx = (off >> PAGE_SHIFT)\n", pfn);

	if (io_remap_pfn_range(vma, vma->vm_start, pfn, size, vma->vm_page_prot))
		return -EAGAIN;
	return 0;
}


static struct file_operations vdin_fops = {
    .owner   = THIS_MODULE,
    .open    = vdin_open,
    .release = vdin_release,
    .ioctl   = vdin_ioctl,
    .mmap    = vdin_mmap,
};


static int vdin_probe(struct platform_device *pdev)
{
    int ret;
    int i;
    struct device *devp;
    struct resource *res;
    char name[12];

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

//        vdin_devp[i]->declock = SPIN_LOCK_UNLOCKED;    //old_style_spin_init 
        spin_lock_init(&vdin_devp[i]->declock);
        vdin_devp[i]->decop = NULL;

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


        /* get device memory */
        res = platform_get_resource(pdev, IORESOURCE_MEM, i);
        if (!res) {
            printk(KERN_ERR "vdin: can't get memory resource\n");
            return -EFAULT;
        }
        vdin_devp[i]->mem_start = res->start;
        vdin_devp[i]->mem_size  = res->end - res->start + 1;
        pr_info(" vdin[%d] memory start addr is %x, mem_size is %x . \n",i,
            vdin_devp[i]->mem_start,vdin_devp[i]->mem_size);

        /* get device irq */
        res = platform_get_resource(pdev, IORESOURCE_IRQ, i);
        if (!res) {
            printk(KERN_ERR "vdin: can't get memory resource\n");
            return -EFAULT;
        }
        vdin_devp[i]->irq = res->start;
        vdin_devp[i]->flags = VDIN_FLAG_NULL;
        pr_info("vdin%d: flags:0x%x\n", vdin_devp[i]->index, vdin_devp[i]->flags);

        vdin_devp[i]->addr_offset = 0;
        vdin_devp[i]->para.flag = 0;

        sprintf(name, "vdin%d-irq", i);
        /* register vdin irq */
        ret = request_irq(vdin_devp[i]->irq, vdin_isr, IRQF_SHARED, name, (void *)vdin_devp[i]);
        if (ret) {
            printk(KERN_ERR "vdin: irq regist error.\n");
            return -ENOENT;
        }
        disable_irq(vdin_devp[i]->irq);
        init_timer(&vdin_devp[i]->timer);
        vdin_devp[i]->timer.data = (ulong) &vdin_devp[i]->timer;
        vdin_devp[i]->timer.function = vdin_put_timer_func;
        vdin_devp[i]->timer.expires = jiffies + VDIN_PUT_INTERVAL * 50;
        add_timer(&vdin_devp[i]->timer);
        vdin_devp[i]->workqueue = create_singlethread_workqueue(VDIN_DRIVER_NAME);
//        vdin_devp[i]->workqueue = create_workqueue(VDIN_DRIVER_NAME);

        mutex_init(&vdin_devp[i]->mm_lock);

        INIT_WORK(&vdin_devp[i]->dec_work, vdin_isr_wq);
    }

    printk(KERN_INFO "vdin: driver initialized ok\n");
    return 0;
}

static int vdin_remove(struct platform_device *pdev)
{
    int i = 0;



    for (i = 0; i < VDIN_COUNT; ++i)
    {
        mutex_destroy(vdin_devp[i]->mm_lock);
        if (vdin_devp[i]->workqueue != NULL)
            destroy_workqueue(vdin_devp[i]->workqueue);
        del_timer_sync(&vdin_devp[i]->timer);
        free_irq(vdin_devp[i]->irq,(void *)vdin_devp[i]);
        del_timer(&vdin_devp[i]->timer);
        device_destroy(vdin_clsp, MKDEV(MAJOR(vdin_devno), i));
        cdev_del(&vdin_devp[i]->cdev);
        kfree(vdin_devp[i]);
    }
    class_destroy(vdin_clsp);
    unregister_chrdev_region(vdin_devno, VDIN_COUNT);

    printk(KERN_ERR "vdin: driver removed ok.\n");
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

