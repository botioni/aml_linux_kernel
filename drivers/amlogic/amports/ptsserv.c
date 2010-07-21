#include <linux/module.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/list.h>
#include <linux/spinlock.h>
#include <linux/slab.h>
#include <linux/amports/ptsserv.h>
#include <linux/amports/timestamp.h>

#include <mach/am_regs.h>

//#define DEBUG_VIDEO
//#define DEBUG_AUDIO
//#define DEBUG_CHECKIN
//#define DEBUG_CHECKOUT

#define VIDEO_REC_SIZE  4096
#define AUDIO_REC_SIZE  4096
#define VIDEO_LOOKUP_RESOLUTION 2500
#define AUDIO_LOOKUP_RESOLUTION 1024

enum {
    PTS_IDLE       = 0,
    PTS_INIT       = 1,
    PTS_LOADING    = 2,
    PTS_RUNNING    = 3,
    PTS_DEINIT     = 4
};

typedef struct pts_rec_s {
    struct list_head list;
    u32 offset;
    u32 val;
} pts_rec_t;

typedef struct pts_table_s {
    u32 status;
    int rec_num;
    int lookup_threshold;
    u32 lookup_cache_offset;
    bool lookup_cache_valid;
    u32 lookup_cache_pts;
    u32 wr_pages;
    u32 wr_ptr;
    u32 rd_pages;
    u32 rd_ptr;
    u32 buf_start;
    u32 buf_size;
    pts_rec_t *pts_recs;
    struct list_head *pts_search;
    struct list_head valid_list;
    struct list_head free_list;
} pts_table_t;

static struct task_struct *pts_task = NULL;
static spinlock_t lock = SPIN_LOCK_UNLOCKED;

static pts_table_t pts_table[PTS_TYPE_MAX] =
{
    {.status = PTS_IDLE,
     .rec_num = VIDEO_REC_SIZE,
     .lookup_threshold = VIDEO_LOOKUP_RESOLUTION,
    },
    {.status = PTS_IDLE,
    .rec_num = AUDIO_REC_SIZE,
    .lookup_threshold = AUDIO_LOOKUP_RESOLUTION,
    },
};

/* to-do: a HW implentation? */
static int pts_thread(void *unused)
{
    unsigned p;

    printk(KERN_INFO "pts_thread up\n");

    while (!kthread_should_stop()) {
        pts_table_t *pTable;

        pTable = &pts_table[PTS_TYPE_VIDEO];

        p = READ_MPEG_REG(VLD_MEM_VIFIFO_RP);
        if (p < pTable->rd_ptr) {
            pTable->rd_pages++;

            WRITE_MPEG_REG(VLD_MEM_RD_PAGE, pTable->rd_pages);
            WRITE_MPEG_REG(VLD_MEM_RD_PAGE_RP, p);
        }
        pTable->rd_ptr = p;

        p = READ_MPEG_REG(VLD_MEM_VIFIFO_WP);
        if (p < pTable->wr_ptr)
            pTable->wr_pages++;
        pTable->wr_ptr = p;

        pTable = &pts_table[PTS_TYPE_AUDIO];

        p = READ_MPEG_REG(AIU_MEM_AIFIFO_MAN_RP);
        if (p < pTable->rd_ptr) {
            pTable->rd_pages++;

            WRITE_MPEG_REG(AIU_MEM_RD_PAGE, pTable->rd_pages);
            WRITE_MPEG_REG(AIU_MEM_RD_PAGE_RP, p);
        }
        pTable->rd_ptr = p;

        p = READ_MPEG_REG(AIU_MEM_AIFIFO_MAN_WP);
        if (p < pTable->wr_ptr)
            pTable->wr_pages++;
        pTable->wr_ptr = p;

        msleep(5);
    }

    printk(KERN_INFO "pts_thread quit\n");

    return 0;
}

int pts_checkin_offset(u8 type, u32 offset, u32 val)
{
    ulong flags;
    const u32 pts_reg[PTS_TYPE_MAX] = {VIDEO_PTS, AUDIO_PTS};
    pts_table_t *pTable;

    if (type >= PTS_TYPE_MAX)
        return -EINVAL;

    pTable = &pts_table[type];

    spin_lock_irqsave(&lock, flags);

    if (likely((pTable->status == PTS_RUNNING) ||
               (pTable->status == PTS_LOADING))) {
        pts_rec_t *rec;
#ifdef DEBUG_CHECKIN
#ifdef DEBUG_VIDEO
        if (type == PTS_TYPE_VIDEO)
            printk("check in vpts <0x%x:0x%x>\n", offset, val);
#endif
#ifdef DEBUG_AUDIO
        if (type == PTS_TYPE_AUDIO)
            printk("check in apts <0x%x:0x%x>\n", offset, val);
#endif
#endif
        if (list_empty(&pTable->free_list)) {
            rec = list_entry(pTable->valid_list.next, pts_rec_t, list);
        } else {
            rec = list_entry(pTable->free_list.next, pts_rec_t, list);
        }

        rec->offset = offset;
        rec->val = val;

        list_move_tail(&rec->list, &pTable->valid_list);

        spin_unlock_irqrestore(&lock, flags);

        if (pTable->status == PTS_LOADING) {
#ifdef DEBUG_VIDEO
        if (type == PTS_TYPE_VIDEO)
            printk("init vpts[%d] at 0x%x\n", type, val);
#endif
#ifdef DEBUG_VIDEO
        if (type == PTS_TYPE_AUDIO)
            printk("init apts[%d] at 0x%x\n", type, val);
#endif
            WRITE_MPEG_REG(pts_reg[type], val);
            pTable->status = PTS_RUNNING;
        }

        return 0;

    } else {
        spin_unlock_irqrestore(&lock, flags);

        return -EINVAL;
    }
}

EXPORT_SYMBOL(pts_checkin_offset);

int pts_checkin_pageoffset(u8 type, u32 page_offset, u32 val)
{
    ulong flags;
    u32 offset, page, page_no;

    if (type >= PTS_TYPE_MAX)
        return -EINVAL;

    spin_lock_irqsave(&lock, flags);

    page = pts_table[type].wr_pages;
    offset = pts_table[type].wr_ptr - pts_table[type].buf_start;

    spin_unlock_irqrestore(&lock, flags);

    page_no = (page_offset < offset) ? (page + 1) : page;

    return pts_checkin_offset(type,
        pts_table[type].buf_size * page_no + page_offset, val);
}

EXPORT_SYMBOL(pts_checkin_pageoffset);

int pts_checkin_wrptr(u8 type, u32 ptr, u32 val)
{
    ulong flags;
    u32 offset, page, page_no;

    if (type >= PTS_TYPE_MAX)
        return -EINVAL;

    spin_lock_irqsave(&lock, flags);

    page = pts_table[type].wr_pages;
    offset = pts_table[type].wr_ptr;

    spin_unlock_irqrestore(&lock, flags);

    /* to-do: sanity check for input ptr range */
    page_no = (ptr < offset) ? (page + 1) : page;

    return pts_checkin_offset(type,
        pts_table[type].buf_size * page_no + ptr - pts_table[type].buf_start, val);
}

EXPORT_SYMBOL(pts_checkin_wrptr);

int pts_checkin(u8 type, u32 val)
{
    if (type == PTS_TYPE_VIDEO)
        return pts_checkin_pageoffset(type,
            READ_MPEG_REG(VLD_MEM_VIFIFO_WP) - pts_table[type].buf_start, val);
    else if (type == PTS_TYPE_AUDIO)
        return pts_checkin_pageoffset(type,
            READ_MPEG_REG(AIU_MEM_AIFIFO_MAN_WP) - pts_table[type].buf_start, val);
    else
        return -EINVAL;
}

EXPORT_SYMBOL(pts_checkin);

int pts_lookup_pageoffset(u8 type, u32 page_offset, u32 *val, u32 pts_margin)
{
    ulong flags;
    u32 offset, page, page_no;

    if (type >= PTS_TYPE_MAX)
        return -EINVAL;

    spin_lock_irqsave(&lock, flags);

    page = pts_table[type].rd_pages;
    offset = pts_table[type].rd_ptr - pts_table[type].buf_start;

    spin_unlock_irqrestore(&lock, flags);

    page_no = (page_offset < offset) ? (page + 1) : page;

    return pts_lookup_offset(type,
        page_offset + pts_table[type].buf_size * page_no, val, pts_margin);
}

EXPORT_SYMBOL(pts_lookup_pageoffset);

int pts_lookup_offset(u8 type, u32 offset, u32 *val, u32 pts_margin)
{
    ulong flags;
    pts_table_t *pTable;
    int lookup_threshold;
#if defined(DEBUG_VIDEO) || defined(DEBUG_AUDIO)
    int look_cnt = 0;
#endif

    if (type >= PTS_TYPE_MAX)
        return -EINVAL;

    pTable = &pts_table[type];

    if (pts_margin == 0)
        lookup_threshold = pTable->lookup_threshold;
    else
        lookup_threshold = pts_margin;

    spin_lock_irqsave(&lock, flags);

    if (likely(pTable->status == PTS_RUNNING)) {
        pts_rec_t *p, *p2 = NULL;

        if ((pTable->lookup_cache_valid) &&
            (offset == pTable->lookup_cache_offset)) {
            spin_unlock_irqrestore(&lock, flags);

            *val = pTable->lookup_cache_pts;
            return 0;
        }

        if (list_empty(&pTable->valid_list)) {
            spin_unlock_irqrestore(&lock, flags);

            return -1;
        }

        if (pTable->pts_search == &pTable->valid_list)
            p = list_entry(pTable->valid_list.next, pts_rec_t, list);
        else
            p = list_entry(pTable->pts_search, pts_rec_t, list);

        if (p->offset < offset) {
            p2 = p; /* lookup candidate */

            list_for_each_entry_continue(p, &pTable->valid_list, list) {
#if 0
if (type == PTS_TYPE_VIDEO)
    printk("   >> rec: 0x%x\n", p->offset);
#endif
#if defined(DEBUG_VIDEO) || defined(DEBUG_AUDIO)
    look_cnt++;
#endif

                if (p->offset > offset) {
                    break;
                }

                p2 = p;
            }
        }
        else if (p->offset > offset) {
            list_for_each_entry_continue_reverse(p, &pTable->valid_list, list) {
#if 0
if (type == PTS_TYPE_VIDEO)
    printk("   >> rec: 0x%x\n", p->offset);
#endif
#ifdef DEBUG
    look_cnt++;
#endif
                if (p->offset <= offset) {
                    p2 = p;
                    break;
                }
            }
        }
        else
            p2 = p;

        if ((p2) &&
            ((offset - p2->offset) < lookup_threshold)) {
#ifdef DEBUG_CHECKOUT
#ifdef DEBUG_VIDEO
            if (type == PTS_TYPE_VIDEO)
                printk("vpts look up offset<0x%x> --> <0x%x:0x%x>, look_cnt = %d\n",
                    offset, p2->offset, p2->val, look_cnt);
#endif
#ifdef DEBUG_AUDIO
            if (type == PTS_TYPE_AUDIO)
                printk("apts look up offset<0x%x> --> <0x%x:0x%x>, look_cnt = %d\n",
                    offset, p2->offset, p2->val, look_cnt);
#endif
#endif
            *val = p2->val;

            pTable->lookup_cache_pts = *val;
            pTable->lookup_cache_offset = offset;
            pTable->lookup_cache_valid = true;

            /* update next look up search start point */
            pTable->pts_search = p2->list.next;

            list_move_tail(&p2->list, &pTable->free_list);

            spin_unlock_irqrestore(&lock, flags);

            return 0;

        } else {
#ifdef DEBUG_CHECKOUT
#ifdef DEBUG_VIDEO
            if (type == PTS_TYPE_VIDEO)
                printk("vpts look up offset<0x%x> failed, look_cnt = %d\n", offset, look_cnt);
#endif
#ifdef DEBUG_AUDIO
            if (type == PTS_TYPE_AUDIO)
                printk("apts look up offset<0x%x> failed, look_cnt = %d\n", offset, look_cnt);
#endif
#endif
            spin_unlock_irqrestore(&lock, flags);

            return -1;
        }
    }

    spin_unlock_irqrestore(&lock, flags);

    return -1;
}

EXPORT_SYMBOL(pts_lookup_offset);

int pts_set_resolution(u8 type, u32 level)
{
    if (type >= PTS_TYPE_MAX)
        return -EINVAL;

    pts_table[type].lookup_threshold = level;
    return 0;
}

EXPORT_SYMBOL(pts_set_resolution);

int pts_set_rec_size(u8 type, u32 val)
{
    ulong flags;

    if (type >= PTS_TYPE_MAX)
        return -EINVAL;

    spin_lock_irqsave(&lock, flags);

    if (pts_table[type].status == PTS_IDLE) {
        pts_table[type].rec_num = val;

        spin_unlock_irqrestore(&lock, flags);

        return 0;

    } else {
        spin_unlock_irqrestore(&lock, flags);

        return -EBUSY;
    }
}

EXPORT_SYMBOL(pts_set_rec_size);

int pts_start(u8 type)
{
    ulong flags;
    int i;
    pts_table_t *pTable;

    if (type >= PTS_TYPE_MAX)
        return -EINVAL;

    pTable = &pts_table[type];

    spin_lock_irqsave(&lock, flags);

    if (likely(pTable->status == PTS_IDLE)) {
        pTable->status = PTS_INIT;

        spin_unlock_irqrestore(&lock, flags);

        pTable->pts_recs = kcalloc(pTable->rec_num,
                            sizeof(pts_rec_t), GFP_KERNEL);

        if (!pTable->pts_recs) {
            pTable->status = 0;
            return -ENOMEM;
        }

        if (type == PTS_TYPE_VIDEO) {
            pTable->buf_start = READ_MPEG_REG(VLD_MEM_VIFIFO_START_PTR);
            pTable->buf_size = READ_MPEG_REG(VLD_MEM_VIFIFO_END_PTR)
                                 - pTable->buf_start + 8;

            WRITE_MPEG_REG(VIDEO_PTS, 0);

            WRITE_MPEG_REG(VLD_MEM_RD_PAGE, 0);
            WRITE_MPEG_REG(VLD_MEM_RD_PAGE_RP, 0);
        }
        else if (type == PTS_TYPE_AUDIO) {
            pTable->buf_start = READ_MPEG_REG(AIU_MEM_AIFIFO_START_PTR);
            pTable->buf_size = READ_MPEG_REG(AIU_MEM_AIFIFO_END_PTR)
                                 - pTable->buf_start + 8;

            WRITE_MPEG_REG(AUDIO_PTS, 0);
        }

        INIT_LIST_HEAD(&pTable->valid_list);
        INIT_LIST_HEAD(&pTable->free_list);

        for (i = 0; i < pTable->rec_num; i++)
            list_add_tail(&pTable->pts_recs[i].list, &pTable->free_list);

        pTable->wr_pages = 0;
        pTable->rd_pages = 0;
        pTable->wr_ptr = pTable->buf_start;
        pTable->rd_ptr = pTable->buf_start;
        pTable->pts_search = &pTable->valid_list;
        pTable->status = PTS_LOADING;
        pTable->lookup_cache_valid = false;

        printk("pts started\n");

        return 0;

    } else {
        spin_unlock_irqrestore(&lock, flags);

        return -EBUSY;
    }
}

EXPORT_SYMBOL(pts_start);

int pts_stop(u8 type)
{
    ulong flags;
    pts_table_t *pTable;

    if (type >= PTS_TYPE_MAX)
        return -EINVAL;

    pTable = &pts_table[type];

    spin_lock_irqsave(&lock, flags);

    if (likely((pTable->status == PTS_RUNNING) ||
               (pTable->status == PTS_LOADING))) {
        pTable->status = PTS_DEINIT;

        spin_unlock_irqrestore(&lock, flags);

        kfree(pTable->pts_recs);

        pTable->pts_recs = NULL;

        INIT_LIST_HEAD(&pTable->valid_list);
        INIT_LIST_HEAD(&pTable->free_list);

        pTable->status = PTS_IDLE;

        if (type == PTS_TYPE_AUDIO)
            timestamp_apts_set(-1);

        printk("pts stopped\n");

        return 0;

    } else {
        spin_unlock_irqrestore(&lock, flags);

        return -EBUSY;
    }
}

EXPORT_SYMBOL(pts_stop);

static int __init pts_init(void)
{
    pts_task = kthread_run(pts_thread, NULL, "ptsserv thread");

    if (IS_ERR(pts_task))
        return PTR_ERR(pts_task);

    return 0;
}

static void __exit pts_exit(void)
{
    kthread_stop(pts_task);
}

module_init(pts_init);
module_exit(pts_exit);

MODULE_DESCRIPTION("AMLOGIC PTS service driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Tim Yao <timyao@amlogic.com>");
