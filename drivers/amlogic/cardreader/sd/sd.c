/*****************************************************************
**                                                              **
**  Copyright (C) 2004 Amlogic,Inc.                             **
**  All rights reserved                                         **
**        Filename : sd.c /Project:AVOS  driver                 **
**        Revision : 1.0                                        **
**                                                              **
*****************************************************************/
#include <linux/slab.h>

#include <asm/drivers/cardreader/cardreader.h>
#include <asm/drivers/cardreader/card_io.h>
#include <asm/drivers/cardreader/sd.h>

#include "sd_protocol.h"

static SD_MMC_Card_Info_t sd_info;
extern SD_MMC_Card_Info_t *sd_mmc_info;
extern unsigned char sdio_card_flag;
extern unsigned char sdio_function_no;
extern unsigned disable_high_speed;
extern unsigned disable_wide_bus;

unsigned char sd_insert_detector(void)
{
	int ret = sd_mmc_check_insert();
	if(ret)
        return CARD_INSERTED;
    else
        return CARD_REMOVED;
}

unsigned char sd_open(void)
{
	int ret;
	
	ret = sd_mmc_init(&sd_info);
	if(ret) {
		disable_high_speed = 1;
		ret = sd_mmc_init(&sd_info);
	}
	if(ret) {
		disable_high_speed = 0;
		disable_wide_bus = 1;
		ret = sd_mmc_init(&sd_info);
	}
	if(ret)
		return CARD_UNIT_READY;
	else {
		if(sd_info.card_type == CARD_TYPE_SDIO) {
			sdio_card_flag = 1;
			sdio_function_no = sd_info.sdio_function_no;
		}
		else {
			sdio_card_flag = 0;
		}
		return CARD_UNIT_PROCESSED;
	}
}

unsigned char sd_close(void)
{
	disable_high_speed = 0;
	disable_wide_bus = 0;
	sd_mmc_exit();
	return CARD_UNIT_PROCESSED;
}

unsigned char sd_read_info(unsigned *blk_length, unsigned *capacity, u32 *raw_cid)
{
	if(sd_info.inited_flag) {
		if(blk_length)
			*blk_length = 512;
		if(capacity)
			*capacity = sd_info.blk_nums;
		if(raw_cid)
			memcpy(raw_cid, &(sd_info.raw_cid), sizeof(sd_info.raw_cid));
		return 0;
	}
	else
		return 1;
}

int sd_ioctl(unsigned dev, int req, void *argp)
{
	/*unsigned32 ret=0;
    int errno;
    blkdev_request1 *req1 =(blkdev_request1 *) argp;  
    blkdev_request *r =&(req1->req); 
    blkdev_sg_buffer* psgbuf ;
    void * databuf;
    avfs_status_code status;
    INT32U capacity = 0;

    switch (req)
    {
        case BLKIO_REQUEST:
        {
            psgbuf=&(r->bufs[0]);
            databuf=psgbuf->buffer;
            switch (r->req)
            {
                case BLKDEV_REQ_READ:
                case BLKDEV_REQ_READ_DEV:
                	card_get_dev();
                    ret = sd_mmc_read_data(r->start,r->count*512,(INT8U *)databuf);
                    if(ret)
                    {
                    	disable_high_speed = 1;
                    	sd_mmc_info->inited_flag = 0;
                        ret = sd_mmc_init(&sd_info);
                        if(!ret)
                            ret = sd_mmc_read_data(r->start,r->count*512,(INT8U *)databuf);
                    }
                    card_put_dev();
                    if(!ret)
                        status=AVFS_SUCCESSFUL;
                    else
                        status=AVFS_IO_ERROR;                   
                    if(r->req_done)
                        r->req_done(r->done_arg, status, ret);       
                    break;
                case BLKDEV_REQ_WRITE:
                case BLKDEV_REQ_WRITE_DEV:
                	card_get_dev();
                    ret = sd_mmc_write_data(r->start,r->count*512,(INT8U *)databuf);
                    //if((ret) && (ret!=SD_MMC_ERROR_WRITE_PROTECTED) && (sd_mmc_check_insert()))
                    //{
                    //	sd_mmc_info->inited_flag = 0;
                    //    ret = sd_mmc_init(&sd_info);
                    //    if(!ret)
                    //        ret = sd_mmc_write_data(r->start,r->count*512,(INT8U *)databuf);
                    //}                  
                    card_put_dev();
                    if(!ret)
                        status=AVFS_SUCCESSFUL;
                    else
                        status=AVFS_IO_ERROR;                       
                    if(r->req_done)
                        r->req_done(r->done_arg, status, ret);       
                    break;
                case BLKDEV_REQ_ASYREAD_DEV:
                case BLKDEV_REQ_ASYREAD_MEM:
                    break;
                default:
                    errno = EBADRQC;
                    ret=-1;
                    break;
            }
            break;
        }
        case BLKIO_GETSIZE:
        {
            sd_read_info(NULL, &capacity);
            r->status = capacity;
            break;
        }
        case BLKIO_GET_TYPE_STR:
        {
        	//if(sd_info.card_type == CARD_TYPE_SD)
        		*(char **)argp = "SD";
        	//else
        	//	*(char **)argp = "MMC";
        	break;
    	}
    	case BLKIO_GET_DEVSTAT:
    	{
    		blkdev_stat_t *info = (blkdev_stat_t *)argp;
    		sd_mmc_get_info(info);
    		break;
    	}
        default:
        {
            errno = EBADRQC;
            ret=-1;
            break;
        }
    }*/
    return 0; 
}

void sd_init(void)
{
	sd_io_init();
	sd_mmc_prepare_init();
}

