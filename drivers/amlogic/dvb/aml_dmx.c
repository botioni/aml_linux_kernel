/*
 * AMLOGIC Smart card driver.
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
 */

#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/wait.h>
#include <linux/string.h>
#include <linux/interrupt.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/spinlock.h>
#include <linux/fcntl.h>
#include <asm/irq.h>
#include <asm/uaccess.h>
#include <linux/poll.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#ifdef ARC_700
#include <asm/arch/am_regs.h>
#else
#include <mach/am_regs.h>
#endif
#include "aml_dvb.h"

#define ENABLE_SEC_BUFF_WATCHDOG
#define USE_AHB_MODE

#if 1
//#define pr_dbg(fmt, args...) printk(KERN_DEBUG "DVB: " fmt, ## args)
#define pr_dbg(fmt, args...) printk( "DVB: " fmt, ## args)
#else
#define pr_dbg(fmt, args...)
#endif

#define pr_error(fmt, args...) printk( "DVB: " fmt, ## args)

#define DMX_READ_REG(i,r)\
	((i)?((i==1)?READ_MPEG_REG(r##_2):READ_MPEG_REG(r##_3)):READ_MPEG_REG(r))

#define DMX_WRITE_REG(i,r,d)\
	do{\
	if(i==1) {\
		WRITE_MPEG_REG(r##_2,d);\
	} else if(i==2) {\
		WRITE_MPEG_REG(r##_3,d);\
	}\
	else {\
		WRITE_MPEG_REG(r,d);\
	}\
	}while(0)





#define NO_SUB

#define SYS_CHAN_COUNT    (4)
#define SEC_GRP_LEN_0     (0xc)
#define SEC_GRP_LEN_1     (0xc)
#define SEC_GRP_LEN_2     (0xc)
#define SEC_GRP_LEN_3     (0xc)
#define LARGE_SEC_BUFF_MASK  0xFFFFFFFF
#define LARGE_SEC_BUFF_COUNT 32
#define WATCHDOG_TIMER    250

/*Reset the demux device*/
void dmx_reset_hw(struct aml_dvb *dvb);

/*Section buffer watchdog*/
static void section_buffer_watchdog_func(unsigned long arg)
{
	struct aml_dvb *dvb = (struct aml_dvb*)arg;
	struct aml_dmx *dmx;
	u32 section_busy32 = 0, om_cmd_status32 = 0;
	u16 demux_int_status1 = 0;
	u32 device_no = 0;
	u32 filter_number = 0;
	u32 i = 0;
	unsigned long flags;

	spin_lock_irqsave(&dvb->slock, flags);
	
	for(device_no=0; device_no<DMX_DEV_COUNT; device_no++) {
		dmx = &dvb->dmx[device_no];
		
		if(dmx->init) {
			om_cmd_status32 = DMX_READ_REG(device_no, OM_CMD_STATUS);
#if 1
			if(om_cmd_status32 & 0x8e00) { // bit 15:12 -- om_cmd_count
                                           // bit  11:9 -- overflow_count
				/*BUG: If the recoder is running, return*/
				
				/*Reset the demux*/
				pr_error("reset the demux %x \n",om_cmd_status32);
				dmx_reset_hw(dvb);
				goto end;
			}
#else
			/*
			// bit 15:12 -- om_cmd_count (read only)
			// bit  11:9 -- overflow_count // bit  11:9 -- om_cmd_wr_ptr (read only)
			// bit   8:6 -- om_overwrite_count // bit   8:6 -- om_cmd_rd_ptr (read only)
			// bit   5:3 -- type_stb_om_w_rd (read only)
			// bit     2 -- unit_start_stb_om_w_rd (read only)
			// bit     1 -- om_cmd_overflow (read only)
			// bit     0 -- om_cmd_pending (read)
			// bit     0 -- om_cmd_read_finished (write)
			*/
			if(om_cmd_status32 & 0x0002) {
				pr_error("reset the demux\n");
				dmx_reset_hw(dvb);
				goto end;
			}
#endif
			section_busy32 = DMX_READ_REG(device_no, SEC_BUFF_BUSY);
			if((section_busy32 & LARGE_SEC_BUFF_MASK) == LARGE_SEC_BUFF_MASK)  {
				/*All the largest section buffers occupied, clear buffers*/
				DMX_WRITE_REG(device_no, SEC_BUFF_READY, section_busy32);
			} else {
				for(i = 0; i < 32; i++) {
					if(section_busy32 & (1 << i)) {
						DMX_WRITE_REG(device_no, SEC_BUFF_NUMBER, i);
						filter_number = (DMX_READ_REG(device_no, SEC_BUFF_NUMBER) >> 8);
						if(dmx->filter[i].used) {
							section_busy32 &= ~(1 << i);
						}
					}
				}
				if(section_busy32) {
					/*Clear invalid buffers*/
					DMX_WRITE_REG(device_no, SEC_BUFF_READY, section_busy32);
					pr_error("clear invalid buffers 0x%x\n", section_busy32);
				}
#if 0
				section_busy32 = 0x7fffffff;
				for(i = 0; i < SEC_BUF_BUSY_SIZE; i++) {
					dmx->section_busy[i] = ((i == SEC_BUF_BUSY_SIZE-1) ? 
							DMX_READ_REG(device_no, SEC_BUFF_BUSY) : dmx->section_busy[i+1]);
					section_busy32 &= dmx->section_busy[i];
				}

				/*count the number of '1' bits*/
				i = section_busy32;
				i = (i & 0x55555555) + ((i & 0xaaaaaaaa) >> 1);
				i = (i & 0x33333333) + ((i & 0xcccccccc) >> 2);
				i = (i & 0x0f0f0f0f) + ((i & 0xf0f0f0f0) >> 4);
				i = (i & 0x00ff00ff) + ((i & 0xff00ff00) >> 8);
				i = (i & 0x0000ffff) + ((i & 0xffff0000) >> 16);
				if(i > LARGE_SEC_BUFF_COUNT) {
					/*too long some of the section buffers are being processed*/
					DMX_WRITE_REG(device_no, SEC_BUFF_READY, section_busy32);
				}
#endif
			}
			demux_int_status1 = DMX_READ_REG(device_no, STB_INT_STATUS) & 0xfff7;
			if(demux_int_status1 & (1 << TS_ERROR_PIN)) {
				DMX_WRITE_REG(device_no, STB_INT_STATUS, (1 << TS_ERROR_PIN));
			}
		}
	}

end:
	spin_unlock_irqrestore(&dvb->slock, flags);
#ifdef ENABLE_SEC_BUFF_WATCHDOG
	mod_timer(&dvb->watchdog_timer, jiffies+msecs_to_jiffies(WATCHDOG_TIMER));
#endif	
	return;
}

static inline int sec_filter_match(struct aml_dmx *dmx, struct aml_filter *f, u8 *p)
{
	int b;
	u8 neq = 0;
	
	if(!f->used || !dmx->channel[f->chan_id].used)
		return 0;
	
	for(b=0; b<FILTER_LEN; b++) {
		u8 xor = p[b]^f->value[b];
		
		if(xor&f->maskandmode[b])
			return 0;
		
		if(xor&f->maskandnotmode[b])
			neq = 1;
	}
	
	if(f->neq && !neq)
		return 0;
	
	return 1;
}

static void sec_data_notify(struct aml_dmx *dmx, u16 sec_num, u16 buf_num)
{
	u8 *p = (u8*)dmx->sec_buf[buf_num].addr;
	int sec_len;
	struct aml_filter *f;
	struct dvb_demux_feed *feed;
	
	if(sec_num>=FILTER_COUNT)
		return;
	f = &dmx->filter[sec_num];
#if 1	
	if(!sec_filter_match(dmx, f, p)) {
		int i, chid = f->chan_id;
		
		for(i=0; i<FILTER_COUNT; i++) {
			if(i==sec_num)
				continue;
			f = &dmx->filter[i];
			if(f->chan_id!=chid)
				continue;
			if(sec_filter_match(dmx, f, p)) {
				sec_num = i;
				break;
			}
		}
		
		if(i>=FILTER_COUNT)
			return;
	}
#endif	
	sec_len = (((p[1]&0xF)<<8)|p[2])+3;
	feed = f->feed;
	if(feed->feed.sec.check_crc) {
		struct dvb_demux *demux = feed->demux;
		struct dmx_section_feed *sec = &feed->feed.sec;
		int section_syntax_indicator;
		
		section_syntax_indicator = ((p[1] & 0x80) != 0);
		sec->seclen  = sec_len;
                sec->crc_val = ~0;
		if (section_syntax_indicator &&
				demux->check_crc32(feed, p, sec_len)) {
#if 0
			int i;
			
			for(i=0; i<sec_len; i++)
			{
				printk("%02x ", p[i]);
				if(!((i+1)%16))
					printk("\n");
			}
			printk("\nsection data\n");
#endif
			pr_error("section CRC check failed!\n");
			return;
		}
	}
	if(f->feed && f->feed->cb.sec) {
		f->feed->cb.sec(p, sec_len, NULL, 0, f->filter, DMX_OK);
	}
}

static void process_section(struct aml_dmx *dmx)
{
	u32 ready, i;
	u16 sec_num;
	
	//pr_dbg("section\n");
	ready = DMX_READ_REG(dmx->id, SEC_BUFF_READY);
	if(ready) {
#ifdef USE_AHB_MODE
	//	WRITE_ISA_REG(AHB_BRIDGE_CTRL1, READ_ISA_REG (AHB_BRIDGE_CTRL1) | (1 << 31));
	//	WRITE_ISA_REG(AHB_BRIDGE_CTRL1, READ_ISA_REG (AHB_BRIDGE_CTRL1) & (~ (1 << 31)));
#endif
		for(i=0; i<32; i++) {
			if(!(ready & (1 << i)))
				continue;
		
			DMX_WRITE_REG(dmx->id, SEC_BUFF_NUMBER, i);
			sec_num = (DMX_READ_REG(dmx->id, SEC_BUFF_NUMBER) >> 8);
			
			sec_data_notify(dmx, sec_num, i);
			
			DMX_WRITE_REG(dmx->id, SEC_BUFF_READY, (1 << i));
		}
	}
}

#ifdef NO_SUB
static void process_sub(struct aml_dmx *dmx)
{
}
#endif

static void process_pes(struct aml_dmx *dmx)
{
}

static void process_om_read(struct aml_dmx *dmx)
{
	unsigned i;
	unsigned short om_cmd_status_data_0 = 0;
	unsigned short om_cmd_status_data_1 = 0;
//	unsigned short om_cmd_status_data_2 = 0;
	unsigned short om_cmd_data_out = 0;
	
	om_cmd_status_data_0 = DMX_READ_REG(dmx->id, OM_CMD_STATUS);
	om_cmd_status_data_1 = DMX_READ_REG(dmx->id, OM_CMD_DATA);
//	om_cmd_status_data_2 = DMX_READ_REG(dmx->id, OM_CMD_DATA2);
	
	if(om_cmd_status_data_0 & 1) {
		DMX_WRITE_REG(dmx->id, OM_DATA_RD_ADDR, (1<<15) | ((om_cmd_status_data_1 & 0xff) << 2));
		for(i = 0; i<(((om_cmd_status_data_1 >> 7) & 0x1fc) >> 1); i++) {
			om_cmd_data_out = DMX_READ_REG(dmx->id, OM_DATA_RD); 
		}
	
		om_cmd_data_out = DMX_READ_REG(dmx->id, OM_DATA_RD_ADDR); 
		DMX_WRITE_REG(dmx->id, OM_DATA_RD_ADDR, 0);
		DMX_WRITE_REG(dmx->id, OM_CMD_STATUS, 1);
	}
}

static void dmx_irq_bh_handler(unsigned long arg)
{
	struct aml_dmx *dmx = (struct aml_dmx*)arg;
#if 0
	u32 status;

	status = DMX_READ_REG(dmx->id, STB_INT_STATUS);
	
	if(status) {
		DMX_WRITE_REG(dmx->id, STB_INT_STATUS, status);
	}
#endif
	return;
}

static irqreturn_t dmx_irq_handler(int irq_number, void *para)
{
	struct aml_dmx *dmx = (struct aml_dmx*)para;
	u32 status;

	status = DMX_READ_REG(dmx->id, STB_INT_STATUS);
	if(!status) return IRQ_HANDLED;
	//printk("demux int\n");
	if(status & (1<<SECTION_BUFFER_READY)) {
		process_section(dmx);
	}
#ifdef NO_SUB
        if(status & (1<<SUB_PES_READY)) {
		process_sub(dmx);
	}
#endif
	if(status & (1<<OTHER_PES_READY)) {
		process_pes(dmx);
	}
	if(status & (1<<OM_CMD_READ_PENDING)) {
		process_om_read(dmx);
	}
	if(status & (1<<DUPLICATED_PACKET)) {
	}
	if(status & (1<<DIS_CONTINUITY_PACKET)) {
	}
	if(status & (1<<VIDEO_SPLICING_POINT)) {
	}
	if(status & (1<<AUDIO_SPLICING_POINT)) {
	}
	if(status & (1<<TS_ERROR_PIN)) {
	}
	
	if(dmx->irq_handler) {
		dmx->irq_handler(dmx->dmx_irq, (void*)dmx->id);
	}
	
	DMX_WRITE_REG(dmx->id, STB_INT_STATUS, status);
	
	tasklet_schedule(&dmx->dmx_tasklet);
	return IRQ_HANDLED;
}

static void dvr_irq_bh_handler(unsigned long arg)
{
	struct aml_dmx *dmx = (struct aml_dmx*)arg;
	return;
}

static irqreturn_t dvr_irq_handler(int irq_number, void *para)
{
	struct aml_dmx *dmx = (struct aml_dmx*)para;
	
	tasklet_schedule(&dmx->dvr_tasklet);
	return  IRQ_HANDLED;
}

/*Enable the STB*/
static void stb_enable(struct aml_dvb *dvb)
{
	int out_src, des_in, en_des, invert_clk, fec_s, fec_clk, hiu;
	
	switch(dvb->stb_source) {
		case AM_TS_SRC_TS0:
			out_src = 0;
			des_in  = 0;
			en_des  = 1;
			invert_clk = 1;
			fec_s   = 0;
			fec_clk = 4;
			hiu     = 0;
		break;
		case AM_TS_SRC_TS1:
			out_src = 1;
			des_in  = 1;
			en_des  = 1;
			invert_clk = 1;
			fec_s   = 1;
			fec_clk = 4;
			hiu     = 0;
		break;
		case AM_TS_SRC_TS2:
			out_src = 2;
			des_in  = 2;
			en_des  = 1;
			invert_clk = 1;
			fec_s   = 2;
			fec_clk = 4;
			hiu     = 0;
		break;		
		case AM_TS_SRC_S2P0:
			out_src = 6;
			des_in  = 0;
			en_des  = 1;
			invert_clk = 1;
			fec_s   = 0;
			fec_clk = 4;
			hiu     = 0;
		break;
		case AM_TS_SRC_S2P1:
			out_src = 6;
			des_in  = 1;
			en_des  = 1;
			invert_clk = 1;
			fec_s   = 1;
			fec_clk = 4;
			hiu     = 0;
		break;
		case AM_TS_SRC_HIU:
			out_src = 0;
			des_in  = 0;
			en_des  = 0;
			invert_clk = 0;
			fec_s   = 0;
			fec_clk = 4;
			hiu     = 1;
		break;
		default:
			out_src = 0;
			des_in  = 0;
			en_des  = 0;
			invert_clk = 0;
			fec_s   = 0;
			fec_clk = 0;
			hiu     = 0;
		break;
	}
	
	WRITE_MPEG_REG(STB_TOP_CONFIG, 
		(out_src<<TS_OUTPUT_SOURCE) |
		(des_in<<DES_INPUT_SEL)     |
		(en_des<<ENABLE_DES_PL)     |
		(0<<INVERT_S2P0_FEC_ERROR)    |
		(0<<INVERT_S2P0_FEC_DATA)     |
		(0<<INVERT_S2P0_FEC_SYNC)     |
		(0<<INVERT_S2P0_FEC_VALID)    |
		(invert_clk<<INVERT_S2P0_FEC_CLK)|
		(fec_s<<S2P0_FEC_SERIAL_SEL));

	WRITE_MPEG_REG(TS_FILE_CONFIG,
		(6<<DES_OUT_DLY)                      |
		(3<<TRANSPORT_SCRAMBLING_CONTROL_ODD) |
		(hiu<<TS_HIU_ENABLE)                  |
		(fec_clk<<FEC_FILE_CLK_DIV));
}

int dsc_set_pid(struct aml_dsc *dsc, int pid)
{
	u32 data;
	
	WRITE_MPEG_REG(TS_PL_PID_INDEX, (dsc->id & 0x0f)>>1);
	data = READ_MPEG_REG(TS_PL_PID_DATA);
	if(dsc->id&1) {
		data &= 0xFFFF0000;
		data |= pid;
	} else {
		data &= 0xFFFF;
		data |= (pid<<16);
	}
	WRITE_MPEG_REG(TS_PL_PID_INDEX, (dsc->id & 0x0f)>>1);
	WRITE_MPEG_REG(TS_PL_PID_DATA, data);
	WRITE_MPEG_REG(TS_PL_PID_INDEX, 0);
	pr_dbg("set DSC %d PID %d\n", dsc->id, pid);
	return 0;
}

int dsc_set_key(struct aml_dsc *dsc, int type, u8 *key)
{
	u16 k0, k1, k2, k3;
	u32 key0, key1;
	
	k0 = (key[0]<<8)|key[1];
	k1 = (key[2]<<8)|key[3];
	k2 = (key[4]<<8)|key[5];
	k3 = (key[6]<<8)|key[7];
	
	key0 = (k0<<16)|k1;
	key1 = (k2<<16)|k3;
	WRITE_MPEG_REG(COMM_DESC_KEY0, key0);
	WRITE_MPEG_REG(COMM_DESC_KEY1, key1);
	WRITE_MPEG_REG(COMM_DESC_KEY_RW, (dsc->id + type*DSC_COUNT));
	
	pr_dbg("set DSC %d type %d key %04x %04x %04x %04x\n", dsc->id, type,
			k0, k1, k2, k3);
	return 0;
}

int dsc_release(struct aml_dsc *dsc)
{
	u32 data;
	
	WRITE_MPEG_REG(TS_PL_PID_INDEX, (dsc->id & 0x0f)>>1);
	data = READ_MPEG_REG(TS_PL_PID_DATA);
	if(dsc->id&1) {
		data |= 1<<PID_MATCH_DISABLE_LOW;
	} else {
		data |= 1<<PID_MATCH_DISABLE_HIGH;
	}
	WRITE_MPEG_REG(TS_PL_PID_INDEX, (dsc->id & 0x0f)>>1);
	WRITE_MPEG_REG(TS_PL_PID_DATA, data);
	
	pr_dbg("release DSC %d\n", dsc->id);
	return 0;
}

/*Set section buffer*/
static int dmx_alloc_sec_buffer(struct aml_dmx *dmx)
{
	unsigned long base;
	unsigned long grp_addr[SEC_BUF_GRP_COUNT];
	int grp_len[SEC_BUF_GRP_COUNT];
	int i;
	
	if(dmx->sec_pages)
		return 0;
	
	grp_len[0] = (1<<SEC_GRP_LEN_0)*8;
	grp_len[1] = (1<<SEC_GRP_LEN_1)*8;
	grp_len[2] = (1<<SEC_GRP_LEN_2)*8;
	grp_len[3] = (1<<SEC_GRP_LEN_3)*8;

	dmx->sec_total_len = grp_len[0]+grp_len[1]+grp_len[2]+grp_len[3];
	dmx->sec_pages = __get_free_pages(GFP_KERNEL, get_order(dmx->sec_total_len));
	if(!dmx->sec_pages) {
		pr_error("cannot allocate section buffer %d bytes %d order\n", dmx->sec_total_len, get_order(dmx->sec_total_len));
		return -1;
	}
	
	grp_addr[0] = virt_to_phys((void*)dmx->sec_pages);
	grp_addr[1] = grp_addr[0]+grp_len[0];
	grp_addr[2] = grp_addr[1]+grp_len[1];
	grp_addr[3] = grp_addr[2]+grp_len[2];
	
	dmx->sec_buf[0].addr = dmx->sec_pages;
	dmx->sec_buf[0].len  = grp_len[0]/8;
	
	for(i=1; i<SEC_BUF_COUNT; i++) {
		dmx->sec_buf[i].addr = dmx->sec_buf[i-1].addr+dmx->sec_buf[i-1].len;
		dmx->sec_buf[i].len  = grp_len[i/8]/8;
	}
	
	base = grp_addr[0]&0xFFFF0000;
	DMX_WRITE_REG(dmx->id, SEC_BUFF_BASE, base>>16);
	DMX_WRITE_REG(dmx->id, SEC_BUFF_01_START, (((grp_addr[0]-base)>>8)<<16)|((grp_addr[1]-base)>>8));
	DMX_WRITE_REG(dmx->id, SEC_BUFF_23_START, (((grp_addr[2]-base)>>8)<<16)|((grp_addr[3]-base)>>8));
	DMX_WRITE_REG(dmx->id, SEC_BUFF_SIZE, SEC_GRP_LEN_0|
		(SEC_GRP_LEN_1<<4)|
		(SEC_GRP_LEN_2<<8)|
		(SEC_GRP_LEN_3<<12));
	
	return 0;
}

#ifdef NO_SUB
/*Set subtitle buffer*/
static int dmx_alloc_sub_buffer(struct aml_dmx *dmx)
{
	unsigned long addr;
	
	if(dmx->sub_pages)
		return 0;
	
	dmx->sub_buf_len = 64*1024;
	dmx->sub_pages = __get_free_pages(GFP_KERNEL, get_order(dmx->sub_buf_len));
	if(!dmx->sub_pages) {
		pr_error("cannot allocate subtitle buffer\n");
		return -1;
	}
	
	addr = virt_to_phys((void*)dmx->sub_pages);
	DMX_WRITE_REG(dmx->id, SB_START, addr>>12);
	DMX_WRITE_REG(dmx->id, SB_LAST_ADDR, (dmx->sub_buf_len>>3));
	return 0;	
}
#endif /*NO_SUB*/

/*Set PES buffer*/
static int dmx_alloc_pes_buffer(struct aml_dmx *dmx)
{
	unsigned long addr;
	
	if(dmx->pes_pages)
		return 0;
	
	dmx->pes_buf_len = 64*1024;
	dmx->pes_pages = __get_free_pages(GFP_KERNEL, get_order(dmx->pes_buf_len));
	if(!dmx->pes_pages) {
		pr_error("cannot allocate pes buffer\n");
		return -1;
	}
	
	addr = virt_to_phys((void*)dmx->pes_pages);
	DMX_WRITE_REG(dmx->id, OB_START, addr>>12);
	DMX_WRITE_REG(dmx->id, OB_LAST_ADDR, (dmx->pes_buf_len>>3));
	return 0;	
}

static void dmx_set_mux(struct aml_dvb *dvb)
{
#define PREG_PIN_MUX_REG3 PERIPHS_PIN_MUX_3
#define PINMUX3_GPIOC0_FECA_DVALID  1<<22
#define PINMUX3_GPIOC1_FECA_FAIL       1<<21
#define PINMUX3_GPIOC2_FECA_SOP         1<<20
#define PINMUX3_GPIOC3_FECA_CLK         1<<19
#define PINMUX3_GPIOC4_FECA_D0           1<<18
#define PINMUX3_GPIOC5_11_FECA_D1_7 1<<17

#define PREG_PIN_MUX_REG5 PERIPHS_PIN_MUX_5
#define PINMUX5_GPIOD12_FECB_DVALID  1<<22
#define PINMUX5_GPIOD13_FECB_FAIL       1<<21
#define PINMUX5_GPIOD14_FECB_SOP         1<<20
#define PINMUX5_GPIOD15_FECB_CLK         1<<19
#define PINMUX5_GPIOD16_FECB_D0           1<<18
#define PINMUX5_GPIOD17_24_FECB_D1_7 1<<17

#ifdef CONFIG_MACH_MESON_8726M_DVBC
	SET_CBUS_REG_MASK(PREG_PIN_MUX_REG3,PINMUX3_GPIOC4_FECA_D0);
	SET_CBUS_REG_MASK(PREG_PIN_MUX_REG3,PINMUX3_GPIOC5_11_FECA_D1_7);
 	SET_CBUS_REG_MASK(PREG_PIN_MUX_REG3,PINMUX3_GPIOC3_FECA_CLK);
	SET_CBUS_REG_MASK(PREG_PIN_MUX_REG3,PINMUX3_GPIOC2_FECA_SOP);
	SET_CBUS_REG_MASK(PREG_PIN_MUX_REG3,PINMUX3_GPIOC0_FECA_DVALID);
	SET_CBUS_REG_MASK(PREG_PIN_MUX_REG3,PINMUX3_GPIOC1_FECA_FAIL);
#else
	SET_CBUS_REG_MASK(PREG_PIN_MUX_REG5,PINMUX5_GPIOD16_FECB_D0);
	SET_CBUS_REG_MASK(PREG_PIN_MUX_REG5,PINMUX5_GPIOD17_24_FECB_D1_7);
 	SET_CBUS_REG_MASK(PREG_PIN_MUX_REG5,PINMUX5_GPIOD15_FECB_CLK);
	SET_CBUS_REG_MASK(PREG_PIN_MUX_REG5,PINMUX5_GPIOD14_FECB_SOP);
	SET_CBUS_REG_MASK(PREG_PIN_MUX_REG5,PINMUX5_GPIOD12_FECB_DVALID);
	SET_CBUS_REG_MASK(PREG_PIN_MUX_REG5,PINMUX5_GPIOD13_FECB_FAIL);
#endif

}

/*Initalize the registers*/
static int dmx_init(struct aml_dmx *dmx)
{
	struct aml_dvb *dvb = (struct aml_dvb*)dmx->demux.priv;
	int irq;
	
	if(dmx->init) return 0;
	
	pr_dbg("demux init\n");
	
	/*Register irq handlers*/
	if(dmx->dmx_irq!=-1) {
		pr_dbg("request irq\n");
		tasklet_init(&dmx->dmx_tasklet, dmx_irq_bh_handler, (unsigned long)dmx);
		irq = request_irq(dmx->dmx_irq, dmx_irq_handler, IRQF_SHARED, "dmx irq", dmx);
	}
	
	if(dmx->dvr_irq!=-1) {
		tasklet_init(&dmx->dvr_tasklet, dvr_irq_bh_handler, (unsigned long)dmx);
		irq = request_irq(dmx->dvr_irq, dvr_irq_handler, IRQF_SHARED, "dvr irq", dmx);
	}
	
	/*Allocate buffer*/
	if(dmx_alloc_sec_buffer(dmx)<0)
		return -1;
#ifdef NO_SUB
	if(dmx_alloc_sub_buffer(dmx)<0)
		return -1;
#endif
	if(dmx_alloc_pes_buffer(dmx)<0)
		return -1;
	
	/*Reset the hardware*/
	if (!dvb->dmx_init) {
		pr_dbg("demux reset\n");
		dmx_set_mux(dvb);
		
		init_timer(&dvb->watchdog_timer);
		dvb->watchdog_timer.function = section_buffer_watchdog_func;
		dvb->watchdog_timer.expires  = jiffies + msecs_to_jiffies(WATCHDOG_TIMER);
		dvb->watchdog_timer.data     = (unsigned long)dvb;
#ifdef ENABLE_SEC_BUFF_WATCHDOG
		add_timer(&dvb->watchdog_timer);
#endif
		dmx_reset_hw(dvb);
	}
	
	dvb->dmx_init++;
	
	dmx->init = 1;
	
	return 0;
}

/*Release the resource*/
static int dmx_deinit(struct aml_dmx *dmx)
{
	struct aml_dvb *dvb = (struct aml_dvb*)dmx->demux.priv;
	
	if(!dmx->init) return 0;
	
	DMX_WRITE_REG(dmx->id, DEMUX_CONTROL, 0);
	
	dvb->dmx_init--;
	
	/*Reset the hardware*/
	if(!dvb->dmx_init) {
		dmx_reset_hw(dvb);
#ifdef ENABLE_SEC_BUFF_WATCHDOG
		del_timer_sync(&dvb->watchdog_timer);
#endif
	}
	
	if(dmx->sec_pages) {
		free_pages(dmx->sec_pages, get_order(dmx->sec_total_len));
		dmx->sec_pages = 0;
	}
#ifdef NO_SUB
	if(dmx->sub_pages) {
		free_pages(dmx->sub_pages, get_order(dmx->sub_buf_len));
		dmx->sub_pages = 0;
	}
#endif
	if(dmx->pes_pages) {
		free_pages(dmx->pes_pages, get_order(dmx->pes_buf_len));
		dmx->pes_pages = 0;
	}
	
	if(dmx->dmx_irq!=-1) {
		free_irq(dmx->dmx_irq, dmx);
		tasklet_kill(&dmx->dmx_tasklet);
	}
	if(dmx->dvr_irq!=-1) {
		free_irq(dmx->dvr_irq, dmx);
		tasklet_kill(&dmx->dvr_tasklet);
	}
	
	dmx->init = 0;

	return 0;
}

/*Enable the demux device*/
static int dmx_enable(struct aml_dmx *dmx)
{
	struct aml_dvb *dvb = (struct aml_dvb*)dmx->demux.priv;
	int fec_sel, hi_bsf, fec_clk, record;
	int fec_core_sel;
	
	pr_dbg("demux enable\n");
	
	switch(dmx->source) {
		case AM_TS_SRC_TS0:
			fec_sel = 0;
			fec_clk = 1;
			record  = dmx->record?1:0;
			break;
		case AM_TS_SRC_TS1:
			fec_sel = 1;
			fec_clk = 1;
			record  = dmx->record?1:0;
		break;
		case AM_TS_SRC_TS2:
			fec_sel = 2;
			fec_clk = 0;
			record  = dmx->record?1:0;
		break;		
		case AM_TS_SRC_S2P0:
		case AM_TS_SRC_S2P1:
			fec_sel = 6;
			fec_clk = 1;
			record  = dmx->record?1:0;
		break;
		case AM_TS_SRC_HIU:
			fec_sel = 7;
			fec_clk = 0;
			record  = 0;
		break;
		default:
			fec_sel = 0;
			fec_clk = 0;
			record  = 0;
		break;
	}
	
	if(dmx->channel[0].used || dmx->channel[1].used)
		hi_bsf = 1;
	else
		hi_bsf = 0;
	fec_core_sel = (dmx->source==(dvb?dvb->dsc_source:0))?1:0;
	
	if(dmx->chan_count) {
		/*Initialize the registers*/
		DMX_WRITE_REG(dmx->id, STB_INT_MASK,
			(0<<(AUDIO_SPLICING_POINT))     |
			(0<<(VIDEO_SPLICING_POINT))     |
			(0<<(OTHER_PES_READY))          |
			(1<<(SUB_PES_READY))            |
			(1<<(SECTION_BUFFER_READY))     |
			(0<<(OM_CMD_READ_PENDING))      |
			(0<<(TS_ERROR_PIN))             |
			(1<<(NEW_PDTS_READY))           | 
			(0<<(DUPLICATED_PACKET))        | 
			(0<<(DIS_CONTINUITY_PACKET)));
		DMX_WRITE_REG(dmx->id, DEMUX_MEM_REQ_EN, 
#ifdef USE_AHB_MODE
			(1<<SECTION_AHB_DMA_EN)         |
#endif
			(1<<SECTION_PACKET)             |
			(1<<VIDEO_PACKET)               |
			(1<<AUDIO_PACKET)               |
			(1<<SUB_PACKET)                 |
			(0<<OTHER_PES_PACKET));
	 	DMX_WRITE_REG(dmx->id, PES_STRONG_SYNC, 0x1234);
 		DMX_WRITE_REG(dmx->id, DEMUX_ENDIAN,
			(7<<OTHER_ENDIAN)               |
			(7<<BYPASS_ENDIAN)              |
			(0<<SECTION_ENDIAN));
		DMX_WRITE_REG(dmx->id, TS_HIU_CTL,
			(0<<LAST_BURST_THRESHOLD)       |
			(hi_bsf<<USE_HI_BSF_INTERFACE));

		
		DMX_WRITE_REG(dmx->id, FEC_INPUT_CONTROL,
			(fec_core_sel<<FEC_CORE_SEL)    |
			(fec_sel<<FEC_SEL)              |
			(fec_clk<<FEC_INPUT_FEC_CLK)    |
			(0<<FEC_INPUT_SOP)              |
			(0<<FEC_INPUT_D_VALID)          |
			(0<<FEC_INPUT_D_FAIL));
		DMX_WRITE_REG(dmx->id, STB_OM_CTL, 
			(0x20<<MAX_OM_DMA_COUNT)        |
			(0x7f<<LAST_OM_ADDR));
		DMX_WRITE_REG(dmx->id, DEMUX_CONTROL,
			(0<<BYPASS_USE_RECODER_PATH)          |
			(0<<INSERT_AUDIO_PES_STRONG_SYNC)     |
			(0<<INSERT_VIDEO_PES_STRONG_SYNC)     |    
			(0<<OTHER_INT_AT_PES_BEGINING)        |
			(0<<DISCARD_AV_PACKAGE)               |
			(0<<TS_RECORDER_SELECT)               |
			(record<<TS_RECORDER_ENABLE)          |
			(0<<KEEP_DUPLICATE_PACKAGE)           |
			(1<<SECTION_END_WITH_TABLE_ID)        |
			(1<<STB_DEMUX_ENABLE));
	} else {
		DMX_WRITE_REG(dmx->id, STB_INT_MASK, 0);
		DMX_WRITE_REG(dmx->id, FEC_INPUT_CONTROL, 0);
		DMX_WRITE_REG(dmx->id, DEMUX_CONTROL, 0);
	}
	
	
	return 0;
}

/*Get the channel's ID by its PID*/
static int dmx_get_chan(struct aml_dmx *dmx, int pid)
{
	int id;
	
	for(id=0; id<CHANNEL_COUNT; id++) {
		if(dmx->channel[id].used && dmx->channel[id].pid==pid)
			return id;
	}
	
	return -1;
}

/*Get the channel's target*/
static u32 dmx_get_chan_target(struct aml_dmx *dmx, int cid)
{
	u32 type;
	
	if(!dmx->channel[cid].used) {
		return 0xFFFF;
	}
	
	if(dmx->channel[cid].type==DMX_TYPE_SEC) {
		type = SECTION_PACKET;
	} else {
		switch(dmx->channel[cid].pes_type) {
			case DMX_TS_PES_AUDIO:
				type = AUDIO_PACKET;
			break;
			case DMX_TS_PES_VIDEO:
				type = VIDEO_PACKET;
			break;
			case DMX_TS_PES_SUBTITLE:
				type = SUB_PACKET;
			break;
			default:
				type = OTHER_PES_PACKET;
			break;
		}
	}
	
	pr_dbg("chan target: %x %x\n", type, dmx->channel[cid].pid);
	return (type<<PID_TYPE)|dmx->channel[cid].pid;
}

/*Get the advance value of the channel*/
static inline u32 dmx_get_chan_advance(struct aml_dmx *dmx, int cid)
{
	return 0;
}

/*Set the channel registers*/
static int dmx_set_chan_regs(struct aml_dmx *dmx, int cid)
{
	u32 data, addr, advance, max;
	
	pr_dbg("set channel (id:%d PID:0x%x) registers\n", cid, dmx->channel[cid].pid);
	
	while(DMX_READ_REG(dmx->id, FM_WR_ADDR) & 0x8000) {
		udelay(100);
	}
	
	if(cid&1) {
		data = (dmx_get_chan_target(dmx, cid-1)<<16) | dmx_get_chan_target(dmx, cid);
		advance = (dmx_get_chan_advance(dmx, cid)<<8) | dmx_get_chan_advance(dmx, cid-1);
	} else {
		data = (dmx_get_chan_target(dmx, cid)<<16) | dmx_get_chan_target(dmx, cid+1);
		advance = (dmx_get_chan_advance(dmx, cid+1)<<8) | dmx_get_chan_advance(dmx, cid);
	}
	addr = cid>>1;
	DMX_WRITE_REG(dmx->id, FM_WR_DATA, data);
	DMX_WRITE_REG(dmx->id, FM_WR_ADDR, (advance<<16)|0x8000|addr);
	
	pr_dbg("write fm %x:%x\n", (advance<<16)|0x8000|addr, data);
	
	for(max=CHANNEL_COUNT-1; max>0; max--) {
		if(dmx->channel[max].used)
			break;
	}
	
	data = DMX_READ_REG(dmx->id, MAX_FM_COMP_ADDR)&0xF0;
	DMX_WRITE_REG(dmx->id, MAX_FM_COMP_ADDR, data|(max>>1));
	
	pr_dbg("write fm comp %x\n", data|(max>>1));
	
	if(DMX_READ_REG(dmx->id, OM_CMD_STATUS)&0x8e00) {
		pr_error("error send cmd %x\n", DMX_READ_REG(dmx->id, OM_CMD_STATUS));
	}
	
	return 0;
}

/*Get the filter target*/
static int dmx_get_filter_target(struct aml_dmx *dmx, int fid, u32 *target, u8 *advance)
{
	struct dmx_section_filter *filter;
	struct aml_filter *f;
	int i, cid, neq_bytes;
	
	fid = fid&0xFFFF;
	f = &dmx->filter[fid];
	
	if(!f->used) {
		target[0] = 0x1fff;
		for(i=1; i<FILTER_LEN; i++) {
			target[i] = 0x9fff;
		}
		return 0;
	}
	
	cid = f->chan_id;
	filter = f->filter;
	
	neq_bytes = 0;
	if(filter->filter_mode[0]!=0xFF) {
		neq_bytes = 2;
	} else {
		for(i=1; i<FILTER_LEN; i++) {
			if(filter->filter_mode[i]!=0xFF)
				neq_bytes++;
		}
	}
	
	f->neq = 0;
	
	for(i=0; i<FILTER_LEN; i++) {
		u8 value = filter->filter_value[i];
		u8 mask  = filter->filter_mask[i];
		u8 mode  = filter->filter_mode[i];
		u8 mb, mb1, nb, v, t, adv = 0;
		
		if(!i) {
			mb = 1;
			mb1= 1;
			v  = 0;
			if(mode==0xFF) {
				t = mask&0xF0;
				if(t) {
					mb1 = 0;
					adv |= t^0xF0;
					v   |= (value&0xF0)|adv;
				}
				t = mask&0x0F;
				if(t) {
					mb  = 0;
					adv |= t^0x0F;
					v   |= (value&0x0F)|adv;
				}
			}
			
			target[i] = (mb<<SECTION_FIRSTBYTE_MASKLOW)         |
				(mb1<<SECTION_FIRSTBYTE_MASKHIGH)           |
				(0<<SECTION_FIRSTBYTE_DISABLE_PID_CHECK)    |
				(cid<<SECTION_FIRSTBYTE_PID_INDEX)          |
				v;
			advance[i] = adv;
		} else {
			mb = 1;
			nb = 0;
			v = 0;
			
			if(mode==0xFF) {
				mb  = 0;
				nb  = 0;
				adv = mask^0xFF;
				v   = value|adv;
			} else {
				if(neq_bytes==1) {
					mb  = 0;
					nb  = 1;
					adv = mask^0xFF;
					v   = value&~adv;
				}
			}
			
			target[i] = (mb<<SECTION_RESTBYTE_MASK)             |
				(nb<<SECTION_RESTBYTE_MASK_EQ)              |
				(0<<SECTION_RESTBYTE_DISABLE_PID_CHECK)     |
				(cid<<SECTION_RESTBYTE_PID_INDEX)           |
				v;
			advance[i] = adv;
		}
		
		f->value[i] = value;
		f->maskandmode[i] = mask&mode;
		f->maskandnotmode[i] = mask&~mode;
		
		if(f->maskandnotmode[i])
			f->neq = 1;
	}
	
	return 0;
}

/*Set the filter registers*/
static int dmx_set_filter_regs(struct aml_dmx *dmx, int fid)
{
	u32 t1[FILTER_LEN], t2[FILTER_LEN];
	u8 advance1[FILTER_LEN], advance2[FILTER_LEN];
	u32 addr, data, max, adv;
	int i;
	
	fid = fid&0xFFFF;
	
	pr_dbg("set filter (id:%d) registers\n", fid);
	
	if(fid&1) {
		dmx_get_filter_target(dmx, fid-1, t1, advance1);
		dmx_get_filter_target(dmx, fid, t2, advance2);
	} else {
		dmx_get_filter_target(dmx, fid, t1, advance1);
		dmx_get_filter_target(dmx, fid+1, t2, advance2);
	}
	
	for(i=0; i<FILTER_LEN; i++) {
		while(DMX_READ_REG(dmx->id, FM_WR_ADDR) & 0x8000) {
			udelay(100);
		}
		
		data = (t1[i]<<16)|t2[i];
		addr = (fid>>1)|((i+1)<<4);
		adv  = (advance1[i]<<8)|advance2[i];
		
		DMX_WRITE_REG(dmx->id, FM_WR_DATA, data);
		DMX_WRITE_REG(dmx->id, FM_WR_ADDR, (adv<<16)|0x8000|addr);
		
		pr_dbg("write fm %x:%x\n", (adv<<16)|0x8000|addr, data);
	}

	for(max=FILTER_COUNT-1; max>0; max--) {
		if(dmx->filter[max].used)
			break;
	}
	
	data = DMX_READ_REG(dmx->id, MAX_FM_COMP_ADDR)&0xF;
	DMX_WRITE_REG(dmx->id, MAX_FM_COMP_ADDR, data|((max>>1)<<4));
	
	pr_dbg("write fm comp %x\n", data|((max>>1)<<4));
	
	if(DMX_READ_REG(dmx->id, OM_CMD_STATUS)&0x8e00) {
		pr_error("error send cmd %x\n",DMX_READ_REG(dmx->id, OM_CMD_STATUS));
	}
	
	return 0;
}

/*Clear the filter's buffer*/
static void dmx_clear_filter_buffer(struct aml_dmx *dmx, int fid)
{
	u32 section_busy32 = DMX_READ_REG(dmx->id, SEC_BUFF_READY);
	u32 filter_number;
	int i;
	
	if(!section_busy32)
		return;
	
	for(i = 0; i < 32; i++) {
		if(section_busy32 & (1 << i)) {
			DMX_WRITE_REG(dmx->id, SEC_BUFF_NUMBER, i);
			filter_number = (DMX_READ_REG(dmx->id, SEC_BUFF_NUMBER) >> 8);
			if(filter_number != fid) {
				section_busy32 &= ~(1 << i);
			}
		}
	}
	
	if(section_busy32) {
		DMX_WRITE_REG(dmx->id, SEC_BUFF_READY, section_busy32);
	}
}

/*Reset the demux device*/
void dmx_reset_hw(struct aml_dvb *dvb)
{
	int id, times;
	
	for (id=0; id<DMX_DEV_COUNT; id++) {
		if(!dvb->dmx[id].init)
			continue;
		if(dvb->dmx[id].dmx_irq!=-1) {
			disable_irq(dvb->dmx[id].dmx_irq);
		}
		if(dvb->dmx[id].dvr_irq!=-1) {
			disable_irq(dvb->dmx[id].dvr_irq);
		}
	}
#ifdef ENABLE_SEC_BUFF_WATCHDOG
	del_timer_sync(&dvb->watchdog_timer);
#endif
	
	WRITE_MPEG_REG(RESET1_REGISTER, RESET_DEMUXSTB);
	
	for (id=0; id<DMX_DEV_COUNT; id++) {
		times = 0;
		while(times++ < 1000000) {
			if (!(DMX_READ_REG(id, OM_CMD_STATUS)&0x01))
				break;
		}
	}
	
	WRITE_MPEG_REG(STB_TOP_CONFIG, 0);
	
	for (id=0; id<DMX_DEV_COUNT; id++) {
		u32 version, data;
		
		if(!dvb->dmx[id].init)
			continue;
		
		if(dvb->dmx[id].dmx_irq!=-1) {
			enable_irq(dvb->dmx[id].dmx_irq);
		}
		if(dvb->dmx[id].dvr_irq!=-1) {
			enable_irq(dvb->dmx[id].dvr_irq);
		}
		DMX_WRITE_REG(id, DEMUX_CONTROL, 0x0000);
		version = DMX_READ_REG(id, STB_VERSION);
		DMX_WRITE_REG(id, STB_TEST_REG, version);
		pr_dbg("STB %d hardware version : %d\n", id, version);
		DMX_WRITE_REG(id, STB_TEST_REG, 0x5550);
		data = DMX_READ_REG(id, STB_TEST_REG);
		if(data!=0x5550)
			pr_error("STB %d register access failed\n", id);
		DMX_WRITE_REG(id, STB_TEST_REG, 0xaaa0);
		data = DMX_READ_REG(id, STB_TEST_REG);
		if(data!=0xaaa0)
			pr_error("STB %d register access failed\n", id);
		DMX_WRITE_REG(id, MAX_FM_COMP_ADDR, 0x0000);
		DMX_WRITE_REG(id, STB_INT_MASK, 0);
		DMX_WRITE_REG(id, STB_INT_STATUS, 0xffff);
		DMX_WRITE_REG(id, FEC_INPUT_CONTROL, 0);
	}
	
	stb_enable(dvb);
	
	for(id=0; id<DMX_DEV_COUNT; id++)
	{
		struct aml_dmx *dmx = &dvb->dmx[id];
		int n;
		unsigned long addr;
		unsigned long base;
		unsigned long grp_addr[SEC_BUF_GRP_COUNT];
		int grp_len[SEC_BUF_GRP_COUNT];
		if(!dvb->dmx[id].init)
			continue;

		
		if(dmx->sec_pages) {
			grp_len[0] = (1<<SEC_GRP_LEN_0)*8;
			grp_len[1] = (1<<SEC_GRP_LEN_1)*8;
			grp_len[2] = (1<<SEC_GRP_LEN_2)*8;
			grp_len[3] = (1<<SEC_GRP_LEN_3)*8;

				
			grp_addr[0] = virt_to_phys((void*)dmx->sec_pages);
			grp_addr[1] = grp_addr[0]+grp_len[0];
			grp_addr[2] = grp_addr[1]+grp_len[1];
			grp_addr[3] = grp_addr[2]+grp_len[2];
			
			base = grp_addr[0]&0xFFFF0000;
			DMX_WRITE_REG(dmx->id, SEC_BUFF_BASE, base>>16);
			DMX_WRITE_REG(dmx->id, SEC_BUFF_01_START, (((grp_addr[0]-base)>>8)<<16)|((grp_addr[1]-base)>>8));
			DMX_WRITE_REG(dmx->id, SEC_BUFF_23_START, (((grp_addr[2]-base)>>8)<<16)|((grp_addr[3]-base)>>8));
			DMX_WRITE_REG(dmx->id, SEC_BUFF_SIZE, SEC_GRP_LEN_0|
				(SEC_GRP_LEN_1<<4)|
				(SEC_GRP_LEN_2<<8)|
				(SEC_GRP_LEN_3<<12));
		}
		
		if(dmx->sub_pages) {	
			addr = virt_to_phys((void*)dmx->sub_pages);
			DMX_WRITE_REG(dmx->id, SB_START, addr>>12);
			DMX_WRITE_REG(dmx->id, SB_LAST_ADDR, (dmx->sub_buf_len>>3));
		}
		
		if(dmx->pes_pages) {
			addr = virt_to_phys((void*)dmx->pes_pages);
			DMX_WRITE_REG(dmx->id, OB_START, addr>>12);
			DMX_WRITE_REG(dmx->id, OB_LAST_ADDR, (dmx->pes_buf_len>>3));
		}
		
		for(n=0; n<CHANNEL_COUNT; n++)
		{
			struct aml_channel *chan = &dmx->channel[n];
			
			if(chan->used)
			{
				dmx_set_chan_regs(dmx, n);
			}
		}
		
		for(n=0; n<FILTER_COUNT; n++)
		{
			struct aml_filter *filter = &dmx->filter[n];
			
			if(filter->used)
			{
				dmx_set_filter_regs(dmx, n);
			}
		}
				
		dmx_enable(&dvb->dmx[id]);
	}
	
	for(id=0; id<DSC_COUNT; id++)
	{
		struct aml_dsc *dsc = &dvb->dsc[id];
		
		if(dsc->used)
		{
			dsc_set_pid(dsc, dsc->pid);
			
			if(dsc->set&1)
				dsc_set_key(dsc, 0, dsc->even);
			if(dsc->set&2)
				dsc_set_key(dsc, 1, dsc->odd);
		}
	}
#ifdef ENABLE_SEC_BUFF_WATCHDOG
	mod_timer(&dvb->watchdog_timer, jiffies+msecs_to_jiffies(WATCHDOG_TIMER));
#endif
}

/*Allocate a new channel*/
int dmx_alloc_chan(struct aml_dmx *dmx, int type, int pes_type, int pid)
{
	int id = -1;
	
	if(type==DMX_TYPE_TS) {
		switch(pes_type) {
			case DMX_TS_PES_VIDEO:
				if(!dmx->channel[0].used)
					id = 0;
			break;
			case DMX_TS_PES_AUDIO:
				if(!dmx->channel[1].used)
					id = 1;
			break;
			case DMX_TS_PES_SUBTITLE:
			case DMX_TS_PES_TELETEXT:
				if(!dmx->channel[2].used)
					id = 2;
			break;
			case DMX_TS_PES_PCR:
				if(!dmx->channel[3].used)
					id = 3;
			break;
			default:
			break;
		}
	} else {
		int i;
		for(i=SYS_CHAN_COUNT; i<CHANNEL_COUNT; i++) {
			if(!dmx->channel[i].used) {
				id = i;
				break;
			}
		}
	}
	
	if(id==-1) {
		pr_error("too many channels\n");
		return -1;
	}
	
	pr_dbg("allocate channel(id:%d PID:0x%x)\n", id, pid);
	
	dmx->channel[id].type = type;
	dmx->channel[id].pes_type = pes_type;
	dmx->channel[id].pid  = pid;
	dmx->channel[id].used = 1;
	dmx->channel[id].filter_count = 0;
	
	dmx_set_chan_regs(dmx, id);
	
	dmx->chan_count++;
	
	dmx_enable(dmx);
	
	return id;
}

/*Free a channel*/
void dmx_free_chan(struct aml_dmx *dmx, int cid)
{
	pr_dbg("free channel(id:%d PID:0x%x)\n", cid, dmx->channel[cid].pid);
	
	dmx->channel[cid].used = 0;
	dmx_set_chan_regs(dmx, cid);
	
	dmx->chan_count--;
	
	dmx_enable(dmx);
}

/*Add a section*/
static int dmx_chan_add_filter(struct aml_dmx *dmx, int cid, struct dvb_demux_feed *feed)
{
	int id = -1;
	int i;
	
	for(i=0; i<FILTER_COUNT; i++) {
		if(!dmx->filter[i].used) {
			id = i;
			break;
		}
	}
	
	if(id==-1) {
		pr_error("too many filters\n");
		return -1;
	}
	
	pr_dbg("channel(id:%d PID:0x%x) add filter(id:%d)\n", cid, feed->pid, id);
	
	dmx->filter[id].chan_id = cid;
	dmx->filter[id].used = 1;
	dmx->filter[id].filter = &feed->filter->filter;
	dmx->filter[id].feed = feed;
	dmx->channel[cid].filter_count++;
	
	id = (cid<<16)|id;
	feed->priv = (void*)id;
	
	dmx_set_filter_regs(dmx, id);
	
	return id;
}

static void dmx_remove_filter(struct aml_dmx *dmx, struct dvb_demux_feed *feed)
{
	int id = (int)feed->priv;
	int cid = id>>16;
	int fid = id&0xFFFF;
	
	pr_dbg("channel(id:%d PID:0x%x) remove filter(id:%d)\n", cid, feed->pid, fid);
	
	dmx->filter[fid].used = 0;
	dmx->channel[cid].filter_count--;
	
	dmx_set_filter_regs(dmx, id);
	dmx_clear_filter_buffer(dmx, id);
	
	if(dmx->channel[cid].filter_count<=0) {
		dmx_free_chan(dmx, cid);
	}
}

static int dmx_add_feed(struct aml_dmx *dmx, struct dvb_demux_feed *feed)
{
	int id, ret;
	
	switch(feed->type)
	{
		case DMX_TYPE_TS:
			if((ret=dmx_get_chan(dmx, feed->pid))>=0) {
				pr_error("PID %d already used\n", feed->pid);
				return -EBUSY;
			}
			if((ret=dmx_alloc_chan(dmx, feed->type, feed->pes_type, feed->pid))<0) {
				return ret;
			}
			feed->priv = (void*)ret;
		break;
		case DMX_TYPE_SEC:
			if((id=dmx_get_chan(dmx, feed->pid))<0) {
				if((id=dmx_alloc_chan(dmx, feed->type, feed->pes_type, feed->pid))<0) {
					return id;
				}
			} else {
				if(id<SYS_CHAN_COUNT) {
					pr_error("pid 0x%x is not a section\n", feed->pid);
					return -EINVAL;
				}
			}
			if((ret=dmx_chan_add_filter(dmx, id, feed))<0) {
				if(!dmx->channel[id].filter_count) {
					dmx_free_chan(dmx, id);
				}
			}
		break;
		default:
			return -EINVAL;
		break;
	}
	
	dmx->feed_count++;
	
	return 0;
}

static int dmx_remove_feed(struct aml_dmx *dmx, struct dvb_demux_feed *feed)
{
	switch(feed->type)
	{
		case DMX_TYPE_TS:
			dmx_free_chan(dmx, (int)feed->priv);
		break;
		case DMX_TYPE_SEC:
			dmx_remove_filter(dmx, feed);
		break;
		default:
			return -EINVAL;
		break;
	}
	
	dmx->feed_count--;
	return 0;
}

int aml_dmx_hw_init(struct aml_dmx *dmx)
{
	struct aml_dvb *dvb = (struct aml_dvb*)dmx->demux.priv;
	unsigned long flags;
	int ret;
	
	/*Demux initialize*/
	spin_lock_irqsave(&dmx->slock, flags);
	ret = dmx_init(dmx);
	spin_unlock_irqrestore(&dmx->slock, flags);
	
	return ret;
}

int aml_dmx_hw_deinit(struct aml_dmx *dmx)
{
	unsigned long flags;
	int ret;
	spin_lock_irqsave(&dmx->slock, flags);
	ret = dmx_deinit(dmx);
	spin_unlock_irqrestore(&dmx->slock, flags);
	
	return ret;
}

int aml_dmx_hw_start_feed(struct dvb_demux_feed *dvbdmxfeed)
{
	struct aml_dmx *dmx = (struct aml_dmx*)dvbdmxfeed->demux;
	struct aml_dvb *dvb = (struct aml_dvb*)dmx->demux.priv;
	unsigned long flags;
	int ret = 0;
	
	spin_lock_irqsave(&dmx->slock, flags);
	ret = dmx_add_feed(dmx, dvbdmxfeed);
	spin_unlock_irqrestore(&dmx->slock, flags);
	
	return ret;
}

int aml_dmx_hw_stop_feed(struct dvb_demux_feed *dvbdmxfeed)
{
	struct aml_dmx *dmx = (struct aml_dmx*)dvbdmxfeed->demux;
	struct aml_dvb *dvb = (struct aml_dvb*)dmx->demux.priv;
	unsigned long flags;
	
	spin_lock_irqsave(&dmx->slock, flags);
	dmx_remove_feed(dmx, dvbdmxfeed);
	spin_unlock_irqrestore(&dmx->slock, flags);
	
	return 0;
}

int aml_dmx_hw_set_source(struct dmx_demux* demux, dmx_source_t src)
{
	struct aml_dmx *dmx = (struct aml_dmx*)demux;
	struct aml_dvb *dvb = (struct aml_dvb*)dmx->demux.priv;
	int ret = 0;
	unsigned long flags;
	
	spin_lock_irqsave(&dmx->slock, flags);
	switch(src) {
		case DMX_SOURCE_FRONT0:
#ifndef CONFIG_AMLOGIC_S2P_TS0
			if(dmx->source!=AM_TS_SRC_TS0) {
				dmx->source = AM_TS_SRC_TS0;
				ret = 1;
			}
#else
			if(dmx->source!=AM_TS_SRC_S2P0) {
				dmx->source = AM_TS_SRC_S2P0;
				ret = 1;
			}
#endif
		break;
		case DMX_SOURCE_FRONT1:
#ifndef CONFIG_AMLOGIC_S2P_TS1
			if(dmx->source!=AM_TS_SRC_TS1) {
				dmx->source = AM_TS_SRC_TS1;
				ret = 1;
			}
#else
			if(dmx->source!=AM_TS_SRC_S2P1) {
				dmx->source = AM_TS_SRC_S2P1;
				ret = 1;
			}
#endif
		break;
		case DMX_SOURCE_FRONT2:
			if(dmx->source!=AM_TS_SRC_TS2) {
				dmx->source = AM_TS_SRC_TS2;
				ret = 1;
			}
		break;
		case DMX_SOURCE_DVR0:
			if(dmx->source!=AM_TS_SRC_HIU) {
				dmx->source = AM_TS_SRC_HIU;
				ret = 1;
			}
		break;
		default:
			pr_error("illegal demux source %d\n", src);
			ret = -EINVAL;
		break;
	}
	
	if(ret>0) {
		dmx_reset_hw(dvb);
	}
	
	spin_unlock_irqrestore(&dmx->slock, flags);
	
	return ret;
}

int aml_stb_hw_set_source(struct aml_dvb *dvb, dmx_source_t src)
{
	int ret = 0;
	unsigned long flags;
	
	spin_lock_irqsave(&dvb->slock, flags);

	switch(src) {
		case DMX_SOURCE_FRONT0:
#ifndef CONFIG_AMLOGIC_S2P_TS0
			dvb->stb_source = AM_TS_SRC_TS0;
#else
			dvb->stb_source = AM_TS_SRC_S2P0;
#endif
		break;
		case DMX_SOURCE_FRONT1:
#ifndef CONFIG_AMLOGIC_S2P_TS1
			dvb->stb_source = AM_TS_SRC_TS1;
#else
			dvb->stb_source = AM_TS_SRC_S2P1;
#endif
		break;
		case DMX_SOURCE_FRONT2:
			dvb->stb_source = AM_TS_SRC_TS2;

		break;
		case DMX_SOURCE_DVR0:
			dvb->stb_source = AM_TS_SRC_HIU;
		break;
		default:
			pr_error("illegal demux source %d\n", src);
			ret = -EINVAL;
		break;
	}
	
	if(ret==0)
		dmx_reset_hw(dvb);
	
	spin_unlock_irqrestore(&dvb->slock, flags);
	
	return ret;
}

int aml_dsc_hw_set_source(struct aml_dvb *dvb, dmx_source_t src)
{
	int ret = 0;
	unsigned long flags;
	
	spin_lock_irqsave(&dvb->slock, flags);

	switch(src) {
		case DMX_SOURCE_FRONT0:
#ifndef CONFIG_AMLOGIC_S2P_TS0
			dvb->dsc_source = AM_TS_SRC_TS0;
#else
			dvb->dsc_source = AM_TS_SRC_S2P0;
#endif
		break;
		case DMX_SOURCE_FRONT1:
#ifndef CONFIG_AMLOGIC_S2P_TS1
			dvb->dsc_source = AM_TS_SRC_TS1;
#else
			dvb->dsc_source = AM_TS_SRC_S2P1;
#endif
		break;
		case DMX_SOURCE_DVR0:
			dvb->dsc_source = AM_TS_SRC_HIU;
		break;
		default:
			pr_error("illegal demux source %d\n", src);
			ret = -EINVAL;
		break;
	}
	
	if(ret==0)
		dmx_reset_hw(dvb);
	
	spin_unlock_irqrestore(&dvb->slock, flags);
	
	return ret;
}





static ssize_t dmx_reg_addr_show_source(struct class *class, struct class_attribute *attr,char *buf);
static ssize_t dmx_reg_addr_store_source(struct class *class,struct class_attribute *attr,
                          const char *buf,
                          size_t size);
static ssize_t dmx_id_show_source(struct class *class, struct class_attribute *attr,char *buf);
static ssize_t dmx_id_store_source(struct class *class,struct class_attribute *attr,
                          const char *buf,
                          size_t size);
static ssize_t dmx_reg_value_show_source(struct class *class, struct class_attribute *attr,char *buf);
static ssize_t dmx_reg_value_store_source(struct class *class,struct class_attribute *attr,
                          const char *buf,
                          size_t size);


static int reg_addr=0;
static int dmx_id=0;
static struct class_attribute aml_dmx_class_attrs[] = {
	__ATTR(dmx_id,  S_IRUGO | S_IWUSR, dmx_id_show_source, dmx_id_store_source),	
	__ATTR(register_addr,  S_IRUGO | S_IWUSR, dmx_reg_addr_show_source, dmx_reg_addr_store_source),
	__ATTR(register_value,  S_IRUGO | S_IWUSR, dmx_reg_value_show_source, dmx_reg_value_store_source),
	__ATTR_NULL
};

static struct class aml_dmx_class = {
	.name = "dmx",
	.class_attrs = aml_dmx_class_attrs,
};


static ssize_t dmx_id_show_source(struct class *class, struct class_attribute *attr,char *buf)
{
	int ret;
	ret = sprintf(buf, "%d\n", dmx_id);
	return ret;	
}
static ssize_t dmx_id_store_source(struct class *class,struct class_attribute *attr,
                          const char *buf,
                          size_t size)
{
	int id=0;
	id=simple_strtol(buf,0,16);	

	if(id<0||id>2)
	{
		pr_dbg("dmx id must 0 ~2\n");
	}
	else
	{
		dmx_id=id;
	}
	
	return size;
}


static ssize_t dmx_reg_addr_show_source(struct class *class, struct class_attribute *attr,char *buf)
{
	int ret;
	ret = sprintf(buf, "%x\n", reg_addr);
	return ret;		
}
static ssize_t dmx_reg_addr_store_source(struct class *class,struct class_attribute *attr,
                          const char *buf,
                          size_t size)
{
	int addr=0;
	addr=simple_strtol(buf,0,16);	
	reg_addr=addr;
	return size;	
}


static ssize_t dmx_reg_value_show_source(struct class *class, struct class_attribute *attr,char *buf)
{
	int ret,value;
	value=READ_MPEG_REG(reg_addr);
	ret = sprintf(buf, "%x\n", value);
	return ret;		
}
static ssize_t dmx_reg_value_store_source(struct class *class,struct class_attribute *attr,
                          const char *buf,
                          size_t size)
{
	int value=0;
	value=simple_strtol(buf,0,16);
	WRITE_MPEG_REG(reg_addr,value);
	return size;
}

int aml_regist_dmx_class()
{

	if(class_register(&aml_dmx_class)<0) {
                pr_error("register class error\n");
        }
	
	return 0;
}


int aml_unregist_dmx_class()
{

	class_unregister(&aml_dmx_class);
	return 0;
}



