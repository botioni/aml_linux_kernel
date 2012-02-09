#include <linux/cdev.h>
#include <linux/types.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <asm/uaccess.h>
#include <linux/platform_device.h>
#include "mach/am_regs.h"

#include <linux/efuse.h>
#include "efuse_regs.h"


static void __efuse_write_byte( unsigned long addr, unsigned long data );
static void __efuse_read_dword( unsigned long addr, unsigned long *data);

extern int efuseinfo_num;
extern int efuse_active_version;
extern unsigned efuse_active_customerid;
extern pfn efuse_getinfoex;
extern efuseinfo_t efuseinfo[];

//#define EFUSE_DEBUG

#ifdef EFUSE_DEBUG

static unsigned long efuse_test_buf_32[EFUSE_DWORDS] = {0};
static unsigned char* efuse_test_buf_8 = (unsigned char*)efuse_test_buf_32;

static void __efuse_write_byte_debug(unsigned long addr, unsigned char data)
{
	efuse_test_buf_8[addr] = data;		
}

static void __efuse_read_dword_debug(unsigned long addr, unsigned long *data)    
{
	*data = efuse_test_buf_32[addr >> 2];		
}

void __efuse_debug_init(void)
{
	__efuse_write_byte_debug(0, 0x02);
	__efuse_write_byte_debug(60, 0x02);
	__efuse_write_byte_debug(1, 0x03);
	__efuse_write_byte_debug(61, 0x03);
	__efuse_write_byte_debug(2, 0x00);
	__efuse_write_byte_debug(62, 0x00);
	__efuse_write_byte_debug(3, 0xA3);
	__efuse_write_byte_debug(63, 0xA3);
	__efuse_write_byte_debug(380, 0x01);
	__efuse_write_byte_debug(381, 0x8e);
	__efuse_write_byte_debug(382, 0x0b);
	__efuse_write_byte_debug(383, 0x66);
}
#endif


static void __efuse_write_byte( unsigned long addr, unsigned long data )
{
	unsigned long auto_wr_is_enabled = 0;

	if (READ_CBUS_REG( EFUSE_CNTL1) & (1 << CNTL1_AUTO_WR_ENABLE_BIT)) {                                                                                
		auto_wr_is_enabled = 1;
	} else {                                                                                
		/* temporarily enable Write mode */
		WRITE_CBUS_REG_BITS( EFUSE_CNTL1, CNTL1_AUTO_WR_ENABLE_ON,
		CNTL1_AUTO_WR_ENABLE_BIT, CNTL1_AUTO_WR_ENABLE_SIZE );
	}

	/* write the address */
	WRITE_CBUS_REG_BITS( EFUSE_CNTL1, addr,
	CNTL1_BYTE_ADDR_BIT, CNTL1_BYTE_ADDR_SIZE );
	/* set starting byte address */
	WRITE_CBUS_REG_BITS( EFUSE_CNTL1, CNTL1_BYTE_ADDR_SET_ON,
	CNTL1_BYTE_ADDR_SET_BIT, CNTL1_BYTE_ADDR_SET_SIZE );
	WRITE_CBUS_REG_BITS( EFUSE_CNTL1, CNTL1_BYTE_ADDR_SET_OFF,
	CNTL1_BYTE_ADDR_SET_BIT, CNTL1_BYTE_ADDR_SET_SIZE );

	/* write the byte */
	WRITE_CBUS_REG_BITS( EFUSE_CNTL1, data,
	CNTL1_BYTE_WR_DATA_BIT, CNTL1_BYTE_WR_DATA_SIZE );
	/* start the write process */
	WRITE_CBUS_REG_BITS( EFUSE_CNTL1, CNTL1_AUTO_WR_START_ON,
	CNTL1_AUTO_WR_START_BIT, CNTL1_AUTO_WR_START_SIZE );
	WRITE_CBUS_REG_BITS( EFUSE_CNTL1, CNTL1_AUTO_WR_START_OFF,
	CNTL1_AUTO_WR_START_BIT, CNTL1_AUTO_WR_START_SIZE );
	/* dummy read */
	READ_CBUS_REG( EFUSE_CNTL1 );

	while ( READ_CBUS_REG(EFUSE_CNTL1) & ( 1 << CNTL1_AUTO_WR_BUSY_BIT ) ) {                                                                                
		udelay(1);
	}

	/* if auto write wasn't enabled and we enabled it, then disable it upon exit */
	if (auto_wr_is_enabled == 0 ) {                                                                                
		WRITE_CBUS_REG_BITS( EFUSE_CNTL1, CNTL1_AUTO_WR_ENABLE_OFF,
		CNTL1_AUTO_WR_ENABLE_BIT, CNTL1_AUTO_WR_ENABLE_SIZE );
	}

	//printk(KERN_INFO "__efuse_write_byte: addr=%ld, data=0x%ld\n", addr, data);
}

static void __efuse_read_dword( unsigned long addr, unsigned long *data )
{
	//unsigned long auto_rd_is_enabled = 0;
	

	//if( READ_CBUS_REG(EFUSE_CNTL1) & ( 1 << CNTL1_AUTO_RD_ENABLE_BIT ) ){                                                                               
	//	auto_rd_is_enabled = 1;
	//} else {                                                                               
		/* temporarily enable Read mode */
		WRITE_CBUS_REG_BITS( EFUSE_CNTL1, CNTL1_AUTO_RD_ENABLE_ON,
		CNTL1_AUTO_RD_ENABLE_BIT, CNTL1_AUTO_RD_ENABLE_SIZE );
	//}

	/* write the address */
	WRITE_CBUS_REG_BITS( EFUSE_CNTL1, addr,
	CNTL1_BYTE_ADDR_BIT,  CNTL1_BYTE_ADDR_SIZE );
	/* set starting byte address */
	WRITE_CBUS_REG_BITS( EFUSE_CNTL1, CNTL1_BYTE_ADDR_SET_ON,
	CNTL1_BYTE_ADDR_SET_BIT, CNTL1_BYTE_ADDR_SET_SIZE );
	WRITE_CBUS_REG_BITS( EFUSE_CNTL1, CNTL1_BYTE_ADDR_SET_OFF,
	CNTL1_BYTE_ADDR_SET_BIT, CNTL1_BYTE_ADDR_SET_SIZE );

	/* start the read process */
	WRITE_CBUS_REG_BITS( EFUSE_CNTL1, CNTL1_AUTO_WR_START_ON,
	CNTL1_AUTO_RD_START_BIT, CNTL1_AUTO_RD_START_SIZE );
	WRITE_CBUS_REG_BITS( EFUSE_CNTL1, CNTL1_AUTO_WR_START_OFF,
	CNTL1_AUTO_RD_START_BIT, CNTL1_AUTO_RD_START_SIZE );
	/* dummy read */
	READ_CBUS_REG( EFUSE_CNTL1 );

	while ( READ_CBUS_REG(EFUSE_CNTL1) & ( 1 << CNTL1_AUTO_RD_BUSY_BIT ) ) {                                                                               
		udelay(1);
	}
	/* read the 32-bits value */
	( *data ) = READ_CBUS_REG( EFUSE_CNTL2 );

	/* if auto read wasn't enabled and we enabled it, then disable it upon exit */
	//if ( auto_rd_is_enabled == 0 ){                                                                               
		WRITE_CBUS_REG_BITS( EFUSE_CNTL1, CNTL1_AUTO_RD_ENABLE_OFF,
		CNTL1_AUTO_RD_ENABLE_BIT, CNTL1_AUTO_RD_ENABLE_SIZE );
	//}

	//printk(KERN_INFO "__efuse_read_dword: addr=%ld, data=0x%lx\n", addr, *data);
}

static ssize_t __efuse_read( char *buf, size_t count, loff_t *ppos )
{
	unsigned long* contents = (unsigned long*)kzalloc(sizeof(unsigned long)*EFUSE_DWORDS, GFP_KERNEL);
	unsigned pos = *ppos;
	unsigned long *pdw;
	char* tmp_p;
		
	/*pos may not align to 4*/
	unsigned int dwsize = (count + 3 +  pos%4) >> 2;

	if (!contents) {
		printk(KERN_INFO "memory not enough\n"); 
		return -ENOMEM;
	}	
	if (pos >= EFUSE_BYTES)
		return 0;
	if (count > EFUSE_BYTES - pos)
		count = EFUSE_BYTES - pos;
	if (count > EFUSE_BYTES)
		return -EFAULT;
	
	for (pdw = contents + pos/4; dwsize-- > 0 && pos < EFUSE_BYTES; pos += 4, ++pdw) {
		#ifdef EFUSE_DEBUG     		
		__efuse_read_dword_debug(pos, pdw);
		#else
		/*if pos does not align to 4,  __efuse_read_dword read from next dword, so, discount this un-aligned partition*/
		__efuse_read_dword((pos - pos%4), pdw);
		#endif
	}     
	
	tmp_p = (char*)contents;
    tmp_p += *ppos;                           

	memcpy(buf, tmp_p, count);

	*ppos += count;
	
	if (contents)
		kfree(contents);
	return count;
}

static ssize_t __efuse_write(const char *buf, size_t count, loff_t *ppos )
{
	unsigned pos = *ppos;
	//loff_t *readppos = ppos;
	unsigned char *pc;	

	if (pos >= EFUSE_BYTES)
		return 0;       /* Past EOF */
	if (count > EFUSE_BYTES - pos)
		count = EFUSE_BYTES - pos;
	if (count > EFUSE_BYTES)
		return -EFAULT;
	
	for (pc = buf; count--; ++pos, ++pc){
		#ifdef EFUSE_DEBUG    
         __efuse_write_byte_debug(pos, *pc);  
         #else                             
          __efuse_write_byte(pos, *pc);   
         #endif
    }

	*ppos = pos;

	return (const char *)pc - buf;
}

//=================================================================================================
static int cpu_is_before_m6(void)
{
	unsigned int val;
	asm("mrc p15, 0, %0, c0, c0, 5	@ get MPIDR" : "=r" (val) : : "cc");
	
	return ((val & 0x40000000) == 0x40000000);
}

static int check_if_version0(void)
{
	unsigned long* contents = (unsigned long*)kzalloc(sizeof(unsigned long)*EFUSE_DWORDS, GFP_KERNEL);
	char *op = NULL;
	loff_t pos = 0;
	int i;
	int ret = 1;	
		
	if (!contents) {
		printk(KERN_INFO "memory not enough\n"); 
		return -ENOMEM;
	}	
	op = (char *)contents;

	__efuse_read(op, EFUSE_BYTES, &pos);	
	for(i=0; i<EFUSE_BYTES; i++)
		if((op[i] != 0) && (i!=0) &&(i!=1)&&(i!=2)&&(i!=3)&& (i!=0x40)){
			ret = 0;
			break;
		}
	
	if(contents)
		kfree(contents);
	return ret;
}

static int efuse_readversion(void)
{
	loff_t ppos;
	char ver_buf[4], buf[4];
	
	if(efuse_active_version != -1)
		return efuse_active_version;
	
	memset(ver_buf, 0, sizeof(ver_buf));	
	if(cpu_is_before_m6()){    // M1, M2, M3, A3
		ppos = 380;
		__efuse_read(buf, 4, &ppos);
		efuse_bch_dec(buf, 4, ver_buf, 0);		
		if(ver_buf[0] != 0){
			efuse_active_version = ver_buf[0];
			return ver_buf[0];
		}
		else{   // distinguish free efuse layout and M1/M2 old version
			//if(efuse_is_all_free())
			if(check_if_version0()){
				efuse_active_version = 0;
				return 0;
			}
			else
				return -1;
		}
	}
	else{
		ppos = 3;
		__efuse_read(ver_buf, 1, &ppos);
		if(ver_buf[0] != 0){
			efuse_active_version = ver_buf[0];
			return ver_buf[0];
		}
		else
			return -1;	
	}		
}

static int check_if_bchversion(void)
{
	if(cpu_is_before_m6())
		return 1;
	else
		return 0;
}

//=================================================================================================
// public interface
//=================================================================================================
int efuse_getinfo_byID(unsigned id, efuseinfo_item_t *info)
{
	int ver;
	int i;
	efuseinfo_t *vx = NULL;
	efuseinfo_item_t *item = NULL;
	int size;
	int ret = -1;		
	
	if(id == EFUSE_VERSION_ID){
		strcpy(info->title, "version");		
		if(cpu_is_before_m6()){
			info->offset = 380;		
			info->data_len = 3;			
		}
		else{
				info->offset = 3;		
				info->data_len = 5;		
			}
		return 0;		
	}	
	
		ver = efuse_readversion();
		if(ver < 0){
			printk("efuse version is not selected.\n");
			return -1;
		}
		
		for(i=0; i<efuseinfo_num; i++){
			if(efuseinfo[i].version == ver){
				vx = &(efuseinfo[i]);
				break;
			}				
		}
		if(!vx){
			printk("efuse version %d is not supported.\n", ver);
			return -1;
		}	
		
		item = vx->efuseinfo_version;
		size = vx->size;
		ret = -1;		
		for(i=0; i<size; i++, item++){			
			if(id == item->id){
				strcpy(info->title, item->title);				
				info->offset = item->offset;
				info->id = item->id;				
				info->data_len = item->data_len;								
				ret = 0;
				break;
			}
		}
		
		if((ret < 0) && (efuse_getinfoex != NULL))
			ret = efuse_getinfoex(id, info);		
		if(ret < 0)
			printk("ID:%d is not found.\n", id);
			
		return ret;
}


int check_if_efused(loff_t pos, size_t count)
{
	loff_t local_pos = pos;
	int i;
	unsigned char* buf = NULL;
	unsigned enc_len = count;
	int ret = check_if_bchversion();
	if(ret != 0)
		enc_len += ((count+29)/30);
		
	buf = (unsigned char*)kzalloc(sizeof(char)*enc_len, GFP_KERNEL);
	if (buf) {
		if (__efuse_read(buf, enc_len, &local_pos) == enc_len) {
			for (i = 0; i < enc_len; i++) {
				if (buf[i]) {
					printk("pos %d value is %d", (size_t)(pos + i), buf[i]);
					return 1;
				}
			}
		}
	} else {
		printk("no memory \n");
		return -ENOMEM;
	}
	kfree(buf);
	buf = NULL;
	return 0;
}

int efuse_read_item(char *buf, size_t count, loff_t *ppos)
{	
	int ret;
	unsigned enc_len = count;		
	char* contents = NULL;
	char *penc = NULL;
	char *pdata = NULL;		
	int reverse = 0;
	unsigned pos = (unsigned)*ppos;
	if(pos == 60)  //licence
		reverse = 1;
	ret = check_if_bchversion();
	if(ret != 0)
		enc_len += ((count+29)/30);
	
	contents = (char*)kzalloc(sizeof(char)*EFUSE_BYTES, GFP_KERNEL);
	if (!contents) {
		printk(KERN_INFO "memory not enough\n"); 
		return -ENOMEM;
	}	
	memset(contents, 0, sizeof(contents));	
		
	penc = contents;
	pdata = buf;			
	if(ret != 0){						
		__efuse_read(contents, enc_len, ppos);		
		while(enc_len >= 31){
			efuse_bch_dec(penc, 31, pdata, reverse);
			penc += 31;
			pdata += 30;
			enc_len -= 31;
		}
		if((enc_len > 0))
			efuse_bch_dec(penc, enc_len, pdata, reverse);
	}	
	else
		__efuse_read(buf, enc_len, ppos);	
		
	if(contents)
		kfree(contents);
	return count;	
}

int efuse_write_item(char *buf, size_t count, loff_t *ppos)
{
	int ret;
	unsigned data_len = count;	
	unsigned enc_len = count;
	char* contents = NULL;
	char *pdata = NULL;
	char *penc = NULL;		
	int reverse = 0;
	unsigned pos = (unsigned)*ppos;
	if(pos == 60)
		reverse = 1;
	ret = check_if_bchversion();
	if(ret != 0)
		enc_len += ((count+29)/30);
		
	contents = (char*)kzalloc(sizeof(char)*EFUSE_BYTES, GFP_KERNEL);
	if (!contents) {
		printk(KERN_INFO "memory not enough\n"); 
		return -ENOMEM;
	}	
	memset(contents, 0, sizeof(contents));

	pdata = buf;
	penc = contents;			
	if(ret != 0){				
		while(data_len >= 30){
			efuse_bch_enc(pdata, 30, penc, reverse);
			data_len -= 30;
			pdata += 30;
			penc += 31;		
		}
		if(data_len > 0)
			efuse_bch_enc(pdata, data_len, penc, reverse);
	}	
	else
		memcpy(penc, pdata, data_len);
	
	__efuse_write(contents, enc_len, ppos);
	
	if(contents)
		kfree(contents);
	return count ;		
}

