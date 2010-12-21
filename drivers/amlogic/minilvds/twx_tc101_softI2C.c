#if 1

#define   sIIC_SDA_BIT			20
#define	sIIC_SCL_BIT			19
#define 	T101ADDR   			0xfc


static void setbitval(unsigned add,unsigned bit,unsigned val)
{
    add = add | 0xc0000000;
    val?(*(volatile unsigned *)add|=(1<<bit)):(*(volatile unsigned *)add&=~(1<<bit));
}
static unsigned getbitval(unsigned add, unsigned bit)
{	
		unsigned ret;
    add = add | 0xc0000000;
    ret = ( (*(volatile unsigned *)add) & (1<<bit) )? 1:0;
    return ret;
}

static void set_scl(unsigned _scl)
{
		setbitval(0x01108034, sIIC_SCL_BIT, _scl);
}

static void set_sda(unsigned _sda)
{
		setbitval(0x01108034, sIIC_SDA_BIT, _sda);	
} 

static unsigned get_scl(void)
{
		return getbitval(0x01108038, sIIC_SCL_BIT);	
}

static unsigned get_sda(void)
{
		return getbitval(0x01108038, sIIC_SDA_BIT);	
}

/*
		en[0]: output
		en[1]: input
*/
static void sw_sda_in(unsigned en)
{
		setbitval(0x01108030, sIIC_SDA_BIT, en);
}

static void sw_scl_in(unsigned en)
{
		setbitval(0x01108030, sIIC_SCL_BIT, en);
}




#define IIC_DELAY_TIME		10
#define TRUE			1
#define FALSE   		0
#define IIC_RETRY_TIME		2
#define IIC_PAGE_SIZE  		8
#define IIC_PAGE_MOD   		7

static void delay_us_foriic(int count)
{
	int i=0;
	int j=0;
	int k=0;
	for(i=count;i>0;i--)
	{
		udelay(2);
	}
}

static void IICDelay(unsigned count)
{
		delay_us_foriic(count);
}


static void IICStart(void)
{
    		set_sda(TRUE);
		set_scl(TRUE);
		IICDelay(IIC_DELAY_TIME);
		
		set_sda(FALSE);
		IICDelay(IIC_DELAY_TIME);
    		set_scl(FALSE);
		IICDelay(IIC_DELAY_TIME);
    return ;
}
 
static void IICRStart(void)
{
	IICDelay(IIC_DELAY_TIME);
    	set_scl(TRUE);
	IICDelay(IIC_DELAY_TIME);
    	set_sda(TRUE);
	IICDelay(IIC_DELAY_TIME);
	set_sda(FALSE);
	IICDelay(IIC_DELAY_TIME);
    	set_scl(FALSE);
 	IICDelay(IIC_DELAY_TIME);

    return ;
}

static void IICStop(void)
{
    /* A LOW to HIGH transition on the SDA line while SCL is
     * HIGH defines a STOP condition.
     */
    	set_scl(FALSE);
 	IICDelay(IIC_DELAY_TIME);

    	set_sda(FALSE);
 	IICDelay(IIC_DELAY_TIME);

    	set_scl(TRUE);
 	IICDelay(IIC_DELAY_TIME);

    	set_sda(TRUE);
 	IICDelay(IIC_DELAY_TIME);
}


static unsigned IICTAck(void)
{
    int i, ret = FALSE;
	
    set_sda(FALSE);
    IICDelay(IIC_DELAY_TIME);		
    set_scl(FALSE);
    IICDelay(IIC_DELAY_TIME);
    sw_sda_in(1);
    IICDelay(IIC_DELAY_TIME);

    // give an scl pulse to make slave release SDA 
    set_scl(TRUE);
//    sw_sda_in(TRUE);
    IICDelay(IIC_DELAY_TIME);   //
		
	  for(i=0; i<200; i++)
	  {
	  
              if(!get_sda())
      	       {
                  ret = TRUE;
                  break;
              }
		udelay(10);
         };  // wait for ack 
         
		sw_sda_in(FALSE);
 		IICDelay(IIC_DELAY_TIME);

    		set_scl(FALSE);
 		IICDelay(20);
		set_sda(TRUE);
		IICDelay(10);


    return ret;
}


static void IICNoAck(void)
{
    	set_scl(FALSE);
 	IICDelay(IIC_DELAY_TIME);

    	set_sda(TRUE);
 	IICDelay(IIC_DELAY_TIME);

    	set_scl(TRUE);
 	IICDelay(IIC_DELAY_TIME);

    	set_scl(FALSE);
 	IICDelay(IIC_DELAY_TIME);

    return ;
}

static void IICAck(void)
{

    	set_scl(FALSE);
 	IICDelay(IIC_DELAY_TIME);

    	set_sda(FALSE);
 	IICDelay(IIC_DELAY_TIME);

    	set_scl(TRUE);
  	IICDelay(IIC_DELAY_TIME);

    	set_scl(FALSE);
  	IICDelay(IIC_DELAY_TIME);

   	return ;
}


static unsigned char IICRead8Bit(void)
{

    unsigned char rbyte = 0;
    int i;
    set_scl(FALSE);
 	IICDelay(2*IIC_DELAY_TIME);
  	sw_sda_in(TRUE);
    for (i = 0; i < 8; i++) 
	{
        get_sda();
        set_scl(TRUE);
 	IICDelay(4*IIC_DELAY_TIME);

        rbyte = (rbyte<<1) | ((unsigned char)(get_sda()));
        set_scl(FALSE);
 	IICDelay(3*IIC_DELAY_TIME);
    }
  	sw_sda_in(FALSE);

    return rbyte;
}

static unsigned char IICWrite8Bit(unsigned char wbyte)
{
    int i;
    
    set_scl(FALSE);
 	IICDelay(IIC_DELAY_TIME);
    for(i = 0x80; i; i >>= 1) 
    {
        if(wbyte & i)
            set_sda(TRUE);
        else
            set_sda(FALSE);
        set_scl(TRUE);
        
 	IICDelay(3*IIC_DELAY_TIME);
	   	 	
	set_scl(FALSE);
 	IICDelay(2*IIC_DELAY_TIME);
    }

    set_scl(FALSE);
 	IICDelay(2*IIC_DELAY_TIME);

    return 0;
}


void IICInit(void)
{
		setbitval(0x011080b8, 0, 0);
		setbitval(0x011080b8, 1, 0);
		setbitval(0x011080b8, 2, 0);
		setbitval(0x011080b8, 3, 0);
		setbitval(0x011080b8, 4, 0);
		setbitval(0x011080b8, 5, 0);
		setbitval(0x011080b8, 6, 0);
		setbitval(0x011080b8, 7, 0);
		setbitval(0x011080b4, 31, 0);
		setbitval(0x011080C8, 28, 0);		
		setbitval(0x011080c8, 29, 0);
		setbitval(0x01108030, 20, 0);		
		setbitval(0x01108030, 19, 0);
		setbitval(0x01108034, 20, 1);		
		setbitval(0x01108034, 19, 1);
		
		sw_sda_in(0);
		sw_scl_in(0);
}


unsigned char IIC_Read_forT101(unsigned device_id, unsigned int rw_addr, unsigned char *rw_buff, unsigned int rw_bytes)
{
//#ifdef (PMP_PROJECT)
    unsigned char retry_times = IIC_RETRY_TIME ;
    unsigned int read_bytes ;
    unsigned char read_device_addr ;
    int ret = FALSE ;

    read_device_addr = device_id | ((((rw_addr>>8)&0x7)<<1) & 0xff);

    while(retry_times > 0)
    {
        retry_times--;

        if((rw_addr&0xffff) != 0xffff) 
        {
            /* write sequence read address to device first */
            IICStart() ;
    
            /* select device, write */
            IICWrite8Bit(read_device_addr) ;
            if(!IICTAck())
						{
                continue;
            }
    
    
//            IICWrite8Bit(rw_addr & 0xff);
	     IICWrite8Bit(rw_addr / 256);
           if(!IICTAck()){
                continue;
            }
            
		IICWrite8Bit(rw_addr % 256);
            if(!IICTAck()){
                continue;
            }	
        }
        /* read */
        IICRStart() ;

        /* select device, read */
        IICWrite8Bit(read_device_addr | 0x01) ;
        if(!IICTAck()){
            //test ack fail
            continue;
        }

        read_bytes = rw_bytes ;
		
 	IICDelay(2*IIC_DELAY_TIME);
		
        while (read_bytes > 1) 
        {
            read_bytes--;
            *rw_buff++ = IICRead8Bit();
            IICAck();
        }
        *rw_buff = IICRead8Bit();        //read last byte data
        IICNoAck();
        ret = TRUE;
        break;
    }

    IICStop() ;
	 return ret ;
//#else
//	return 0;
//#endif
   
}

unsigned char  IIC_Write_forT101(unsigned device_id, unsigned int rw_addr, unsigned char *rw_buff,  unsigned int rw_bytes)
{
//#ifdef (PMP_PROJECT)
    unsigned char retry_times = IIC_RETRY_TIME ;
    unsigned int writed_bytes, page_size, write_addr ;
    unsigned char write_device_addr ;
    unsigned char retry_time_per_byte = IIC_RETRY_TIME;
    int ret = TRUE;
    writed_bytes = 0;
    while((retry_times > 0) && (writed_bytes < rw_bytes)) {
        retry_times--;
        
        writed_bytes = 0 ;
        ret = TRUE;
        
        while ((writed_bytes < rw_bytes) && (retry_time_per_byte > 0)){
            retry_time_per_byte--;
            write_addr = rw_addr + writed_bytes;
            write_device_addr = device_id |((((write_addr>>8)&0x7)<<1) & 0xff);
            if((rw_addr&0xff) == 0xff) {
                page_size = rw_bytes;
            }
            else {
                page_size = IIC_PAGE_SIZE - (write_addr & IIC_PAGE_MOD) ;
            }
            
            IICStart() ;
            
            /* write operation, bit[0] = 0 */
            IICWrite8Bit(write_device_addr);
            if(!IICTAck()){
                // test ack fail
                ret = FALSE;
                break;
            }
            
            if((rw_addr&0xff) != 0xff)           
            //weather to send the memory address
            {
//                IICWrite8Bit(write_addr & 0xff);
		  IICWrite8Bit(write_addr /256);
                if(!IICTAck()){
                    //test ack fail
                    ret = FALSE;
                    break ;
                }
                
		  IICWrite8Bit(write_addr % 256);
                if(!IICTAck()){
                    //test ack fail
                    ret = FALSE;
                    break ;
                }				
            }
            while(page_size > 0){
                page_size--;
                IICWrite8Bit(rw_buff[writed_bytes]) ;
                if(!IICTAck()){
                    //test ack fail
                    ret = FALSE ;
                    break ;
                }
                writed_bytes++;
                retry_time_per_byte = IIC_RETRY_TIME;
                if (writed_bytes == rw_bytes) {
                    break;
                }
            }
            
            IICStop() ;
            
            /* max 1ms */ 
 						IICDelay(20*IIC_DELAY_TIME);
        
        } /* paged write */
    }


    return ret;
//#else
//	return 0;
//#endif
}
#endif

void init_Tc101(void)
{
	unsigned char tData[2];
	int a=1,k=0;
	
			tData[0] = 0xb2;
			IIC_Write_forT101(T101ADDR, 0xf830, tData, 1);
			tData[0] = 0xc2;
			IIC_Write_forT101(T101ADDR, 0xf833, tData, 1);
			tData[0] = 0xf0;
			IIC_Write_forT101(T101ADDR, 0xf831, tData, 1);			
			tData[0] = 0x80;
			IIC_Write_forT101(T101ADDR, 0xf840, tData, 1);	
//			tData[0] = 0x02;
//			IIC_Write_forT101(T101ADDR, 0xf841, tData, 1);
			tData[0] = 0xec;
			IIC_Write_forT101(T101ADDR, 0xf881, tData, 1);
//			tData[0] = 0x18;
//			IIC_Write_forT101(T101ADDR, 0xf882, tData, 1);
//			tData[0] = 0x41;
//			IIC_Write_forT101(T101ADDR, 0xf820, tData, 1);

			
			IIC_Read_forT101(T101ADDR,0xf830,tData,1); 
		//	AVTimeDly(10);
			IIC_Read_forT101(T101ADDR,0xf833,tData,1); 
			IIC_Read_forT101(T101ADDR,0xf831,tData,1);
		//	AVTimeDly(10);
			IIC_Read_forT101(T101ADDR,0xf840,tData,1); 
		//	AVTimeDly(10);
			IIC_Read_forT101(T101ADDR,0xf841,tData,1); 
		//	AVTimeDly(10);
			IIC_Read_forT101(T101ADDR,0xf881,tData,1); 
		//	AVTimeDly(10);
		//	IIC_Read_forT101(T101ADDR,0xf882,tData,1); 
		//	AVTimeDly(10);
		//	IIC_Read_forT101(T101ADDR,0xf820,tData,1); 
		//	AVTimeDly(10);
		
}

void test_twx_tc101(void)
{
	IICInit();
	init_Tc101();
}

