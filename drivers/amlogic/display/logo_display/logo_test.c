#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/ctype.h>
#include <linux/interrupt.h>
#include <linux/list.h>
#include <linux/timer.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <mach/am_regs.h>
#include <asm/cacheflush.h>
#include <linux/timer.h>
#include <linux/kthread.h>

///#include "bsp.h"
//#include <asm/arch/ddrmem.h>
#include <linux/slab.h>
#include <linux/osd/apollofbdev.h>
//#include <linux/osd/logo_bmp.h>

#undef  DEBUG 
//#define  DEBUG
#define  CUR_MODULE   "logo_test"
#define  BAR_FRAME_COLOR	 0xff220066   //argb
#define  BAR_CLEAR_COLOR	 0xff226600
#define  BAR_CONTENT_COLOR	 0xff008822
#define  BAR_BORDER_WIDTH	 0x2
#define  BAR_FRAME_WIDTH	 400	 	
#define  BAR_FRAME_HEIGHT  	 20
#define  PIC_BAR_GAP			 BAR_FRAME_HEIGHT




#ifndef  DEBUG
#define  Dprintk(x...)   
#else
#define  Dprintk(x...)  printk(x)
#endif
typedef struct {
 u32  x;
 u32 y;
 u32 w;
 u32 h;
}rect_t;

u32 *logo_bmp; //need change
extern  logo_osd_dev_t*  get_init_fbdev(void) ;
static struct timer_list   logo_timer;
static  int  boot_process;
static int line_length ;
static  char *p_osd_addr;

static  rect_t  bar_rect,pic_rect,mov_rect ;
static u32  _convert_color(u32  bpp, u32  in_color)
{
	u8 a,r,g,b ;
	u32 out_color=0 ;
	
	a=in_color>>24 & 0xff ;
	r=in_color>>16 & 0xff ;  
	g=in_color>>8  & 0xff ;
	b=in_color&0xff ;
	
	switch(bpp)
	{
		case  BPP_TYPE_32_RGBA:
    		out_color=a|b<<8|g<<16|r<<24;//special order.
    		break;
		case BPP_TYPE_24_RGB:
		out_color=(r<<16|g<<8|b);
		break;
		case BPP_TYPE_16_655:
		out_color=((b>>3&0x1f)|(g>>3&0x1f)<<5|(r>>2&0x3f)<<10);		
		break;
		case BPP_TYPE_16_565:
		out_color=((b>>3&0x1f)|(g>>2&0x3f)<<5|(r>>3&0x1f)<<11);		
		break;
		case BPP_TYPE_16_6442:
		out_color=(((a>>6)&0x3)|((b>>4&0xf)<<2)|((g>>4&0xf)<<6)|((r>>2&0x3f)<<10));	
		break;
		case BPP_TYPE_16_844:
		out_color=((b>>4&0xf)<<0|(g>>4&0xf)<<4|(r)<<8);	
		break;	
		case BPP_TYPE_16_4444_R:
		out_color=((a>>4&0xf)|(b>>4&0xf)<<4|(g>>4&0xf)<<8|(r>>4&0xf)<<12);	
		break;	
		case BPP_TYPE_16_4642_R:
		out_color=((a>>6&0x3)|(b>>4&0xf)<<2|(g>>2&0x3f)<<6|(r>>4&0xf)<<12);	
		break;
		case BPP_TYPE_16_1555_A:
		out_color=((a>>7&0x1)<<15|(b>>3&0x1f)|(g>>3&0x1f)<<5|(r>>3&0x1f)<<10);	
		break;
		case BPP_TYPE_16_4444_A:
		out_color=((a>>4&0xf)<<12|(b>>4&0xf)|(g>>4&0xf)<<4|(r>>4&0xf)<<8);	
		break;
		case BPP_TYPE_24_6666_A:
		out_color=((a>>2&0x3f)<<18|(b>>2&0x3f)|(g>>2&0x3f)<<6|(r>>2&0x3f)<<12);	
		break;
		case BPP_TYPE_24_6666_R:
		out_color=((a>>2&0x3f)|(b>>2&0x3f)<<6|(g>>2&0x3f)<<12|(r>>2&0x3f)<<18);	
		break;
		case BPP_TYPE_24_8565:
		out_color=((a)<<16|(b>>3&0x1f)|(g>>2&0x3f)<<5|(r>>3&0x1f)<<11);	
		break;
		case BPP_TYPE_24_5658:
		out_color=((a)|(b>>3&0x1f)<<8|(g>>2&0x3f)<<13|(r>>3&0x1f)<<19);	
		break;
		case BPP_TYPE_24_888_B:
		out_color=((b)<<16|(g)<<8|(r));	
		break;
		case BPP_TYPE_32_BGRA:
		out_color=((a)|(b)<<24|(g)<<16|(r)<<8);	
		break;
		case BPP_TYPE_32_ABGR:
		out_color=((a)<<24|(b)<<16|(g)<<8|(r));	
		break;	
		case BPP_TYPE_32_ARGB:
		out_color=((a)<<24|(b)|(g)<<8|(r)<<16);	
		break;
	}
	return out_color ;
}
static void _fill_rect (u8 *p, u32 line_width, u32 x0, u32 y0,u32 x1,u32 y1, u32 bpp,u32 color)
{
    u32  x, y;
    u8 *wp = p;

  	
	if (bpp <= BPP_TYPE_32_ARGB && bpp >= BPP_TYPE_32_BGRA )
	{
		Dprintk(CUR_MODULE"input 32 bpp\r\n")	;
		 for (y = y0; y < y1; y++) {
      		  wp =(u8*) (p+ y * line_width + x0*4) ;
		   for (x = x0; x <=x1 ; x++) {
            			*wp++ = color&0xff;  
               		*wp++ = color>>8&0xff;   
                		*wp++ = color>>16&0xff;  
				*wp++ = color>>24&0xff;
	
              	}
			 
		   }
	}
	if (bpp <= BPP_TYPE_24_RGB && bpp >=BPP_TYPE_24_6666_A ) {
	    Dprintk(CUR_MODULE"input 24 bpp\r\n")	;
	     for (y = y0; y < y1; y++) {
      		  wp = (u8*)(p +y * line_width + x0*3 );
           	  for (x = x0; x <=x1 ; x++) {
            			*wp++ = color&0xff;  
               		*wp++ = color>>8&0xff;   
                		*wp++ = color>>16&0xff;   
              	}
		  
	     	}
    	}
   	 else if (bpp >=BPP_TYPE_16_655 && bpp <=BPP_TYPE_16_565) {
	 	
	 	Dprintk(CUR_MODULE"input 16 bpp: index%d\r\n",bpp)	;
	     for (y = y0; y <y1; y++) {
            wp = (u8*)(p+y * line_width + x0*2) ;
            for (x = x0; x < x1; x++) {
			*wp++ = color&0xff;		
           	 	*wp++ = color>>8&0xff;
             }
		 
	    }
    	}
	
	//Dprintk(CUR_MODULE"fill rect compelted\r\n",bpp)	;
	
	 
}
static  void  reset_process_bar(logo_osd_dev_t  *osd_dev)
{
	u32  bar_frame_color=_convert_color(osd_dev->bpp,BAR_FRAME_COLOR);
	u32  clear_frame_color=_convert_color(osd_dev->bpp,BAR_CLEAR_COLOR);
	int   bpp=osd_dev->bpp;//bytes per pixel
	int   Bpp=(bpp >8?(bpp>16?(bpp>24?4:3):2):1);
	
	line_length=(osd_dev->config.osd_ctl.xres_virtual)*Bpp; 
	p_osd_addr=(char *)ioremap_nocache(osd_dev->config.osd_ctl.addr,line_length*osd_dev->config.osd_ctl.yres_virtual)   ;
	
	Dprintk(CUR_MODULE"addr:0x%x ===line length :%d \r\n",(unsigned int)p_osd_addr,line_length);
	//_fill_rect((char*)p_osd_addr,line_length,0,00,720,576,24,0xff000000);
	//_fill_rect((char*)p_osd_addr,line_length,100,15,400,25,24,0xffff0000);
	//_fill_rect((char*)p_osd_addr,line_length,0,25,104,27,24,0xffff0000);
	//_fill_rect((char*)p_osd_addr,line_length,100,27,400,37,24,_convert_color(0xffff0000));
	_fill_rect((char*)p_osd_addr,line_length,bar_rect.x,bar_rect.y, \
												bar_rect.x+bar_rect.w, \
												bar_rect.y+bar_rect.h,osd_dev->bpp, \
												bar_frame_color);
	_fill_rect((char*)p_osd_addr,line_length,bar_rect.x + BAR_BORDER_WIDTH ,bar_rect.y + BAR_BORDER_WIDTH , \
												bar_rect.x+bar_rect.w - BAR_BORDER_WIDTH , \
												bar_rect.y+bar_rect.h - BAR_BORDER_WIDTH, \
												osd_dev->bpp,clear_frame_color);
			
		
}
static  void  setup_object_pos(logo_osd_dev_t  *osd_dev)
{
	
	pic_rect.x=(osd_dev->config.osd_ctl.xres - logo_bmp[0])>>1 ;//bmp[0] width
	pic_rect.y=(osd_dev->config.osd_ctl.yres-(logo_bmp[1]+BAR_FRAME_HEIGHT + PIC_BAR_GAP)) >>1 ; 
	pic_rect.w=logo_bmp[0] ;
	pic_rect.h=logo_bmp[1];

	bar_rect.x=(osd_dev->config.osd_ctl.xres-BAR_FRAME_WIDTH) >>1;
	bar_rect.y=pic_rect.y + pic_rect.h + PIC_BAR_GAP;
	bar_rect.w=BAR_FRAME_WIDTH;
	bar_rect.h=BAR_FRAME_HEIGHT;

	Dprintk("%s.pic_rect=>%d:%d:%d:%d---bar_rect=>%d:%d:%d:%d\r\n",__func__,pic_rect.x,pic_rect.y,pic_rect.w,pic_rect.h ,\
													bar_rect.x,bar_rect.y,bar_rect.w,bar_rect.h)	;
	mov_rect.x =bar_rect.x  + (BAR_BORDER_WIDTH) + 2 ;
	mov_rect.y =bar_rect.y  + (BAR_BORDER_WIDTH) + 2 ;
	mov_rect.w = (bar_rect.w - (BAR_BORDER_WIDTH<<1) -4)/10 ;
	mov_rect.h = BAR_FRAME_HEIGHT - (BAR_BORDER_WIDTH <<1) -4 ;
	
		
}
/********************************************************
*we will load bitmap data into buffer which is pointed by bmp and the data 
*will be convert to match color format first .
*********************************************************/
int  load_bitmap_local(char *bmp , int bpp)
{
	
	u32	*bmp_data_32,*buf_bmp_32;
	u16 *buf_bmp_16;
	u8	r,g,b;
	u32 i,j;
	u32  width=logo_bmp[0],height=logo_bmp[1];

	bmp_data_32=(u32*)&logo_bmp[2]; 
	buf_bmp_32=(u32*)bmp;
	buf_bmp_16=(u16*)bmp;
	for(i=0;i<height;i++)
	for(j=0;j<width;j++)
	{
		r=(*bmp_data_32&0x00ff0000)>>16; //bitmap data is 655 format
		g=(*bmp_data_32&0x0000ff00)>>8;
		b=(*bmp_data_32&0x000000ff);
		switch(bpp)
		{
			case  BPP_TYPE_32_RGBA:
    			*buf_bmp_32=0xff|b<<8|g<<16|r<<24;//special order.
    			break;
			case BPP_TYPE_32_BGRA:
			*buf_bmp_32=(0xff|(b)<<24|(g)<<16|(r)<<8);	
			break;
			case BPP_TYPE_32_ABGR:
			*buf_bmp_32=(0xff<<24|(b)<<16|(g)<<8|(r));	
			break;	
			case BPP_TYPE_32_ARGB:
			*buf_bmp_32=(0xff<<24|(b)|(g)<<8|(r)<<16);	
			break;	
			case BPP_TYPE_24_888_B:
			*buf_bmp_32=((b)<<16|(g)<<8|(r));	
			break;
			case BPP_TYPE_24_5658:
			*buf_bmp_32=(0xff|(b>>3&0x1f)<<8|(g>>2&0x3f)<<13|(r>>3&0x1f)<<19);	
			break;
			case BPP_TYPE_24_6666_A:
			*buf_bmp_32=((0xff>>2&0x3f)<<18|(b>>2&0x3f)|(g>>2&0x3f)<<6|(r>>2&0x3f)<<12);	
			break;
			case BPP_TYPE_24_6666_R:
			*buf_bmp_32=((0xff>>2&0x3f)|(b>>2&0x3f)<<6|(g>>2&0x3f)<<12|(r>>2&0x3f)<<18);	
			break;
			case BPP_TYPE_24_8565:
			*buf_bmp_32=((0xff)<<16|(b>>3&0x1f)|(g>>2&0x3f)<<5|(r>>3&0x1f)<<11);	
			break;
			case BPP_TYPE_24_RGB:
			*buf_bmp_32=(r<<16|g<<8|b);
			break;
			case BPP_TYPE_16_655:
			//*buf_bmp_16=(b>>1|(g)<<5|(r<<1)<<10);	
			*buf_bmp_16=b|(g)<<5|(r<<10);
			break;	
			case BPP_TYPE_16_565:
			*buf_bmp_16=((b>>3&0x1f)|(g>>2&0x3f)<<5|(r>>3&0x1f)<<11);;		
	 		break;
			case BPP_TYPE_16_6442:
			*buf_bmp_16=(3|(b>>1)<<2|(g>>1)<<6|r<<10);//alpha =3	
			break;
			case BPP_TYPE_16_844:
			*buf_bmp_16=((b>>1)<<0|(g>>1)<<4|(r<<2)<<8);	
			break;
			case BPP_TYPE_16_4444_R:
			*buf_bmp_16=(0xf|(b>>1&0xf)<<4|(g>>1&0xf)<<8|(r>>2)<<12);	
			break;	
			case BPP_TYPE_16_4642_R:
			*buf_bmp_16=(0x3|(b>>1&0xf)<<2|(g<<1&0x3f)<<6|(r>>2&0xf)<<12);	
			break;
			case BPP_TYPE_16_1555_A:
			*buf_bmp_16=(1<<15|(b&0x1f)|(g&0x1f)<<5|(r>>1&0x1f)<<10);	
			break;
			case BPP_TYPE_16_4444_A:
			*buf_bmp_16=(0xf<<12|(b>>1&0xf)|(g>>1&0xf)<<4|(r>>2&0xf)<<8);	
			break;
			
		}
		bmp_data_32++;
		buf_bmp_32++;
		buf_bmp_16++;
		
	}
	return 0;
}
/************************************************************
*
*	func name : show_bitmap
*		para	  :  
*				p			framebuffer pointer
*				line_width	osd fix line width
*				xres			osd width
*				height			osd height
*				bpp			osd bits per pixel
*
*************************************************************/

 int  show_bitmap(char *p, unsigned line_width,unsigned x0,unsigned y0, unsigned width, unsigned height, unsigned bpp)
{
	int x, y;
    	char *wp = p;
 	int  *bmp;
	int * rec_pos;
	int  BMP_WIDTH=logo_bmp[0],BMP_HEIGHT=logo_bmp[1];
	
	bmp=(int*)kmalloc(BMP_WIDTH*BMP_HEIGHT*4,GFP_KERNEL);
	if(NULL==bmp) 
	{
		Dprintk("Error can't alloc memory for bmp buffer\r\n");
		return -1 ;
	}

	
	load_bitmap_local((char*)bmp,bpp);

	height =min(height,y0+BMP_HEIGHT);
	width =min(width,x0+BMP_WIDTH);
    	if (bpp <= BPP_TYPE_32_ARGB && bpp >= BPP_TYPE_32_BGRA ){
	 int * bmp32=bmp;
	 rec_pos=bmp32+BMP_WIDTH*(BMP_HEIGHT-1) ;
	 
        for (y = y0 ; y <height; y++) {
            wp = &p[y * line_width+ 4*x0];
		bmp32=rec_pos;
	      for (x = x0; x < width; x++) {
		   	
		   *wp++ = *bmp32&0xff;   
                *wp++ = *bmp32>>8&0xff;   
                *wp++ = *bmp32>>16&0xff;   
                *wp++ = *bmp32>>24&0xff;   
		   bmp32++;	
		}
		rec_pos-=BMP_WIDTH;  
	      		
        }
    }
     else if (bpp >=BPP_TYPE_16_655 && bpp <=BPP_TYPE_16_565) {
	 short* bmp16=(short *)bmp;
	  
	  rec_pos=(int *)(bmp16+BMP_WIDTH*(BMP_HEIGHT-1));
        for (y = y0; y < height; y++) {
            wp = &p[y * line_width +x0*2];
		bmp16=(short *)rec_pos ;	
            for (x = x0; x < width; x++) {
		*wp++ = *bmp16 &0xff;	
		*wp++ =*bmp16>>8&0xff;	
		
	       bmp16++;
                
            }
		rec_pos-=BMP_WIDTH/(sizeof(int)/sizeof(short));	
			
        }
    }
    else if (bpp <= BPP_TYPE_24_RGB && bpp >=BPP_TYPE_24_6666_A ) {
	 int * bmp24=bmp;	
	rec_pos=bmp24+(BMP_WIDTH)*(BMP_HEIGHT-1)  ;
        for (y = y0; y < height; y++) {
            wp = &p[y * line_width+x0*3];
		bmp24=rec_pos ;		
            for (x = x0; x < width  ; x++) {
      		      *wp++ = *bmp24&0xff;  
	      		*wp++ = *bmp24>>8&0xff;   
                	*wp++ = *bmp24>>16&0xff;  
		   	bmp24++;	
			 
            }
		rec_pos-=BMP_WIDTH;  		
        }
    }
	kfree(bmp);
	return 0;
}

static void  display_logo(void *para)
{
	logo_osd_dev_t  *osd_dev=(logo_osd_dev_t  *)para ;
	Dprintk("%s.logo display=>x:%d,y:%d,w:%d,h:%d\r\n",__func__,pic_rect.x,pic_rect.y,osd_dev->config.osd_ctl.xres,osd_dev->config.osd_ctl.yres);
	show_bitmap(p_osd_addr,line_length,pic_rect.x,pic_rect.y,osd_dev->config.osd_ctl.xres,osd_dev->config.osd_ctl.yres,osd_dev->bpp);
}
static void  display_process(unsigned long  para)
{
	logo_osd_dev_t  *osd_dev=(logo_osd_dev_t  *)para ;

	u32  bar_content_color=_convert_color(osd_dev->bpp,BAR_CONTENT_COLOR);
	
	boot_process++;
	Dprintk(CUR_MODULE"+++++process addr 0x%x, bar_content_color %d\r\n",osd_dev->config.osd_ctl.addr ,bar_content_color);
	if(boot_process<10)
	mod_timer(&logo_timer,jiffies+HZ/2);
	
	Dprintk(CUR_MODULE"x:%d,y:%d,w:%d,h:%d\r\n",mov_rect.x,mov_rect.y,mov_rect.w,mov_rect.h);
	
	_fill_rect( (char*) p_osd_addr ,line_length,mov_rect.x , mov_rect.y   , \
												mov_rect.x+ mov_rect.w , \
												mov_rect.y +mov_rect.h, \
												osd_dev->bpp,bar_content_color);
	
	mov_rect.x+=mov_rect.w;
	pic_rect.x+=20;
										
	
}


static int logo_test(void  *para)
{
	//display a progressive bar .
	logo_osd_dev_t  *osd_dev=para ;	
	Dprintk(CUR_MODULE"logo_test start\r\n");
	setup_timer(&logo_timer, display_process, (unsigned long)osd_dev) ;
	boot_process=0;
	mod_timer(&logo_timer,jiffies+HZ/4);
	setup_object_pos(osd_dev);
	reset_process_bar(osd_dev);
	display_logo(osd_dev);
	
	return 0;
}


 int  logo_init(void)
{
	logo_osd_dev_t  *osd_dev;

	Dprintk(CUR_MODULE"+++++enter logo test\r\n");

	osd_dev=get_init_fbdev();
	if(osd_dev)
	{
		kthread_run(logo_test,osd_dev,"logo_test");
	}
	
	return 0;
}
static void __exit logo_exit(void)
{
	Dprintk(CUR_MODULE"+++++++exit logo test\r\n");
	iounmap(p_osd_addr);
}
__exitcall(logo_exit);
EXPORT_SYMBOL(logo_init) ;
MODULE_DESCRIPTION("display configure  module");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("jianfeng_wang <jianfeng.wang@amlogic.com>");

