#include <linux/types.h>
#include <linux/vout/vout_notify.h>
#include  "./header/fbdev.h"

#ifdef  CONFIG_FB_AML_LOGO
//enable  logo_module
#define   dev_to_platformdev(dev)   (container_of((dev), struct platform_device,dev) )
#define   FB_DEV_NAME    "apollofb"
#define TOLOWER(x) ((x) | 0x20)

static	logo_osd_config_t  init_osd_config;
static  logo_osd_dev_t   logo_fb={  \
		.vmode=VMODE_720P};		
static  int  logo_xres_def=1280;
static 	int	 logo_yres_def=720 ;
static  int  logo_osd_type_def=OSD_TYPE_32_ARGB;
static  char vmode[10];

#ifdef   CONFIG_FB_AML_LOGO_ON_OSD0
#define   OSD_INDEX   0
#else
#define   OSD_INDEX   1
#endif

extern struct bus_type platform_bus_type;
static  struct platform_device  * fb_dev;
static  struct resource *mem;

static  int  find_valid_bpp_type(enum osd_type_s type)
{
	int  i;
#ifdef   AML_A1H		
    const enum osd_type_s typeTab[33] = {
        OSD_TYPE_INVALID,OSD_TYPE_INVALID,OSD_TYPE_02_PAL4,
        OSD_TYPE_INVALID, OSD_TYPE_04_PAL16,
        OSD_TYPE_INVALID, OSD_TYPE_INVALID, OSD_TYPE_INVALID, OSD_TYPE_08_PAL256,
        OSD_TYPE_16_655  , OSD_TYPE_16_844   , OSD_TYPE_16_6442   , OSD_TYPE_16_4444_R   ,
        OSD_TYPE_16_4642_R  , OSD_TYPE_16_1555_A   , OSD_TYPE_16_4444_A   , OSD_TYPE_16_565/*16*/   ,
        OSD_TYPE_INVALID, OSD_TYPE_INVALID, OSD_TYPE_24_6666_A   , OSD_TYPE_24_6666_R   ,
        OSD_TYPE_24_8565   , OSD_TYPE_24_5658   , OSD_TYPE_24_888_B   , OSD_TYPE_24_RGB /*24*/  ,
        OSD_TYPE_INVALID,OSD_TYPE_INVALID,OSD_TYPE_INVALID,OSD_TYPE_INVALID,
        OSD_TYPE_32_BGRA  , OSD_TYPE_32_ABGR  , OSD_TYPE_32_RGBA  , OSD_TYPE_32_ARGB /*32*/ ,
    };
#else
	 const enum osd_type_s typeTab[33] = {
	  OSD_TYPE_INVALID  , OSD_TYPE_INVALID  , OSD_TYPE_02_PAL4  ,
        OSD_TYPE_INVALID , OSD_TYPE_04_PAL16 ,
        OSD_TYPE_INVALID, OSD_TYPE_INVALID, OSD_TYPE_INVALID, OSD_TYPE_08_PAL256,
        OSD_TYPE_INVALID  , OSD_TYPE_INVALID   ,OSD_TYPE_INVALID   ,OSD_TYPE_INVALID   ,
        OSD_TYPE_INVALID  , OSD_TYPE_16_6442   , OSD_TYPE_INVALID  , OSD_TYPE_16_655/*16*/   ,
        OSD_TYPE_INVALID  , OSD_TYPE_INVALID  , OSD_TYPE_INVALID   , OSD_TYPE_INVALID   ,
        OSD_TYPE_INVALID  , OSD_TYPE_INVALID  , OSD_TYPE_INVALID  , OSD_TYPE_24_RGB /*24*/  ,
        OSD_TYPE_INVALID  ,OSD_TYPE_INVALID  , OSD_TYPE_INVALID  , OSD_TYPE_INVALID  ,
        OSD_TYPE_INVALID  , OSD_TYPE_INVALID ,OSD_TYPE_INVALID , OSD_TYPE_32_RGBA /*32*/ , 	
    };
#endif 
	for (i=0;i<33;i++)
	{
		if(typeTab[i]==type)
		{
			break;
		}
	}
	return i;
   	
}

logo_osd_dev_t*  get_init_fbdev(void) //export point
{
	if(logo_fb.vmode!=VMODE_MAX)
		return &logo_fb;
	else
		return NULL;	
}

EXPORT_SYMBOL(get_init_fbdev) ;
static int  configure_logo_osd(logo_osd_dev_t* plogo_osd,logo_osd_config_t *config)
{
	vmode_t  vmod;
	if(NULL==plogo_osd || NULL==config)
	{
		printk(KERN_ERR"pointer is null\r\n");
		return -1;
	}
	printk("%s input\r\n",vmode);
	if(vmode[0]!='\0')
	{
		if( (vmod=validate_vmode(vmode))==VMODE_MAX)
		{
			printk("%d select\r\n",vmod);
			return -1;
		}
        plogo_osd->vmode=vmod;
	}	
	plogo_osd->bpp=find_valid_bpp_type(logo_osd_type_def) ;
	printk("%d vmode select\r\n",plogo_osd->vmode);
	memcpy(&plogo_osd->config,config,sizeof(logo_osd_config_t)) ;
	return 0;
	
}
static int match_fb_name(struct device *dev, void *data)
{
	const char *name = data;

	if (strncmp(name, dev->bus_id,strlen(FB_DEV_NAME)) == 0)
		return 1;
	return 0;
}

static  int  get_logo_osd_display_para(logo_osd_config_t *config)
{
	struct  device  *dev;
	
	dev=bus_find_device(&platform_bus_type, NULL, FB_DEV_NAME, match_fb_name) ;
	fb_dev=NULL;
	if(dev)
	{
		fb_dev =dev_to_platformdev(dev) ;
		mem=platform_get_resource(fb_dev,IORESOURCE_MEM,OSD_INDEX);
		printk("mem resource: start=0x%x,end=0x%x\r\n",mem->start,mem->end);
	}
	if(NULL==fb_dev)
	{
		return -1;
	}
	config->osd_ctl.addr=mem->start;
	config->osd_ctl.index=OSD_INDEX ;
	config->osd_ctl.type=logo_osd_type_def;
	config->osd_ctl.xres=logo_xres_def;
	config->osd_ctl.yres=logo_yres_def;
	config->osd_ctl.xres_virtual=logo_xres_def;
	config->osd_ctl.yres_virtual=logo_yres_def;
	
	config->osd_ctl.disp_start_x=0;
	config->osd_ctl.disp_end_x=logo_xres_def -1;
	config->osd_ctl.disp_start_y=0;
	config->osd_ctl.disp_end_y=logo_yres_def-1;

	return 0;
	
}
static inline  int  IS_VALID_CONFIG(logo_osd_config_t *config)
{
#ifdef   AML_A1H //apollo_a1h 
	return ((config->osd_ctl.xres>=720 && config->osd_ctl.xres<=1920) &&\
		    (config->osd_ctl.yres>=480 && config->osd_ctl.yres<=1080) &&\
		    (config->osd_ctl.type<= OSD_TYPE_16_4642_R) ) ;
#else	//apollo
	return ((config->osd_ctl.xres>=720 && config->osd_ctl.xres<=1920) &&\
		    	(config->osd_ctl.yres>=480 && config->osd_ctl.yres<=1080) &&\
			(config->osd_ctl.type<=OSD_TYPE_16_4444_R));	
#endif
}
static int logo_test(void)
{
	   memset((char*)init_osd_config.osd_ctl.addr, 0x00,mem->end - mem->start );
	   return 0;
}

extern int __init logo_init(void);
static int __init  init_logo_fbdev(void) //setup logo fb entry point 
{
	
	printk(KERN_NOTICE"start init_logo_fbdev\r\n");
	if( get_logo_osd_display_para(&init_osd_config) )
	{
		printk(KERN_ERR"can't get framebuffer memory resource\r\n");
		return -1;
	}
	printk(KERN_NOTICE"config:xres=%d,yres=%d,bpp=%d\r\n",init_osd_config.osd_ctl.xres, \
											init_osd_config.osd_ctl.yres, \
											init_osd_config.osd_ctl.type);
	
	if(IS_VALID_CONFIG(&init_osd_config))
	{
		if( configure_logo_osd(&logo_fb,&init_osd_config) != 0)
		{
			printk(KERN_NOTICE"configure tv & osd error \r\n");
			return  -1;
		}
		printk(KERN_NOTICE"xres=%d,yres=%d,bpp=%d\r\n",logo_xres_def,logo_yres_def,logo_osd_type_def);

		set_current_vmode(logo_fb.vmode);
		
		osd_setup(&init_osd_config.osd_ctl, \
					0, \
					0, \
					init_osd_config.osd_ctl.xres, \
					init_osd_config.osd_ctl.yres, \
					init_osd_config.osd_ctl.xres_virtual, \
					init_osd_config.osd_ctl.yres_virtual, \
					init_osd_config.osd_ctl.disp_start_x, \
					init_osd_config.osd_ctl.disp_start_y, \
					init_osd_config.osd_ctl.disp_end_x, \
					init_osd_config.osd_ctl.disp_end_y, \
					init_osd_config.osd_ctl.addr, \
					init_osd_config.osd_ctl.type, \
					init_osd_config.osd_ctl.index) ;
		WRITE_MPEG_REG(VPP_MISC, (1<<(12+OSD_INDEX))|(1<<7)|(0<<4)|(1<<0));
		logo_test();
		logo_init();
		
		
	}

	return 0;
}
int  __init  logo_osd_setup(char *str)
{
	char  *ptr=str;
	char    sep[2];
	char  *option;
	int     count=4;
	char	   find=0;
	
	printk(KERN_NOTICE"logo display:%s\r\n",str);
	do
	{
		if(!isalpha(*ptr)&&!isdigit(*ptr))
		{
			find=1;
			break;
		}
	}while(*++ptr != '\0');
	if(!find) return -1;
	sep[0]=*ptr;
	sep[1]='\0' ;
	while((count--) && (option=strsep(&str,sep)))
	{
		printk(KERN_NOTICE"%s\r\n",option);
		switch(count)
		{
			case 3:
			logo_xres_def=simple_strtoul(option,NULL,0);
			break;
			case 2:
			logo_yres_def=simple_strtoul(option,NULL,0);	
			break;
			case 1:
			logo_osd_type_def=simple_strtoul(option,NULL,0);	
			break;
			case 0:
			if(*option=='c' || *option=='C') //cvbs output
			{
				sprintf(vmode,"%d%s",logo_yres_def,"cvbs");
			}else{
				sprintf(vmode,"%d%c",logo_yres_def,TOLOWER(*option));
			}
			
			break;
		}
		
	}
	printk(KERN_NOTICE"xres=%d,yres=%d,bpp=%d,%s\r\n",logo_xres_def,logo_yres_def,logo_osd_type_def,vmode);
	//init_logo_fbdev();
	return 0;
	
}
__setup("osd=",logo_osd_setup) ;
subsys_initcall_sync(init_logo_fbdev) ;

#else   //
//disable logo module
logo_osd_dev_t*  get_init_fbdev(void) //export point
{
	return NULL;	
}

EXPORT_SYMBOL(get_init_fbdev) ;
#endif



