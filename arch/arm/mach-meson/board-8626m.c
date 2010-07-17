/*
 *
 * arch/arm/mach-meson/meson.c
 *
 *  Copyright (C) 2010 AMLOGIC, INC.
 *
 * License terms: GNU General Public License (GPL) version 2
 * Platform machine definition.
 */
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/mm.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/ioport.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <mach/hardware.h>
#include <mach/platform.h>
#include <mach/memory.h>
#include <mach/memory.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <mach/lm.h>

#include "board-8626m.h"

#ifdef CONFIG_CACHE_L2X0
#include <asm/hardware/cache-l2x0.h>
#endif
#if defined(CONFIG_JPEGLOGO)
static struct resource jpeglogo_resources[] = {
    [0] = {
        .start = CONFIG_JPEGLOGO_ADDR,
        .end   = CONFIG_JPEGLOGO_ADDR + CONFIG_JPEGLOGO_SIZE - 1,
        .flags = IORESOURCE_MEM,
    },
    [1] = {
        .start = CODEC_ADDR_START,
        .end   = CODEC_ADDR_END,
        .flags = IORESOURCE_MEM,
    },
};

static struct platform_device jpeglogo_dev = {
	.name = "jpeglogo-dev",
	.id   = 0,
    .num_resources = ARRAY_SIZE(jpeglogo_resources),
    .resource      = jpeglogo_resources,
};
#endif

#ifdef CONFIG_FB_AM
static struct resource fb_device_resources[] = {
    [0] = {
        .start = OSD1_ADDR_START,
        .end   = OSD1_ADDR_END,
        .flags = IORESOURCE_MEM,
    },
    [1] ={ //for osd2
        .start = OSD2_ADDR_START,
        .end   =OSD2_ADDR_END,
        .flags = IORESOURCE_MEM,
    },
};

//#ifdef CONFIG_USB_DWC_OTG_HCD
struct lm_device usb_ld = {
	.irq = INT_USB_B,
	.resource.start = IO_USB_B_BASE,
	.resource.end = -1,
	.dev.dma_mask =  0xffffffff,
	.port_type = USB_PORT_TYPE_HOST,
	.port_speed = USB_PORT_SPEED_DEFAULT,
	.dma_config = USB_DMA_BURST_DEFAULT,
	.set_vbus_power = 0,
};
//#endif

static struct platform_device fb_device = {
    .name       = "apollofb",
    .id         = 0,
    .num_resources = ARRAY_SIZE(fb_device_resources),
    .resource      = fb_device_resources,
};
#endif
#if defined(CONFIG_AM_STREAMING)
static struct resource apollo_codec_resources[] = {
    [0] = {
        .start =  CODEC_ADDR_START,
        .end   = CODEC_ADDR_END,
        .flags = IORESOURCE_MEM,
    },
};

static struct platform_device apollo_codec = {
    .name       = "amstream",
    .id         = 0,
    .num_resources = ARRAY_SIZE(apollo_codec_resources),
    .resource      = apollo_codec_resources,
};
#endif
static struct platform_device __initdata *platform_devs[] = {
    #if defined(CONFIG_JPEGLOGO)
		&jpeglogo_dev,
	#endif	
    #if defined(CONFIG_FB_AM)
    	&fb_device,
    #endif
    #if defined(CONFIG_AM_STREAMING)
	&apollo_codec,
    #endif
};
static char * clock_src_name[]={
		"XTAL input",
		"XTAL input divided by 2",
		"other PLL",
		"DDR PLL",
		"dmod PLL"
};
int set_usb_phy_clk(unsigned long rate)
{

    int divider =0;
    int clk_sel = rate;
    int i;
    int time_dly = 50000;


	// ------------------------------------------------------------
	//  CLK_SEL: These bits select the source for the 12Mhz: 
	// 0 = XTAL input (24, 25, 27Mhz)
	// 1 = XTAL input divided by 2
	// 2 = other PLL output
	// 3 = DDR pll clock (typically 400mhz)
	// 4 = demod 240Mhz PLL output
	CLEAR_CBUS_REG_MASK(PREI_USB_PHY_REG, PREI_USB_PHY_CLK_SEL);
	//clk_sel = 0; // 24M CLK 
	//clk_sel = 1; // 12M, Phy default setting is 12Mhz
	//clk_sel = 2; // other PLL, 540M
	//clk_sel = 3; // DDR, 369M
	//clk_sel = 4; // demod, 240M
	
	printk(KERN_NOTICE"USB PHY clock souce: %s\n",clock_src_name[clk_sel]);
	SET_CBUS_REG_MASK(PREI_USB_PHY_REG, (clk_sel<<5 ));

    if(clk_sel == 3)//DDR runing 396MHz (396/(32+1)=12)
    {
 		CLEAR_CBUS_REG_MASK(PREI_USB_PHY_REG,PREI_USB_PHY_CLK_DIV);
 		SET_CBUS_REG_MASK(PREI_USB_PHY_REG, (32 << 24));
    }else if(clk_sel == 2)//Other PLL running at 540M (540/(44+1)=12)
    {
 		CLEAR_CBUS_REG_MASK(PREI_USB_PHY_REG,PREI_USB_PHY_CLK_DIV);
 		SET_CBUS_REG_MASK(PREI_USB_PHY_REG, (44 << 24));
    }else if(clk_sel == 4)// demod 240M (240/(19+1) = 12)
    {
 		CLEAR_CBUS_REG_MASK(PREI_USB_PHY_REG,PREI_USB_PHY_CLK_DIV);
 		SET_CBUS_REG_MASK(PREI_USB_PHY_REG, (19 << 24));
    }
	// Open clock gate, to enable CLOCK to usb phy 
    SET_CBUS_REG_MASK(PREI_USB_PHY_REG, PREI_USB_PHY_CLK_GATE);
    i=0;
    while(i++<time_dly){};
	
    /*  Reset USB PHY A  */
    SET_CBUS_REG_MASK(PREI_USB_PHY_REG, PREI_USB_PHY_A_AHB_RSET);
    i=0;
    while(i++<time_dly){};  
    CLEAR_CBUS_REG_MASK(PREI_USB_PHY_REG, PREI_USB_PHY_A_AHB_RSET);
    i=0;
    while(i++<time_dly){};
    SET_CBUS_REG_MASK(PREI_USB_PHY_REG, PREI_USB_PHY_A_CLK_RSET);
    i=0;
    while(i++<time_dly){};      
    CLEAR_CBUS_REG_MASK(PREI_USB_PHY_REG, PREI_USB_PHY_A_CLK_RSET);
    i=0;
    while(i++<time_dly){};
    SET_CBUS_REG_MASK(PREI_USB_PHY_REG, PREI_USB_PHY_A_PLL_RSET);
    i=0;
    while(i++<time_dly){};
    CLEAR_CBUS_REG_MASK(PREI_USB_PHY_REG, PREI_USB_PHY_A_PLL_RSET);
    i=0;
    while(i++<time_dly){};

    // ------------------------------------------------------------ 
    // Reset the PHY A by setting POR high for 10uS.
    SET_CBUS_REG_MASK(PREI_USB_PHY_REG, PREI_USB_PHY_A_POR);
    i=0;
    while(i++<time_dly){};
    // Set POR to the PHY high

    CLEAR_CBUS_REG_MASK(PREI_USB_PHY_REG, PREI_USB_PHY_A_POR);
    i=0;
    while(i++<time_dly){};
    
    /* Reset USB PHY B */
    SET_CBUS_REG_MASK(PREI_USB_PHY_REG, PREI_USB_PHY_B_AHB_RSET);
    i=0;
    while(i++<time_dly){};
    CLEAR_CBUS_REG_MASK(PREI_USB_PHY_REG, PREI_USB_PHY_B_AHB_RSET);
    i=0;
    while(i++<time_dly){};
    SET_CBUS_REG_MASK(PREI_USB_PHY_REG, PREI_USB_PHY_B_CLK_RSET);
    i=0;
    while(i++<time_dly){};
    CLEAR_CBUS_REG_MASK(PREI_USB_PHY_REG, PREI_USB_PHY_B_CLK_RSET);
    i=0;
    while(i++<time_dly){};
    SET_CBUS_REG_MASK(PREI_USB_PHY_REG, PREI_USB_PHY_B_PLL_RSET);
    i=0;
    while(i++<time_dly){};
    CLEAR_CBUS_REG_MASK(PREI_USB_PHY_REG, PREI_USB_PHY_B_PLL_RSET);
    i=0;
    while(i++<time_dly){};

    // ------------------------------------------------------------ 
    // Reset the PHY B by setting POR high for 10uS.
    SET_CBUS_REG_MASK(PREI_USB_PHY_REG, PREI_USB_PHY_B_POR);
    i=0;
    while(i++<time_dly){};

    // Set POR to the PHY high
    CLEAR_CBUS_REG_MASK(PREI_USB_PHY_REG, PREI_USB_PHY_B_POR);
    i=0;
    while(i++<time_dly){};

    return 0;
}
static __init void m1_init_machine(void)
{
#ifdef CONFIG_CACHE_L2X0
		/* 128kb (16KB/way), 8-way associativity, evmon/parity/share disabled
		 * Bits:  .... .... .000 0010 0000 .... .... .... */
		l2x0_init((void __iomem *)IO_PL310_BASE, 0x00020000, 0xff800fff);
#endif
	platform_add_devices(platform_devs, ARRAY_SIZE(platform_devs));
	/* todo: load device drivers */
//#ifdef CONFIG_USB_DWC_OTG_HCD
	set_usb_phy_clk(2);
	lm_device_register(&usb_ld);
//#endif
}

static __init void m1_map_io(void)
{
	meson_map_io();
}

static __init void m1_irq_init(void)
{
	meson_init_irq();
}


MACHINE_START(MESON_8626M, "AMLOGIC MESON-M1 8626M")
	.phys_io		= MESON_PERIPHS1_PHYS_BASE,
	.io_pg_offst	= (MESON_PERIPHS1_PHYS_BASE >> 18) & 0xfffc,
	.boot_params	= BOOT_PARAMS_OFFSET,
	.map_io			= m1_map_io,
	.init_irq		= m1_irq_init,
	.timer			= &meson_sys_timer,
	.init_machine	= m1_init_machine,
MACHINE_END
