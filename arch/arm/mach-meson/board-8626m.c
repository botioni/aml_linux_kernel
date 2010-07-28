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
#include <linux/dma-mapping.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/nand_ecc.h>
#include <linux/mtd/partitions.h>
#include <linux/device.h>
#include <linux/spi/flash.h>
#include <mach/hardware.h>
#include <mach/platform.h>
#include <mach/memory.h>
#include <mach/clock.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/setup.h>
#include <mach/lm.h>
#include <asm/memory.h>
#include <asm/mach/map.h>
#include <mach/am_eth_pinmux.h>
#ifdef CONFIG_CACHE_L2X0
#include <asm/hardware/cache-l2x0.h>
#endif
#include <mach/pinmux.h>
#include <mach/gpio.h>
#include <linux/delay.h>

#include "board-8626m.h"

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
    [1] = {
        .start = OSD2_ADDR_START,
        .end   =OSD2_ADDR_END,
        .flags = IORESOURCE_MEM,
    },
};

static struct platform_device fb_device = {
    .name       = "apollofb",
    .id         = 0,
    .num_resources = ARRAY_SIZE(fb_device_resources),
    .resource      = fb_device_resources,
};
#endif
static struct mtd_partition spi_partition_info[] = {
	{
		.name = "U boot",
		.offset = 0,
		.size = 0x80000,
	},
	{
		.name = "conf",
		.offset = 0x80000,
		.size = 0xf0000-0x80000,
	},
};

static struct flash_platform_data amlogic_spi_platform = {
	.parts = spi_partition_info,
	.nr_parts = ARRAY_SIZE(spi_partition_info),
};

static struct resource amlogic_spi_nor_resources[] = {
	{
		.start = 0xc1800000,
		.end = 0xc1ffffff,
		.flags = IORESOURCE_MEM,
	},
};

static struct platform_device amlogic_spi_nor_device = {
	.name = "AMLOGIC_SPI_NOR",
	.id = -1,
	.num_resources = ARRAY_SIZE(amlogic_spi_nor_resources),
	.resource = amlogic_spi_nor_resources,
	.dev = {
		.platform_data = &amlogic_spi_platform,
	},
};
#ifdef CONFIG_USB_DWC_OTG_HCD
struct lm_device usb_ld_b = {
	.type = LM_DEVICE_TYPE_USB,
	.id = 1,
	.irq = INT_USB_B,
	.resource.start = IO_USB_B_BASE,
	.resource.end = -1,
	.dma_mask_room = DMA_BIT_MASK(32),
	.port_type = USB_PORT_TYPE_HOST,
	.port_speed = USB_PORT_SPEED_DEFAULT,
	.dma_config = USB_DMA_BURST_SINGLE,
	.set_vbus_power = 0,
};
#endif
#ifdef CONFIG_SATA_DWC_AHCI
struct lm_device sata_ld = {
	.type = LM_DEVICE_TYPE_SATA,
	.id = 2,
	.irq = INT_SATA,
	.dma_mask_room = DMA_BIT_MASK(32),
	.resource.start = IO_SATA_BASE,
	.resource.end = -1,
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

static struct resource amlogic_card_resource[]  = {
	[0] = {
		.start = 0x1200230,   //physical address
		.end   = 0x120024c,
		.flags = 0x200,
	}
};


struct platform_device amlogic_card_device = { 
	.name = "AMLOGIC_CARD", 
	.id    = -1,
	.num_resources = ARRAY_SIZE(amlogic_card_resource),
	.resource = amlogic_card_resource,
};


static struct platform_device apollo_codec = {
    .name       = "amstream",
    .id         = 0,
    .num_resources = ARRAY_SIZE(apollo_codec_resources),
    .resource      = apollo_codec_resources,
};
#endif

#if defined(CONFIG_AML_AUDIO_DSP)
static struct resource apollo_audiodsp_resources[] = {
    [0] = {
        .start = AUDIODSP_ADDR_START,
        .end   = AUDIODSP_ADDR_END,
        .flags = IORESOURCE_MEM,
    },
};

static struct platform_device apollo_audiodsp = {
    .name       = "audiodsp",
    .id         = 0,
    .num_resources = ARRAY_SIZE(apollo_audiodsp_resources),
    .resource      = apollo_audiodsp_resources,
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
	#if defined(CONFIG_AML_AUDIO_DSP)
		&apollo_audiodsp,
	#endif
    &amlogic_card_device,
    &amlogic_spi_nor_device,
};

static void __init eth_pinmux_init(void)
{
	///GPIOD15-24 for 8626M;
	///GPIOE_16/NA	nRst;
    	eth_clk_set(ETH_CLKSRC_SYS_D3,900*CLK_1M/3,50*CLK_1M);
	/*for dpf_sz with ethernet*/	
	eth_set_pinmux(ETH_BANK2_GPIOD15_D23,ETH_CLK_OUT_GPIOD24_REG5_1,0);
	CLEAR_CBUS_REG_MASK(PREG_ETHERNET_ADDR0, 1);
	SET_CBUS_REG_MASK(PREG_ETHERNET_ADDR0, (1 << 1));
	SET_CBUS_REG_MASK(PREG_ETHERNET_ADDR0, 1);
	udelay(100);
	/*reset*/
	set_gpio_mode(PREG_HGPIO,16,GPIO_OUTPUT_MODE);
	set_gpio_val(PREG_HGPIO,16,0);
	udelay(100);	//waiting reset end;
	set_gpio_val(PREG_HGPIO,16,1);
	udelay(10);	//waiting reset end;
}
static void __init device_pinmux_init(void )
{
	clearall_pinmux();
	/*other deivce power on*/
	/*GPIOA_200e_bit4..usb/eth/YUV power on*/
	set_gpio_mode(PREG_EGPIO,1<<4,GPIO_OUTPUT_MODE);
	set_gpio_val(PREG_EGPIO,1<<4,1);
	/*uart port A,*/
	uart_set_pinmux(UART_PORT_A,UART_A_GPIO_B2_B3);
	/*pinmux of eth*/
	eth_pinmux_init();
}
static __init void m1_init_machine(void)
{
#ifdef CONFIG_CACHE_L2X0
	/* 128kb (16KB/way), 8-way associativity, evmon/parity/share disabled
	 * Bits:  .... .... .000 0010 0000 .... .... .... */
	l2x0_init((void __iomem *)IO_PL310_BASE, 0x00020000, 0xff800fff);
#endif
	device_pinmux_init();
	platform_add_devices(platform_devs, ARRAY_SIZE(platform_devs));
	/* todo: load device drivers */
#ifdef CONFIG_USB_DWC_OTG_HCD
	set_usb_phy_clk(USB_PHY_CLOCK_SEL_XTAL_DIV2);
	lm_device_register(&usb_ld_b);
#endif
#ifdef CONFIG_SATA_DWC_AHCI
	set_sata_phy_clk(SATA_PHY_CLOCK_SEL_DEMOD_PLL);
	lm_device_register(&sata_ld);
#endif
}

/*VIDEO MEMORY MAPING*/
static __initdata struct map_desc meson_video_mem_desc[] = {
	{
		.virtual	= PAGE_ALIGN(__phys_to_virt(RESERVED_MEM_START)),
		.pfn		= __phys_to_pfn(RESERVED_MEM_START),
		.length		= RESERVED_MEM_END-RESERVED_MEM_START+1,
		.type		= MT_DEVICE,
	},
};


static __init void m1_map_io(void)
{
	meson_map_io();
	iotable_init(meson_video_mem_desc, ARRAY_SIZE(meson_video_mem_desc));
}

static __init void m1_irq_init(void)
{
	meson_init_irq();
}


static __init void m1_fixup(struct machine_desc *mach, struct tag *tag, char **cmdline, struct meminfo *m)
{
	struct membank *pbank;
	m->nr_banks = 0;
	pbank=&m->bank[m->nr_banks];
	pbank->start = PAGE_ALIGN(PHYS_MEM_START);
	pbank->size  = SZ_64M & PAGE_MASK;
	pbank->node  = PHYS_TO_NID(PHYS_MEM_START);
	m->nr_banks++;
	pbank=&m->bank[m->nr_banks];
	pbank->start = PAGE_ALIGN(RESERVED_MEM_END+1);
	pbank->size  = (PHYS_MEM_END-RESERVED_MEM_END) & PAGE_MASK;
	pbank->node  = PHYS_TO_NID(RESERVED_MEM_END+1);
	m->nr_banks++;
}

MACHINE_START(MESON_8626M, "AMLOGIC MESON-M1 8626M")
	.phys_io		= MESON_PERIPHS1_PHYS_BASE,
	.io_pg_offst		= (MESON_PERIPHS1_PHYS_BASE >> 18) & 0xfffc,
	.boot_params		= BOOT_PARAMS_OFFSET,
	.map_io			= m1_map_io,
	.init_irq		= m1_irq_init,
	.timer			= &meson_sys_timer,
	.init_machine		= m1_init_machine,
	.video_start		=RESERVED_MEM_START,	/*let the memmap know the memory is reversed, */ 
							/*Because when the board is not support 256M, */
							/*we can used mem=128 for test'*/
	.video_end		=RESERVED_MEM_END,
	.fixup			= m1_fixup,
MACHINE_END
