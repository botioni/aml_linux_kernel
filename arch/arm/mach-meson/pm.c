/*
 * Meson Power Management Routines
 *
 * Copyright (C) 2010 Amlogic, Inc. http://www.amlogic.com/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/pm.h>
#include <linux/suspend.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/spinlock.h>

#include <asm/cacheflush.h>
#include <asm/delay.h>

#include <mach/pm.h>
#include <mach/am_regs.h>
#include <mach/sram.h>
#include <mach/power_gate.h>

#define WAKE_UP_BY_IRQ

static void (*meson_sram_suspend) (struct meson_pm_config *);
static struct meson_pm_config *pdata;

static void meson_sram_push(void *dest, void *src, unsigned int size)
{
    int res = 0;
    memcpy(dest, src, size);
	flush_icache_range((unsigned long)dest, (unsigned long)(dest + size));
	res = memcmp(dest,src,size);
	printk("meson_sram_push (%d)\n", res);
}

#define GATE_OFF(_MOD) do {power_gate_flag[GCLK_IDX_##_MOD] = IS_CLK_GATE_ON(_MOD);CLK_GATE_OFF(_MOD);} while(0)
#define GATE_ON(_MOD) do {if (power_gate_flag[GCLK_IDX_##_MOD]) CLK_GATE_ON(_MOD);} while(0)
#define GATE_SWITCH(flag, _MOD) do {if (flag) GATE_ON(_MOD); else GATE_OFF(_MOD);} while(0)
static int power_gate_flag[GCLK_IDX_MAX];
void power_gate_init(void)
{
    GATE_INIT(AHB_BRIDGE);
    GATE_INIT(AHB_SRAM);
    GATE_INIT(AIU_ADC);
    GATE_INIT(AIU_MIXER_REG);
    GATE_INIT(AIU_AUD_MIXER);
    GATE_INIT(AIU_AIFIFO2);
    GATE_INIT(AIU_AMCLK_MEASURE);
    GATE_INIT(AIU_I2S_OUT);
    GATE_INIT(AIU_IEC958);
    GATE_INIT(AIU_AI_TOP_GLUE);
    GATE_INIT(AIU_AUD_DAC);
    GATE_INIT(AIU_ICE958_AMCLK);
    GATE_INIT(AIU_I2S_DAC_AMCLK);
    GATE_INIT(AIU_I2S_SLOW);
    GATE_INIT(AIU_AUD_DAC_CLK);
    GATE_INIT(ASSIST_MISC);
    GATE_INIT(AMRISC);
    GATE_INIT(AUD_BUF);
    GATE_INIT(AUD_IN);
    GATE_INIT(BLK_MOV);
    GATE_INIT(BT656_IN);
    GATE_INIT(DEMUX);
    GATE_INIT(MMC_DDR);
    GATE_INIT(DDR);
    GATE_INIT(DIG_VID_IN);
    GATE_INIT(ETHERNET);
    GATE_INIT(GE2D);
    GATE_INIT(HDMI_MPEG_DOMAIN);
    GATE_INIT(HIU_PARSER_TOP);
    GATE_INIT(HIU_PARSER);
    GATE_INIT(ISA);
    GATE_INIT(MEDIA_CPU);
    GATE_INIT(MISC_USB0_TO_DDR);
    GATE_INIT(MISC_USB1_TO_DDR);
    GATE_INIT(MISC_SATA_TO_DDR);
    GATE_INIT(AHB_CONTROL_BUS);
    GATE_INIT(AHB_DATA_BUS);
    GATE_INIT(AXI_BUS);
    GATE_INIT(ROM_CLK);
    GATE_INIT(EFUSE);
    GATE_INIT(AHB_ARB0);
    GATE_INIT(RESET);
    GATE_INIT(MDEC_CLK_PIC_DC);
    GATE_INIT(MDEC_CLK_DBLK);
    GATE_INIT(MDEC_CLK_PSC);
    GATE_INIT(MDEC_CLK_ASSIST);
    GATE_INIT(MC_CLK);
    GATE_INIT(IQIDCT_CLK);
    GATE_INIT(VLD_CLK);
    GATE_INIT(NAND);
    GATE_INIT(RESERVED0);
    GATE_INIT(VGHL_PWM);
    GATE_INIT(LED_PWM);
    GATE_INIT(UART1);
    GATE_INIT(SDIO);
    GATE_INIT(ASYNC_FIFO);
    GATE_INIT(STREAM);
    GATE_INIT(RTC);
    GATE_INIT(UART0);
    GATE_INIT(RANDOM_NUM_GEN);
    GATE_INIT(SMART_CARD_MPEG_DOMAIN);
    GATE_INIT(SMART_CARD);
    GATE_INIT(SAR_ADC);
    GATE_INIT(I2C);
    GATE_INIT(IR_REMOTE);
    GATE_INIT(_1200XXX);
    GATE_INIT(SATA);
    GATE_INIT(SPI1);
    GATE_INIT(USB1);
    GATE_INIT(USB0);
    GATE_INIT(VI_CORE);
    GATE_INIT(LCD);
    GATE_INIT(ENC480P_MPEG_DOMAIN);
    GATE_INIT(ENC480I);
    GATE_INIT(VENC_MISC);
    GATE_INIT(ENC480P);
    GATE_INIT(HDMI);
    GATE_INIT(VCLK3_DAC);
    GATE_INIT(VCLK3_MISC);
    GATE_INIT(VCLK3_DVI);
    GATE_INIT(VCLK2_VIU);
    GATE_INIT(VCLK2_VENC_DVI);
    GATE_INIT(VCLK2_VENC_ENC480P);
    GATE_INIT(VCLK2_VENC_BIST);
    GATE_INIT(VCLK1_VENC_656);
    GATE_INIT(VCLK1_VENC_DVI);
    GATE_INIT(VCLK1_VENC_ENCI);
    GATE_INIT(VCLK1_VENC_BIST);
    GATE_INIT(VIDEO_IN);
    GATE_INIT(WIFI);
}

void power_gate_switch(int flag)
{
    //GATE_SWITCH(flag, AHB_BRIDGE);
    //GATE_SWITCH(flag, AHB_SRAM);
    GATE_SWITCH(flag, AIU_ADC);
    GATE_SWITCH(flag, AIU_MIXER_REG);
    GATE_SWITCH(flag, AIU_AUD_MIXER);
    GATE_SWITCH(flag, AIU_AIFIFO2);
    GATE_SWITCH(flag, AIU_AMCLK_MEASURE);
    GATE_SWITCH(flag, AIU_I2S_OUT);
    GATE_SWITCH(flag, AIU_IEC958);
    GATE_SWITCH(flag, AIU_AI_TOP_GLUE);
    GATE_SWITCH(flag, AIU_AUD_DAC);
    GATE_SWITCH(flag, AIU_ICE958_AMCLK);
    GATE_SWITCH(flag, AIU_I2S_DAC_AMCLK);
    GATE_SWITCH(flag, AIU_I2S_SLOW);
    GATE_SWITCH(flag, AIU_AUD_DAC_CLK);
    //GATE_SWITCH(flag, ASSIST_MISC);
    GATE_SWITCH(flag, AMRISC);
    GATE_SWITCH(flag, AUD_BUF);
    GATE_SWITCH(flag, AUD_IN);
    GATE_SWITCH(flag, BLK_MOV);
    GATE_SWITCH(flag, BT656_IN);
    GATE_SWITCH(flag, DEMUX);
    //GATE_SWITCH(flag, MMC_DDR);
    //GATE_SWITCH(flag, DDR);
    GATE_SWITCH(flag, DIG_VID_IN);
    GATE_SWITCH(flag, ETHERNET);
    GATE_SWITCH(flag, GE2D);
    GATE_SWITCH(flag, HDMI_MPEG_DOMAIN);
    //GATE_SWITCH(flag, HIU_PARSER);
    //GATE_SWITCH(flag, HIU_PARSER_TOP);
    //GATE_SWITCH(flag, ISA);
    GATE_SWITCH(flag, MEDIA_CPU);
    GATE_SWITCH(flag, MISC_USB0_TO_DDR);
    GATE_SWITCH(flag, MISC_USB1_TO_DDR);
    GATE_SWITCH(flag, MISC_SATA_TO_DDR);
    //GATE_SWITCH(flag, AHB_CONTROL_BUS);
    //GATE_SWITCH(flag, AHB_DATA_BUS);
    //GATE_SWITCH(flag, AXI_BUS);
    GATE_SWITCH(flag, ROM_CLK);
    GATE_SWITCH(flag, EFUSE);
    //GATE_SWITCH(flag, AHB_ARB0);
    //GATE_SWITCH(flag, RESET);
    GATE_SWITCH(flag, MDEC_CLK_PIC_DC);
    GATE_SWITCH(flag, MDEC_CLK_DBLK);
    GATE_SWITCH(flag, MDEC_CLK_PSC);
    GATE_SWITCH(flag, MDEC_CLK_ASSIST);
    GATE_SWITCH(flag, MC_CLK);
    GATE_SWITCH(flag, IQIDCT_CLK);
    GATE_SWITCH(flag, VLD_CLK);

    GATE_SWITCH(flag, NAND);
    GATE_SWITCH(flag, RESERVED0);
    GATE_SWITCH(flag, VGHL_PWM);
    GATE_SWITCH(flag, LED_PWM);
    //GATE_SWITCH(flag, UART1);
    GATE_SWITCH(flag, SDIO);
    GATE_SWITCH(flag, ASYNC_FIFO);
    //GATE_SWITCH(flag, STREAM);
    //GATE_SWITCH(flag, RTC);
    //GATE_SWITCH(flag, UART0);
    GATE_SWITCH(flag, RANDOM_NUM_GEN);
    GATE_SWITCH(flag, SMART_CARD_MPEG_DOMAIN);
    GATE_SWITCH(flag, SMART_CARD);
    GATE_SWITCH(flag, SAR_ADC);
    GATE_SWITCH(flag, I2C);
    GATE_SWITCH(flag, IR_REMOTE);
//    GATE_SWITCH(flag, _1200XXX);
    GATE_SWITCH(flag, SATA);
    GATE_SWITCH(flag, SPI1);
    GATE_SWITCH(flag, USB1);
    GATE_SWITCH(flag, USB0);
    GATE_SWITCH(flag, VI_CORE);
    GATE_SWITCH(flag, LCD);
    GATE_SWITCH(flag, ENC480P_MPEG_DOMAIN);
    GATE_SWITCH(flag, ENC480I);
    GATE_SWITCH(flag, VENC_MISC);
    GATE_SWITCH(flag, ENC480P);
    GATE_SWITCH(flag, HDMI);
    GATE_SWITCH(flag, VCLK3_DAC);
    GATE_SWITCH(flag, VCLK3_MISC);
    GATE_SWITCH(flag, VCLK3_DVI);
    GATE_SWITCH(flag, VCLK2_VIU);
    GATE_SWITCH(flag, VCLK2_VENC_DVI);
    GATE_SWITCH(flag, VCLK2_VENC_ENC480P);
    GATE_SWITCH(flag, VCLK2_VENC_BIST);
    GATE_SWITCH(flag, VCLK1_VENC_656);
    GATE_SWITCH(flag, VCLK1_VENC_DVI);
    GATE_SWITCH(flag, VCLK1_VENC_ENCI);
    GATE_SWITCH(flag, VCLK1_VENC_BIST);
    GATE_SWITCH(flag, VIDEO_IN);
    GATE_SWITCH(flag, WIFI);
}

static void meson_pm_suspend(void)
{
	int mask_save[4];
    
    printk("power gate stat before suspend %x %x %x %x\n", 
           READ_CBUS_REG(HHI_GCLK_MPEG0), 
           READ_CBUS_REG(HHI_GCLK_MPEG1),
           READ_CBUS_REG(HHI_GCLK_MPEG2),
           READ_CBUS_REG(HHI_GCLK_OTHER));    
	printk(KERN_INFO "enter meson_pm_suspend!\n");

#ifdef WAKE_UP_BY_IRQ	
	mask_save[0] = READ_CBUS_REG(A9_0_IRQ_IN0_INTR_MASK);
	mask_save[1] = READ_CBUS_REG(A9_0_IRQ_IN1_INTR_MASK);
	mask_save[2] = READ_CBUS_REG(A9_0_IRQ_IN2_INTR_MASK);
	mask_save[3] = READ_CBUS_REG(A9_0_IRQ_IN3_INTR_MASK);
	WRITE_CBUS_REG(A9_0_IRQ_IN0_INTR_MASK, 0x0);
	WRITE_CBUS_REG(A9_0_IRQ_IN1_INTR_MASK, 0x0);
	WRITE_CBUS_REG(A9_0_IRQ_IN2_INTR_MASK, (1<<8));
	WRITE_CBUS_REG(A9_0_IRQ_IN3_INTR_MASK, 0x0);
    
    audio_internal_dac_disable();
    video_dac_disable();

    power_gate_switch(0);
    printk("power gate stat in suspend %x %x %x %x\n", 
           READ_CBUS_REG(HHI_GCLK_MPEG0), 
           READ_CBUS_REG(HHI_GCLK_MPEG1),
           READ_CBUS_REG(HHI_GCLK_MPEG2),
           READ_CBUS_REG(HHI_GCLK_OTHER));    
    
	WRITE_MPEG_REG(HHI_A9_CLK_CNTL, READ_MPEG_REG(HHI_A9_CLK_CNTL)&~(1<<7));
	meson_sram_suspend(pdata);
	WRITE_MPEG_REG(HHI_A9_CLK_CNTL, READ_MPEG_REG(HHI_A9_CLK_CNTL)|(1<<7));
	
    printk("intr stat %x %x %x %x\n", 
		READ_CBUS_REG(A9_0_IRQ_IN0_INTR_STAT), 
		READ_CBUS_REG(A9_0_IRQ_IN1_INTR_STAT),
		READ_CBUS_REG(A9_0_IRQ_IN2_INTR_STAT),
		READ_CBUS_REG(A9_0_IRQ_IN3_INTR_STAT));

    power_gate_switch(1);

    WRITE_CBUS_REG(A9_0_IRQ_IN0_INTR_MASK, mask_save[0]);
	WRITE_CBUS_REG(A9_0_IRQ_IN1_INTR_MASK, mask_save[1]);
	WRITE_CBUS_REG(A9_0_IRQ_IN2_INTR_MASK, mask_save[2]);
	WRITE_CBUS_REG(A9_0_IRQ_IN3_INTR_MASK, mask_save[3]);
    printk("power gate stat after suspend %x %x %x %x\n", 
           READ_CBUS_REG(HHI_GCLK_MPEG0), 
           READ_CBUS_REG(HHI_GCLK_MPEG1),
           READ_CBUS_REG(HHI_GCLK_MPEG2),
           READ_CBUS_REG(HHI_GCLK_OTHER));
#else
	int powerPress = 0;
	while(1){
		udelay(jiffies+msecs_to_jiffies(20));
		powerPress = ((READ_CBUS_REG(0x21d1/*RTC_ADDR1*/) >> 2) & 1) ? 0 : 1;
		if(powerPress)
			break;
	}
#endif
}

static int meson_pm_prepare(void)
{
	printk(KERN_INFO "enter meson_pm_prepare!\n");
	meson_sram_push(meson_sram_suspend, meson_cpu_suspend,
						meson_cpu_suspend_sz);
	return 0;
}

static int meson_pm_enter(suspend_state_t state)
{
	int ret = 0;

	switch (state) {
	case PM_SUSPEND_STANDBY:
	case PM_SUSPEND_MEM:
		meson_pm_suspend();
		break;
	default:
		ret = -EINVAL;
	}

	return ret;
}

static struct platform_suspend_ops meson_pm_ops = {
	.enter		= meson_pm_enter,
	.prepare    = meson_pm_prepare,
	.valid		= suspend_valid_only_mem,
};

static int __init meson_pm_probe(struct platform_device *pdev)
{
	printk(KERN_INFO "enter meson_pm_probe!\n");

    power_gate_init();

    pdata = pdev->dev.platform_data;
	if (!pdata) {
		dev_err(&pdev->dev, "cannot get platform data\n");
		return -ENOENT;
	}

	meson_sram_suspend = sram_alloc(meson_cpu_suspend_sz);
	if (!meson_sram_suspend) {
		dev_err(&pdev->dev, "cannot allocate SRAM memory\n");
		return -ENOMEM;
	}

	meson_sram_push(meson_sram_suspend, meson_cpu_suspend,
						meson_cpu_suspend_sz);

	suspend_set_ops(&meson_pm_ops);
	printk(KERN_INFO "meson_pm_probe done 0x%x %d!\n", (unsigned)meson_sram_suspend, meson_cpu_suspend_sz);
	return 0;
}

static int __exit meson_pm_remove(struct platform_device *pdev)
{
	return 0;
}

static struct platform_driver meson_pm_driver = {
	.driver = {
		.name	 = "pm-meson",
		.owner	 = THIS_MODULE,
	},
	.remove = __exit_p(meson_pm_remove),
};

static int __init meson_pm_init(void)
{
	return platform_driver_probe(&meson_pm_driver, meson_pm_probe);
}
late_initcall(meson_pm_init);
