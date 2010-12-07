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
#include <linux/clk.h>

#include <asm/cacheflush.h>
#include <asm/delay.h>

#include <mach/pm.h>
#include <mach/am_regs.h>
#include <mach/sram.h>
#include <mach/power_gate.h>
#include <mach/gpio.h>

#define ON  1
#define OFF 0

#define WAKE_UP_BY_IRQ
#define SYSCLK_32K

static void (*meson_sram_suspend) (struct meson_pm_config *);
static struct meson_pm_config *pdata;
static int mask_save[4];

static void meson_sram_push(void *dest, void *src, unsigned int size)
{
    int res = 0;
    memcpy(dest, src, size);
    flush_icache_range((unsigned long)dest, (unsigned long)(dest + size));
    res = memcmp(dest,src,size);
    printk("compare code in sram result = %d", res);
}

#define GATE_OFF(_MOD) do {power_gate_flag[GCLK_IDX_##_MOD] = IS_CLK_GATE_ON(_MOD);CLK_GATE_OFF(_MOD);} while(0)
#define GATE_ON(_MOD) do {if (power_gate_flag[GCLK_IDX_##_MOD]) CLK_GATE_ON(_MOD);} while(0)
#define GATE_SWITCH(flag, _MOD) do {if (flag) GATE_ON(_MOD); else GATE_OFF(_MOD);} while(0)
static int power_gate_flag[GCLK_IDX_MAX];

#define EARLY_GATE_OFF(_MOD) do {early_power_gate_flag[GCLK_IDX_##_MOD] = IS_CLK_GATE_ON(_MOD);CLK_GATE_OFF(_MOD);} while(0)
#define EARLY_GATE_ON(_MOD) do {if (early_power_gate_flag[GCLK_IDX_##_MOD]) CLK_GATE_ON(_MOD);} while(0)
#define EARLY_GATE_SWITCH(flag, _MOD) do {if (flag) EARLY_GATE_ON(_MOD); else EARLY_GATE_OFF(_MOD);} while(0)
static int early_power_gate_flag[GCLK_IDX_MAX];

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

void power_init_off(void)
{
    GATE_OFF(BT656_IN);
    GATE_OFF(DIG_VID_IN);
    GATE_OFF(VIDEO_IN);
    GATE_OFF(GE2D);
    GATE_OFF(DEMUX);
    GATE_OFF(ETHERNET);
    GATE_OFF(WIFI);
    CLEAR_CBUS_REG_MASK(HHI_DEMOD_CLK_CNTL, (1<<8));
    CLEAR_CBUS_REG_MASK(HHI_SATA_CLK_CNTL, (1<<8));
    CLEAR_CBUS_REG_MASK(HHI_ETH_CLK_CNTL, (1<<8));
    CLEAR_CBUS_REG_MASK(HHI_WIFI_CLK_CNTL, (1<<8));
    SET_CBUS_REG_MASK(HHI_WIFI_PLL_CNTL, (1<<15));
    SET_CBUS_REG_MASK(HHI_DEMOD_PLL_CNTL, (1<<15));
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
    //GATE_SWITCH(flag, _1200XXX);
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
EXPORT_SYMBOL(power_gate_switch);

void early_power_gate_switch(int flag)
{
    EARLY_GATE_SWITCH(flag, AMRISC);
    EARLY_GATE_SWITCH(flag, AUD_IN);
    EARLY_GATE_SWITCH(flag, BLK_MOV);
    EARLY_GATE_SWITCH(flag, BT656_IN);
    EARLY_GATE_SWITCH(flag, DIG_VID_IN);
    EARLY_GATE_SWITCH(flag, GE2D);
    EARLY_GATE_SWITCH(flag, ROM_CLK);
    EARLY_GATE_SWITCH(flag, EFUSE);
    EARLY_GATE_SWITCH(flag, MDEC_CLK_PIC_DC);
    EARLY_GATE_SWITCH(flag, MDEC_CLK_DBLK);
    EARLY_GATE_SWITCH(flag, MDEC_CLK_PSC);
    EARLY_GATE_SWITCH(flag, MDEC_CLK_ASSIST);
    EARLY_GATE_SWITCH(flag, MC_CLK);
    EARLY_GATE_SWITCH(flag, IQIDCT_CLK);
    EARLY_GATE_SWITCH(flag, VLD_CLK);
    EARLY_GATE_SWITCH(flag, RESERVED0);
    EARLY_GATE_SWITCH(flag, VGHL_PWM);
    EARLY_GATE_SWITCH(flag, LED_PWM);
    EARLY_GATE_SWITCH(flag, VI_CORE);
    EARLY_GATE_SWITCH(flag, LCD);
    EARLY_GATE_SWITCH(flag, ENC480P_MPEG_DOMAIN);
    EARLY_GATE_SWITCH(flag, ENC480I);
    EARLY_GATE_SWITCH(flag, VENC_MISC);
    EARLY_GATE_SWITCH(flag, ENC480P);
    EARLY_GATE_SWITCH(flag, HDMI);
    EARLY_GATE_SWITCH(flag, VCLK3_DAC);
    EARLY_GATE_SWITCH(flag, VCLK3_MISC);
    EARLY_GATE_SWITCH(flag, VCLK3_DVI);
    EARLY_GATE_SWITCH(flag, VCLK2_VIU);
    EARLY_GATE_SWITCH(flag, VCLK2_VENC_DVI);
    EARLY_GATE_SWITCH(flag, VCLK2_VENC_ENC480P);
    EARLY_GATE_SWITCH(flag, VCLK2_VENC_BIST);
    EARLY_GATE_SWITCH(flag, VCLK1_VENC_656);
    EARLY_GATE_SWITCH(flag, VCLK1_VENC_DVI);
    EARLY_GATE_SWITCH(flag, VCLK1_VENC_ENCI);
    EARLY_GATE_SWITCH(flag, VCLK1_VENC_BIST);
    EARLY_GATE_SWITCH(flag, VIDEO_IN);
}
EXPORT_SYMBOL(early_power_gate_switch);

#define CLK_COUNT 9
static char clk_flag[CLK_COUNT];
static unsigned clks[CLK_COUNT]={
    HHI_VID_CLK_CNTL,
    HHI_AUD_CLK_CNTL,
    HHI_MALI_CLK_CNTL,
    HHI_HDMI_CLK_CNTL,
    HHI_DEMOD_CLK_CNTL,
    HHI_SATA_CLK_CNTL,
    HHI_ETH_CLK_CNTL,
    HHI_WIFI_CLK_CNTL,
    HHI_MPEG_CLK_CNTL
};

static char clks_name[CLK_COUNT][32]={
    "HHI_VID_CLK_CNTL",
    "HHI_AUD_CLK_CNTL",
    "HHI_MALI_CLK_CNTL",
    "HHI_HDMI_CLK_CNTL",
    "HHI_DEMOD_CLK_CNTL",
    "HHI_SATA_CLK_CNTL",
    "HHI_ETH_CLK_CNTL",
    "HHI_WIFI_CLK_CNTL",
    "HHI_MPEG_CLK_CNTL"
};

static unsigned clk81_backup;
void clk_switch(int flag)
{
    int i;
#ifndef SYSCLK_32K
    struct clk *sys_clk;
#endif
    if (flag){
        for (i=0;i<CLK_COUNT;i++){
            if (clk_flag[i]){
                if (clks[i] == HHI_VID_CLK_CNTL){
                    SET_CBUS_REG_MASK(clks[i], 1);
                }
                else if (clks[i] == HHI_MPEG_CLK_CNTL){
#ifdef SYSCLK_32K
                    SET_CBUS_REG_MASK(clks[i], clk81_backup);
#else
                    sys_clk = clk_get_sys("clk81", NULL);
                    SET_CBUS_REG_MASK(clks[i], (1<<8));
                    CLEAR_CBUS_REG_MASK(UART0_CONTROL, (1 << 19) | 0xFFF);
                    SET_CBUS_REG_MASK(UART0_CONTROL, (((sys_clk->rate / (115200 * 4)) - 1) & 0xfff));
                    CLEAR_CBUS_REG_MASK(UART1_CONTROL, (1 << 19) | 0xFFF);
                    SET_CBUS_REG_MASK(UART1_CONTROL, (((sys_clk->rate / (115200 * 4)) - 1) & 0xfff));
#endif
                }
                else{
                    SET_CBUS_REG_MASK(clks[i], (1<<8));
                }
                printk(KERN_INFO "clk %s(%x) on\n", clks_name[i], clks[i]);
            }
        }
    }
    else{
        for (i=0;i<CLK_COUNT;i++){
            if (clks[i] == HHI_VID_CLK_CNTL){
                clk_flag[i] = READ_CBUS_REG_BITS(clks[i], 0, 1);
                if (clk_flag[i]){
                    CLEAR_CBUS_REG_MASK(clks[i], 1);
                }
            }
            else if (clks[i] == HHI_MPEG_CLK_CNTL){
                clk_flag[i] = READ_CBUS_REG_BITS(clks[i], 8, 1) ? 1 : 0;
                if (clk_flag[i]){
#ifdef SYSCLK_32K
                    clk81_backup = READ_CBUS_REG(clks[i]);
                    CLEAR_CBUS_REG_MASK(clks[i], (1<<8));
                    WRITE_CBUS_REG_BITS(clks[i], 0, 0, 6);
                    WRITE_CBUS_REG_BITS(clks[i], 0, 12, 2);
                    SET_CBUS_REG_MASK(clks[i], (1<<8));
#else
                    sys_clk = clk_get_sys("clk_xtal", NULL);
                    CLEAR_CBUS_REG_MASK(clks[i], (1<<8));
                    CLEAR_CBUS_REG_MASK(UART0_CONTROL, (1 << 19) | 0xFFF);
                    SET_CBUS_REG_MASK(UART0_CONTROL, (((sys_clk->rate / (115200 * 4)) - 1) & 0xfff));
                    CLEAR_CBUS_REG_MASK(UART1_CONTROL, (1 << 19) | 0xFFF);
                    SET_CBUS_REG_MASK(UART1_CONTROL, (((sys_clk->rate / (115200 * 4)) - 1) & 0xfff));
#endif
                }
            }
            else{
                clk_flag[i] = READ_CBUS_REG_BITS(clks[i], 8, 1) ? 1 : 0;
                if (clk_flag[i])    
                    CLEAR_CBUS_REG_MASK(clks[i], (1<<8));
            }
            if (clk_flag[i])
                printk(KERN_INFO "clk %s(%x) off\n", clks_name[i], clks[i]);
        }
    }
}
EXPORT_SYMBOL(clk_switch);

#define EARLY_CLK_COUNT 6
static char early_clk_flag[EARLY_CLK_COUNT];
static unsigned early_clks[EARLY_CLK_COUNT]={
    HHI_VID_CLK_CNTL,
    HHI_DEMOD_CLK_CNTL,
    HHI_SATA_CLK_CNTL,
    HHI_ETH_CLK_CNTL,
    HHI_WIFI_CLK_CNTL,
    HHI_MPEG_CLK_CNTL,
};

static char early_clks_name[EARLY_CLK_COUNT][32]={
    "HHI_VID_CLK_CNTL",
    "HHI_DEMOD_CLK_CNTL",
    "HHI_SATA_CLK_CNTL",
    "HHI_ETH_CLK_CNTL",
    "HHI_WIFI_CLK_CNTL",
    "HHI_MPEG_CLK_CNTL",
};

void early_clk_switch(int flag)
{
    int i;
    struct clk *sys_clk;

    if (flag){
        for (i=0;i<EARLY_CLK_COUNT;i++){
            if (early_clk_flag[i]){
                if (early_clks[i] == HHI_VID_CLK_CNTL){
                    SET_CBUS_REG_MASK(clks[i], 1);
                }
                else if (early_clks[i] == HHI_MPEG_CLK_CNTL){
                    sys_clk = clk_get_sys("clk81", NULL);
                    SET_CBUS_REG_MASK(early_clks[i], (1<<8));
                    CLEAR_CBUS_REG_MASK(UART0_CONTROL, (1 << 19) | 0xFFF);
                    SET_CBUS_REG_MASK(UART0_CONTROL, (((sys_clk->rate / (115200 * 4)) - 1) & 0xfff));
                    CLEAR_CBUS_REG_MASK(UART1_CONTROL, (1 << 19) | 0xFFF);
                    SET_CBUS_REG_MASK(UART1_CONTROL, (((sys_clk->rate / (115200 * 4)) - 1) & 0xfff));
                }
                else{
                    SET_CBUS_REG_MASK(early_clks[i], (1<<8));
                }
                printk(KERN_INFO "late clk %s(%x) on\n", early_clks_name[i], early_clks[i]);
            }
        }
    }
    else{
        for (i=0;i<EARLY_CLK_COUNT;i++){
            if (early_clks[i] == HHI_VID_CLK_CNTL){
                early_clk_flag[i] = READ_CBUS_REG_BITS(early_clks[i], 0, 1);
                if (early_clk_flag[i]){
                    CLEAR_CBUS_REG_MASK(early_clks[i], 1);
                }
            }
            else{
                early_clk_flag[i] = READ_CBUS_REG_BITS(early_clks[i], 8, 1) ? 1 : 0;
                if (early_clk_flag[i]){
                    CLEAR_CBUS_REG_MASK(early_clks[i], (1<<8));
                    if (early_clks[i] == HHI_MPEG_CLK_CNTL){
                        sys_clk = clk_get_sys("clk_xtal", NULL);
                        CLEAR_CBUS_REG_MASK(UART0_CONTROL, (1 << 19) | 0xFFF);
                        SET_CBUS_REG_MASK(UART0_CONTROL, (((sys_clk->rate / (115200 * 4)) - 1) & 0xfff));
                        CLEAR_CBUS_REG_MASK(UART1_CONTROL, (1 << 19) | 0xFFF);
                        SET_CBUS_REG_MASK(UART1_CONTROL, (((sys_clk->rate / (115200 * 4)) - 1) & 0xfff));                        
                    }
                }
            }
            if (early_clk_flag[i])
                printk(KERN_INFO "early clk %s(%x) off\n", early_clks_name[i], early_clks[i]);
        }
    }
}
EXPORT_SYMBOL(early_clk_switch);

#define PLL_COUNT 6
static char pll_flag[PLL_COUNT];
static unsigned plls[PLL_COUNT]={
    HHI_SYS_PLL_CNTL,
    HHI_OTHER_PLL_CNTL,
    HHI_VID_PLL_CNTL,
    HHI_AUD_PLL_CNTL,
    HHI_WIFI_PLL_CNTL,
    HHI_DEMOD_PLL_CNTL,
};

static char plls_name[PLL_COUNT][32]={
    "HHI_SYS_PLL_CNTL",
    "HHI_OTHER_PLL_CNTL",
    "HHI_VID_PLL_CNTL",
    "HHI_AUD_PLL_CNTL",
    "HHI_WIFI_PLL_CNTL",
    "HHI_DEMOD_PLL_CNTL",
};

void pll_switch(int flag)
{
    int i;
    if (flag){
        for (i=0;i<PLL_COUNT;i++){
            if (pll_flag[i]) {
                CLEAR_CBUS_REG_MASK(plls[i], (1<<15));
                printk(KERN_INFO "pll %s(%x) on\n", plls_name[i], plls[i]);
            }
        }
        udelay(1000);
    }
    else{
        for (i=0;i<PLL_COUNT;i++){
            pll_flag[i] = READ_CBUS_REG_BITS(plls[i], 15, 1) ? 0 : 1;
            if (pll_flag[i]){
                printk(KERN_INFO "pll %s(%x) off\n", plls_name[i], plls[i]);
                SET_CBUS_REG_MASK(plls[i], (1<<15));
            }
        }
    }
}
EXPORT_SYMBOL(pll_switch);

#define EARLY_PLL_COUNT 2
static char early_pll_flag[EARLY_PLL_COUNT];
static unsigned early_plls[EARLY_PLL_COUNT]={
//    HHI_OTHER_PLL_CNTL,
//    HHI_VID_PLL_CNTL,
    HHI_WIFI_PLL_CNTL,
    HHI_DEMOD_PLL_CNTL,
};

static char early_plls_name[EARLY_PLL_COUNT][32]={
//    "HHI_OTHER_PLL_CNTL",
//    "HHI_VID_PLL_CNTL",
    "HHI_WIFI_PLL_CNTL",
    "HHI_DEMOD_PLL_CNTL",
};

void early_pll_switch(int flag)
{
    int i;
    if (flag){
        for (i=0;i<EARLY_PLL_COUNT;i++){
            if (early_pll_flag[i]) {
                CLEAR_CBUS_REG_MASK(plls[i], (1<<15));
                printk(KERN_INFO "late pll %s(%x) on\n", early_plls_name[i], early_plls[i]);
            }
        }
        udelay(1000);
    }
    else{
        for (i=0;i<EARLY_PLL_COUNT;i++){
            early_pll_flag[i] = READ_CBUS_REG_BITS(plls[i], 15, 1) ? 0 : 1;
            if (early_pll_flag[i]){
                printk(KERN_INFO "early pll %s(%x) off\n", early_plls_name[i], early_plls[i]);
                SET_CBUS_REG_MASK(early_plls[i], (1<<15));
            }
        }
    }
}
EXPORT_SYMBOL(early_pll_switch);

void analog_switch(int flag)
{
    if (flag){
        printk(KERN_INFO "analog on\n");
        CLEAR_CBUS_REG_MASK(SAR_ADC_REG3, 1<<28);
        SET_CBUS_REG_MASK(AM_ANALOG_TOP_REG0, 1<<1);
    }
    else{
        printk(KERN_INFO "analog off\n");
        SET_CBUS_REG_MASK(SAR_ADC_REG3, 1<<28);         // set 0x21a3 bit[28] 1 to power down
        CLEAR_CBUS_REG_MASK(AM_ANALOG_TOP_REG0, 1<<1);  // set 0x206e bit[1] 0 to shutdown
    }
}

void usb_switch(int flag,int ctrl)
{
    int msk = PREI_USB_PHY_A_POR;
	
    if(ctrl == 1)
        msk = PREI_USB_PHY_B_POR;

    if (flag){
        printk(KERN_INFO "usb %d on\n",ctrl);
        CLEAR_CBUS_REG_MASK(PREI_USB_PHY_REG, msk);
    }
    else{
        printk(KERN_INFO "usb %d off\n",ctrl);
        SET_CBUS_REG_MASK(PREI_USB_PHY_REG, msk);
    }
}

static void meson_pm_suspend(void)
{
    int divider;
    int divider_sel;
    unsigned ddr_clk_N;

    printk(KERN_INFO "enter meson_pm_suspend!\n");
    
    pdata->ddr_clk = READ_CBUS_REG(HHI_DDR_PLL_CNTL);
    ddr_clk_N = (pdata->ddr_clk>>9)&0x1f;
    ddr_clk_N = ddr_clk_N*4; // N*4
    if (ddr_clk_N>0x1f)
        ddr_clk_N=0x1f;
    pdata->ddr_clk &= ~(0x1f<<9);
    pdata->ddr_clk |= ddr_clk_N<<9;
    printk(KERN_INFO "target ddr clock 0x%x!\n", pdata->ddr_clk);
    divider = READ_CBUS_REG_BITS(HHI_A9_CLK_CNTL, 8, 6);
    divider_sel = READ_CBUS_REG_BITS(HHI_A9_CLK_CNTL, 2, 2);
    WRITE_CBUS_REG(HHI_A9_CLK_CNTL, READ_CBUS_REG(HHI_A9_CLK_CNTL)&~(1<<7));
#ifdef SYSCLK_32K
    WRITE_CBUS_REG(HHI_A9_CLK_CNTL, READ_CBUS_REG(HHI_A9_CLK_CNTL)|(1<<9));
#endif
    analog_switch(OFF);

    usb_switch(OFF,0);
    usb_switch(OFF,1);
    
    if(pdata->set_vccx2){
        pdata->set_vccx2(OFF);
    }

    power_gate_switch(OFF);
    
    clk_switch(OFF);
    
    pll_switch(OFF);
    
    meson_sram_push(meson_sram_suspend, meson_cpu_suspend,
                        meson_cpu_suspend_sz);
     
#ifdef WAKE_UP_BY_IRQ 
    WRITE_CBUS_REG(A9_0_IRQ_IN2_INTR_MASK, (1<<8));
    meson_sram_suspend(pdata);
#else
    int powerPress = 0;
    printk(KERN_INFO "sleep ...\n");
    while(1){
        powerPress = ((READ_CBUS_REG(0x21d1/*RTC_ADDR1*/) >> 2) & 1) ? 0 : 1;
        if(powerPress){
            printk(KERN_INFO "intr stat %x %x %x %x\n", 
                   READ_CBUS_REG(A9_0_IRQ_IN0_INTR_STAT), 
                   READ_CBUS_REG(A9_0_IRQ_IN1_INTR_STAT),
                   READ_CBUS_REG(A9_0_IRQ_IN2_INTR_STAT),
                   READ_CBUS_REG(A9_0_IRQ_IN3_INTR_STAT));            
            break;
        }
    }
    printk(KERN_INFO "... wake up\n");
#endif

    if(pdata->set_vccx2){
        pdata->set_vccx2(ON);
    }
    
    pll_switch(ON);
    
    clk_switch(ON);
        
    power_gate_switch(ON);

    usb_switch(ON,0);
    usb_switch(ON,1);

    analog_switch(ON);
    
#ifdef SYSCLK_32K    
    WRITE_CBUS_REG(HHI_A9_CLK_CNTL, READ_CBUS_REG(HHI_A9_CLK_CNTL)&~(1<<9));
#endif
    WRITE_CBUS_REG(HHI_A9_CLK_CNTL, READ_CBUS_REG(HHI_A9_CLK_CNTL)|(1<<7));
}

static int meson_pm_prepare(void)
{
    printk(KERN_INFO "enter meson_pm_prepare!\n");
    mask_save[0] = READ_CBUS_REG(A9_0_IRQ_IN0_INTR_MASK);
    mask_save[1] = READ_CBUS_REG(A9_0_IRQ_IN1_INTR_MASK);
    mask_save[2] = READ_CBUS_REG(A9_0_IRQ_IN2_INTR_MASK);
    mask_save[3] = READ_CBUS_REG(A9_0_IRQ_IN3_INTR_MASK);
    WRITE_CBUS_REG(A9_0_IRQ_IN0_INTR_MASK, 0x0);
    WRITE_CBUS_REG(A9_0_IRQ_IN1_INTR_MASK, 0x0);
    WRITE_CBUS_REG(A9_0_IRQ_IN2_INTR_MASK, 0x0);
    WRITE_CBUS_REG(A9_0_IRQ_IN3_INTR_MASK, 0x0);
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

static void meson_pm_finish(void)
{
    printk(KERN_INFO "enter meson_pm_finish!\n");
    WRITE_CBUS_REG(A9_0_IRQ_IN0_INTR_MASK, mask_save[0]);
    WRITE_CBUS_REG(A9_0_IRQ_IN1_INTR_MASK, mask_save[1]);
    WRITE_CBUS_REG(A9_0_IRQ_IN2_INTR_MASK, mask_save[2]);
    WRITE_CBUS_REG(A9_0_IRQ_IN3_INTR_MASK, mask_save[3]);
}

static struct platform_suspend_ops meson_pm_ops = {
    .enter        = meson_pm_enter,
    .prepare      = meson_pm_prepare,
    .finish       = meson_pm_finish,
    .valid        = suspend_valid_only_mem,
};

static int __init meson_pm_probe(struct platform_device *pdev)
{
    printk(KERN_INFO "enter meson_pm_probe!\n");

    power_init_off();
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
        .name     = "pm-meson",
        .owner     = THIS_MODULE,
    },
    .remove = __exit_p(meson_pm_remove),
};

static int __init meson_pm_init(void)
{
    return platform_driver_probe(&meson_pm_driver, meson_pm_probe);
}
late_initcall(meson_pm_init);
