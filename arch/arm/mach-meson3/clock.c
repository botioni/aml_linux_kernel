/*
 *
 * arch/arm/mach-meson/clock.c
 *
 *  Copyright (C) 2010 AMLOGIC, INC.
 *
 * License terms: GNU General Public License (GPL) version 2
 * Define clocks in the app platform.
 *
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/clk.h>
#include <linux/init.h>
#include <linux/spinlock.h>
#include <linux/delay.h>

#include <asm/clkdev.h>
#include <mach/clock.h>
#include <mach/hardware.h>
#include <mach/clk_set.h>
#include <mach/am_regs.h>
#include <mach/power_gate.h>

static DEFINE_SPINLOCK(clockfw_lock);

#ifdef CONFIG_INIT_A9_CLOCK_FREQ
static unsigned long __initdata init_clock = CONFIG_INIT_A9_CLOCK;
#else
static unsigned long __initdata init_clock = 0;
#endif

// -----------------------------------------
// clk_util_clk_msr
// -----------------------------------------
// from twister_core.v
//        .clk_to_msr_in          ( { 18'h0,                      // [63:46]
//                                    cts_pwm_A_clk,              // [45]
//                                    cts_pwm_B_clk,              // [44]
//                                    cts_pwm_C_clk,              // [43]
//                                    cts_pwm_D_clk,              // [42]
//                                    cts_eth_rx_tx,              // [41]
//                                    cts_pcm_mclk,               // [40]
//                                    cts_pcm_sclk,               // [39]
//                                    cts_vdin_meas_clk,          // [38]
//                                    cts_vdac_clk[1],            // [37]
//                                    cts_hdmi_tx_pixel_clk,      // [36]
//                                    cts_mali_clk,               // [35]
//                                    cts_sdhc_clk1,              // [34]
//                                    cts_sdhc_clk0,              // [33]
//                                    cts_audac_clkpi,            // [32]
//                                    cts_a9_clk,                 // [31]
//                                    cts_ddr_clk,                // [30]
//                                    cts_vdac_clk[0],            // [29]
//                                    cts_sar_adc_clk,            // [28]
//                                    cts_enci_clk,               // [27]
//                                    sc_clk_int,                 // [26]
//                                    usb_clk_12mhz,              // [25]
//                                    lvds_fifo_clk,              // [24]
//                                    HDMI_CH3_TMDSCLK,           // [23]
//                                    mod_eth_clk50_i,            // [22]
//                                    mod_audin_amclk_i,          // [21]
//                                    cts_btclk27,                // [20]
//                                    cts_hdmi_sys_clk,           // [19]
//                                    cts_led_pll_clk,            // [18]
//                                    cts_vghl_pll_clk,           // [17]
//                                    cts_FEC_CLK_2,              // [16]
//                                    cts_FEC_CLK_1,              // [15]
//                                    cts_FEC_CLK_0,              // [14]
//                                    cts_amclk,                  // [13]
//                                    vid2_pll_clk,               // [12]
//                                    cts_eth_rmii,               // [11]
//                                    cts_enct_clk,               // [10]
//                                    cts_encl_clk,               // [9]
//                                    cts_encp_clk,               // [8]
//                                    clk81,                      // [7]
//                                    vid_pll_clk,                // [6]
//                                    aud_pll_clk,                // [5]
//                                    misc_pll_clk,               // [4]
//                                    ddr_pll_clk,                // [3]
//                                    sys_pll_clk,                // [2]
//                                    am_ring_osc_clk_out[1],     // [1]
//                                    am_ring_osc_clk_out[0]} ),  // [0]
//
// For Example
//
// unsigend long    clk81_clk   = clk_util_clk_msr( 2,      // mux select 2
//                                                  50 );   // measure for 50uS
//
// returns a value in "clk81_clk" in Hz
//
// The "uS_gate_time" can be anything between 1uS and 65535 uS, but the limitation is
// the circuit will only count 65536 clocks.  Therefore the uS_gate_time is limited by
//
//   uS_gate_time <= 65535/(expect clock frequency in MHz)
//
// For example, if the expected frequency is 400Mhz, then the uS_gate_time should
// be less than 163.
//
// Your measurement resolution is:
//
//    100% / (uS_gate_time * measure_val )
//
//
unsigned int clk_util_clk_msr(unsigned int clk_mux)
{
    unsigned int regval = 0;
    WRITE_CBUS_REG(MSR_CLK_REG0, 0);
    // Set the measurement gate to 64uS
    CLEAR_CBUS_REG_MASK(MSR_CLK_REG0, 0xffff);
    SET_CBUS_REG_MASK(MSR_CLK_REG0, (64 - 1)); //64uS is enough for measure the frequence?
    // Disable continuous measurement
    // disable interrupts
    CLEAR_CBUS_REG_MASK(MSR_CLK_REG0, ((1 << 18) | (1 << 17)));
    CLEAR_CBUS_REG_MASK(MSR_CLK_REG0, (0x1f << 20));
    SET_CBUS_REG_MASK(MSR_CLK_REG0, (clk_mux << 20) | // Select MUX
                                    (1 << 19) |       // enable the clock
									(1 << 16));       //enable measuring
    // Wait for the measurement to be done
    regval = READ_CBUS_REG(MSR_CLK_REG0);
    do {
        regval = READ_CBUS_REG(MSR_CLK_REG0);
    } while (regval & (1 << 31));

    // disable measuring
    CLEAR_CBUS_REG_MASK(MSR_CLK_REG0, (1 << 16));
    regval = (READ_CBUS_REG(MSR_CLK_REG2) + 31) & 0x000FFFFF;
    // Return value in MHz*measured_val
    return (regval >> 6);
}

unsigned  int get_system_clk(void)
{
    static unsigned int sys_freq = 0;
    if (sys_freq == 0) {
        sys_freq = (clk_util_clk_msr(SYS_PLL_CLK) * 1000000);
    }
    return sys_freq;
}
EXPORT_SYMBOL(get_system_clk);

unsigned int get_mpeg_clk(void)
{
    static unsigned int clk81_freq = 0;
    if (clk81_freq == 0) {
        clk81_freq = (clk_util_clk_msr(CLK81) * 1000000);
    }
    return clk81_freq;
}
EXPORT_SYMBOL(get_mpeg_clk);

unsigned int get_misc_pll_clk(void)
{
    static unsigned int freq = 0;
    if (freq == 0) {
        freq = (clk_util_clk_msr(MISC_PLL_CLK) * 1000000);
    }
    return freq;
}
EXPORT_SYMBOL(get_misc_pll_clk);

long clk_round_rate(struct clk *clk, unsigned long rate)
{
    if (rate < clk->min) {
        return clk->min;
    }

    if (rate > clk->max) {
        return clk->max;
    }

    return rate;
}
EXPORT_SYMBOL(clk_round_rate);

unsigned long clk_get_rate(struct clk *clk)
{
    if (!clk) {
        return 0;
    }

    if (clk->get_rate) {
        return clk->get_rate(clk);
    }

    return clk->rate;
}
EXPORT_SYMBOL(clk_get_rate);

int clk_set_rate(struct clk *clk, unsigned long rate)
{
    unsigned long flags;
    int ret;

    if (clk == NULL || clk->set_rate == NULL) {
        return -EINVAL;
    }

    spin_lock_irqsave(&clockfw_lock, flags);

    ret = clk->set_rate(clk, rate);

    spin_unlock_irqrestore(&clockfw_lock, flags);

    return ret;
}
EXPORT_SYMBOL(clk_set_rate);

static unsigned long xtal_get_rate(struct clk *clk)
{
    unsigned long rate;

    rate = get_xtal_clock();/*refresh from register*/
    clk->rate = rate;

    return rate;
}

static int clk_set_rate_sys_pll(struct clk *clk, unsigned long rate)
{
    unsigned long r = rate;
    int ret;

    if (r < 1000) {
        r = r * 1000000;
    }

    ret = sys_clkpll_setting(0, r);
    if (ret >= 0) {
        clk->rate = r;
    }

    return ret;
}

static int clk_set_rate_misc_pll(struct clk *clk, unsigned long rate)
{
    unsigned long r = rate;
    int ret;

    if (r < 1000) {
        r = r * 1000000;
    }

    ret = misc_pll_setting(0, r);

    if (ret == 0) {
        clk->rate = r;
    }

    return ret;
}

static int clk_set_rate_clk81(struct clk *clk, unsigned long rate)
{
    unsigned long r = rate;
    struct clk *father_clk;
    unsigned long r1;
    int ret;

    if (r < 1000) {
        r = r * 1000000;
    }

    father_clk = clk_get_sys("clk_misc_pll", NULL);

    r1 = clk_get_rate(father_clk);

    if (r1 != r * 4) {
        ret = father_clk->set_rate(father_clk, r * 4);

        if (ret != 0) {
            return ret;
        }
    }

    clk->rate = r;

    WRITE_MPEG_REG(HHI_MPEG_CLK_CNTL,
                   (1 << 12) |          // select SYS PLL
                   ((4 - 1) << 0) |     // div1
                   (1 << 7) |           // cntl_hi_mpeg_div_en, enable gating
                   (1 << 8));           // Connect clk81 to the PLL divider output

    return 0;
}

static int get_best_ratio_sys_pll(uint min_ratio, uint max_ratio, uint min_freq,
                                  uint max_freq, uint out_freq)
{
    uint delta_freq = max_freq - min_freq;
    uint best_freq = min_freq + (max_freq - min_freq) / 2;
    uint ratio = 0;
    uint pll_freq = 0;
    uint found_ratio = 0;
    uint i = 0;

    for (ratio = min_ratio; ratio < 4; ratio++) { /* Deal with 1-3 ratio */
        pll_freq = out_freq * ratio;
        printk("********%s: line %d ratio =%d, pll_freq =%d\n", __func__, __LINE__, ratio, pll_freq);
        if (pll_freq > 700 * CLK_1M && pll_freq < best_freq) {
            if (delta_freq > (best_freq - pll_freq)) {
                delta_freq = best_freq - pll_freq;
                found_ratio = ratio;
				printk("********%s: line %d delta_freq =%d found_ratio = %d\n", __func__, __LINE__, delta_freq, found_ratio);
            }
        } else if (pll_freq >= best_freq && pll_freq < 1300 * CLK_1M) {
            if (delta_freq > (pll_freq - best_freq)) {
                delta_freq = pll_freq - best_freq;
                found_ratio = ratio;
				printk("********%s: line %d delta_freq =%d found_ratio = %d\n", __func__, __LINE__, delta_freq, found_ratio);
            }
        } else if (pll_freq > 1300 * CLK_1M) {
            break;
        }
    }
    if (!found_ratio) {
        for (i=2; i< (max_ratio/2); i++) {
			ratio = i * 2;
            pll_freq = out_freq * ratio;
            printk("********%s: line %d ratio =%d, pll_freq =%d\n", __func__, __LINE__, ratio, pll_freq);
            if (pll_freq > 700 * CLK_1M && pll_freq < best_freq) {
                if (delta_freq > (best_freq - pll_freq)) {
                    delta_freq = best_freq - pll_freq;
                    found_ratio = ratio;
				printk("********%s: line %d delta_freq =%d found_ratio = %d\n", __func__, __LINE__, delta_freq, found_ratio);
                }
            } else if (pll_freq >= best_freq && pll_freq < 1300 * CLK_1M) {
                if (delta_freq > (pll_freq - best_freq)) {
                    delta_freq = pll_freq - best_freq;
                    found_ratio = ratio;
				printk("********%s: line %d delta_freq =%d found_ratio = %d\n", __func__, __LINE__, delta_freq, found_ratio);
                }
            } else if (pll_freq > 1300 * CLK_1M) {
                break;
            }
        }
    }

    /* Check if the ratio is avaliable */
    if (found_ratio){
			printk("********%s: line %d found_ratio = %d\n", __func__, __LINE__, found_ratio);
            return found_ratio;
    }

	return -1;
}

static int clk_set_rate_a9_clk(struct clk *clk, unsigned long rate)
{
    unsigned long r = rate;
    struct clk *father_clk;
    unsigned long r1;
    int ret;
    uint ratio = 0;
    unsigned long flags;

    if (r < 1000) {
        r = r * 1000000;
    }

    father_clk = clk_get_sys("clk_sys_pll", NULL);

    r1 = clk_get_rate(father_clk);

    if (!r1) {
        return -1;
    }

    if (r1 % r) { /* If the PLL freq is not the multuply of requiments, we need re-configurate sys pll */
        ratio = get_best_ratio_sys_pll(1, (0x3f + 1) * 2, 700 * CLK_1M, 1300 * CLK_1M,  r);
        if (ratio > 0) {
            ret = father_clk->set_rate(father_clk, r * ratio);
            if (ret != 0) {
                return ret;
            }
        }
    } else { /* sys pll is multuply of requiments freq, we need not change sys pll setting*/
        ratio = r1 / r;
	}

    clk->rate = r;

    local_irq_save(flags);
    WRITE_MPEG_REG(HHI_SYS_CPU_CLK_CNTL, // A9 clk set to system clock/2
                   (1 << 0) |  // 1 - sys pll clk
                   ((ratio < 3 ? (ratio - 1) : 3) << 2) | // sys pll div 2
                   (1 << 4) |  // APB_CLK_ENABLE
                   (1 << 5) |  // AT_CLK_ENABLE
                   (1 << 7) |  // Connect A9 to the PLL divider output
                   ((ratio < 3 ? 0 : (ratio / 2) - 1) << 8)); // Connect A9 to the PLL divider output
    printk("********%s: READ_MPEG_REG(HHI_SYS_CPU_CLK_CNTL) = 0x%x\n", __FUNCTION__, READ_MPEG_REG(HHI_SYS_CPU_CLK_CNTL));
    udelay(100);
    printk("********%s: clk_util_clk_msr(CTS_A9_CLK) = %dMHz\n", __FUNCTION__, clk_util_clk_msr(CTS_A9_CLK));
    local_irq_restore(flags);

    return 0;
}

static struct clk xtal_clk = {
    .name       = "clk_xtal",
    .rate       = 24000000,
    .get_rate   = xtal_get_rate,
    .set_rate   = NULL,
};

static struct clk clk_sys_pll = {
    .name       = "clk_sys_pll",
    .rate       = 800000000,
    .min        = 200000000,
    .max        = 2000000000,
    .set_rate   = clk_set_rate_sys_pll,
};

static struct clk clk_misc_pll = {
    .name       = "clk_misc_pll",
    .rate       = 800000000,
    .min        = 200000000,
    .max        = 800000000,
    .set_rate   = clk_set_rate_misc_pll,
};

static struct clk clk_ddr_pll = {
    .name       = "clk_ddr",
    .rate       = 400000000,
    .set_rate   = NULL,
};

static struct clk clk81 = {
    .name       = "clk81",
    .rate       = 200000000,
    .min        = 100000000,
    .max        = 400000000,
    .set_rate   = clk_set_rate_clk81,
};

static struct clk a9_clk = {
    .name       = "a9_clk",
    .rate       = 800000000,
    .min        = 200000000,
#if defined(CONFIG_ARCH_MESON3) && defined(CONFIG_CPU_FREQ)
    .max        = 600000000,
#else
    .max        = 800000000,
#endif
    .set_rate   = clk_set_rate_a9_clk,
};

REGISTER_CLK(DDR);
REGISTER_CLK(VLD_CLK);
REGISTER_CLK(IQIDCT_CLK);
REGISTER_CLK(MC_CLK);
REGISTER_CLK(AHB_BRIDGE);
REGISTER_CLK(ISA);
REGISTER_CLK(APB_CBUS);
REGISTER_CLK(_1200XXX);
REGISTER_CLK(SPICC);
REGISTER_CLK(I2C);
REGISTER_CLK(SAR_ADC);
REGISTER_CLK(SMART_CARD_MPEG_DOMAIN);
REGISTER_CLK(RANDOM_NUM_GEN);
REGISTER_CLK(UART0);
REGISTER_CLK(SDHC);
REGISTER_CLK(STREAM);
REGISTER_CLK(ASYNC_FIFO);
REGISTER_CLK(SDIO);
REGISTER_CLK(AUD_BUF);
REGISTER_CLK(HIU_PARSER);
REGISTER_CLK(RESERVED0);
REGISTER_CLK(AMRISC);
REGISTER_CLK(BT656_IN);
REGISTER_CLK(ASSIST_MISC);
REGISTER_CLK(VENC_I_TOP);
REGISTER_CLK(VENC_P_TOP);
REGISTER_CLK(VENC_T_TOP);
REGISTER_CLK(VENC_DAC);
REGISTER_CLK(VI_CORE);
REGISTER_CLK(RESERVED1);
REGISTER_CLK(SPI2);
REGISTER_CLK(MDEC_CLK_ASSIST);
REGISTER_CLK(MDEC_CLK_PSC);
REGISTER_CLK(SPI1);
REGISTER_CLK(AUD_IN);
REGISTER_CLK(ETHERNET);
REGISTER_CLK(DEMUX);
REGISTER_CLK(RESERVED2);
REGISTER_CLK(AIU_AI_TOP_GLUE);
REGISTER_CLK(AIU_IEC958);
REGISTER_CLK(AIU_I2S_OUT);
REGISTER_CLK(AIU_AMCLK_MEASURE);
REGISTER_CLK(AIU_AIFIFO2);
REGISTER_CLK(AIU_AUD_MIXER);
REGISTER_CLK(AIU_MIXER_REG);
REGISTER_CLK(AIU_ADC);
REGISTER_CLK(BLK_MOV);
REGISTER_CLK(RESERVED3);
REGISTER_CLK(UART1);
REGISTER_CLK(LED_PWM);
REGISTER_CLK(VGHL_PWM);
REGISTER_CLK(RESERVED4);
REGISTER_CLK(GE2D);
REGISTER_CLK(USB0);
REGISTER_CLK(USB1);
REGISTER_CLK(RESET);
REGISTER_CLK(NAND);
REGISTER_CLK(HIU_PARSER_TOP);
REGISTER_CLK(MDEC_CLK_DBLK);
REGISTER_CLK(MDEC_CLK_PIC_DC);
REGISTER_CLK(VIDEO_IN);
REGISTER_CLK(AHB_ARB0);
REGISTER_CLK(EFUSE);
REGISTER_CLK(ROM_CLK);
REGISTER_CLK(RESERVED5);
REGISTER_CLK(AHB_DATA_BUS);
REGISTER_CLK(AHB_CONTROL_BUS);
REGISTER_CLK(HDMI_INTR_SYNC);
REGISTER_CLK(HDMI_PCLK);
REGISTER_CLK(RESERVED6);
REGISTER_CLK(RESERVED7);
REGISTER_CLK(RESERVED8);
REGISTER_CLK(MISC_USB1_TO_DDR);
REGISTER_CLK(MISC_USB0_TO_DDR);
REGISTER_CLK(AIU_PCLK);
REGISTER_CLK(MMC_PCLK);
REGISTER_CLK(MISC_DVIN);
REGISTER_CLK(MISC_RDMA);
REGISTER_CLK(RESERVED9);
REGISTER_CLK(UART2);
REGISTER_CLK(VENCI_INT);
REGISTER_CLK(VIU2);
REGISTER_CLK(VENCP_INT);
REGISTER_CLK(VENCT_INT);
REGISTER_CLK(VENCL_INT);
REGISTER_CLK(VENC_L_TOP);
REGISTER_CLK(VCLK2_VENCI);
REGISTER_CLK(VCLK2_VENCI1);
REGISTER_CLK(VCLK2_VENCP);
REGISTER_CLK(VCLK2_VENCP1);
REGISTER_CLK(VCLK2_VENCT);
REGISTER_CLK(VCLK2_VENCT1);
REGISTER_CLK(VCLK2_OTHER);
REGISTER_CLK(VCLK2_ENCI);
REGISTER_CLK(VCLK2_ENCP);
REGISTER_CLK(DAC_CLK);
REGISTER_CLK(AIU_AOCLK);
REGISTER_CLK(AIU_AMCLK);
REGISTER_CLK(AIU_ICE958_AMCLK);
REGISTER_CLK(VCLK1_HDMI);
REGISTER_CLK(AIU_AUDIN_SCLK);
REGISTER_CLK(ENC480P);
REGISTER_CLK(VCLK2_ENCT);
REGISTER_CLK(VCLK2_ENCL);
REGISTER_CLK(VCLK2_VENCL);
REGISTER_CLK(VCLK2_VENCL1);
REGISTER_CLK(VCLK2_OTHER1);

static struct clk_lookup lookups[] = {
    {
        .dev_id = "clk_xtal",
        .clk    = &xtal_clk,
    },
    {
        .dev_id = "clk_sys_pll",
        .clk    = &clk_sys_pll,
    },
    {
        .dev_id = "clk_misc_pll",
        .clk    = &clk_misc_pll,
    },
    {
        .dev_id = "clk_ddr_pll",
        .clk    = &clk_ddr_pll,
    },
    {
        .dev_id = "clk81",
        .clk    = &clk81,
    },
    {
        .dev_id = "a9_clk",
        .clk    = &a9_clk,
    },
    CLK_LOOKUP_ITEM(DDR),
    CLK_LOOKUP_ITEM(VLD_CLK),
    CLK_LOOKUP_ITEM(IQIDCT_CLK),
    CLK_LOOKUP_ITEM(MC_CLK),
    CLK_LOOKUP_ITEM(AHB_BRIDGE),
    CLK_LOOKUP_ITEM(ISA),
    CLK_LOOKUP_ITEM(APB_CBUS),
    CLK_LOOKUP_ITEM(_1200XXX),
    CLK_LOOKUP_ITEM(SPICC),
    CLK_LOOKUP_ITEM(I2C),
    CLK_LOOKUP_ITEM(SAR_ADC),
    CLK_LOOKUP_ITEM(SMART_CARD_MPEG_DOMAIN),
    CLK_LOOKUP_ITEM(RANDOM_NUM_GEN),
    CLK_LOOKUP_ITEM(UART0),
    CLK_LOOKUP_ITEM(SDHC),
    CLK_LOOKUP_ITEM(STREAM),
    CLK_LOOKUP_ITEM(ASYNC_FIFO),
    CLK_LOOKUP_ITEM(SDIO),
    CLK_LOOKUP_ITEM(AUD_BUF),
    CLK_LOOKUP_ITEM(HIU_PARSER),
    CLK_LOOKUP_ITEM(RESERVED0),
    CLK_LOOKUP_ITEM(AMRISC),
    CLK_LOOKUP_ITEM(BT656_IN),
    CLK_LOOKUP_ITEM(ASSIST_MISC),
    CLK_LOOKUP_ITEM(VENC_I_TOP),
    CLK_LOOKUP_ITEM(VENC_P_TOP),
    CLK_LOOKUP_ITEM(VENC_T_TOP),
    CLK_LOOKUP_ITEM(VENC_DAC),
    CLK_LOOKUP_ITEM(VI_CORE),
    CLK_LOOKUP_ITEM(RESERVED1),
    CLK_LOOKUP_ITEM(SPI2),
    CLK_LOOKUP_ITEM(MDEC_CLK_ASSIST),
    CLK_LOOKUP_ITEM(MDEC_CLK_PSC),
    CLK_LOOKUP_ITEM(SPI1),
    CLK_LOOKUP_ITEM(AUD_IN),
    CLK_LOOKUP_ITEM(ETHERNET),
    CLK_LOOKUP_ITEM(DEMUX),
    CLK_LOOKUP_ITEM(RESERVED2),
    CLK_LOOKUP_ITEM(AIU_AI_TOP_GLUE),
    CLK_LOOKUP_ITEM(AIU_IEC958),
    CLK_LOOKUP_ITEM(AIU_I2S_OUT),
    CLK_LOOKUP_ITEM(AIU_AMCLK_MEASURE),
    CLK_LOOKUP_ITEM(AIU_AIFIFO2),
    CLK_LOOKUP_ITEM(AIU_AUD_MIXER),
    CLK_LOOKUP_ITEM(AIU_MIXER_REG),
    CLK_LOOKUP_ITEM(AIU_ADC),
    CLK_LOOKUP_ITEM(BLK_MOV),
    CLK_LOOKUP_ITEM(RESERVED3),
    CLK_LOOKUP_ITEM(UART1),
    CLK_LOOKUP_ITEM(LED_PWM),
    CLK_LOOKUP_ITEM(VGHL_PWM),
    CLK_LOOKUP_ITEM(RESERVED4),
    CLK_LOOKUP_ITEM(GE2D),
    CLK_LOOKUP_ITEM(USB0),
    CLK_LOOKUP_ITEM(USB1),
    CLK_LOOKUP_ITEM(RESET),
    CLK_LOOKUP_ITEM(NAND),
    CLK_LOOKUP_ITEM(HIU_PARSER_TOP),
    CLK_LOOKUP_ITEM(MDEC_CLK_DBLK),
    CLK_LOOKUP_ITEM(MDEC_CLK_PIC_DC),
    CLK_LOOKUP_ITEM(VIDEO_IN),
    CLK_LOOKUP_ITEM(AHB_ARB0),
    CLK_LOOKUP_ITEM(EFUSE),
    CLK_LOOKUP_ITEM(ROM_CLK),
    CLK_LOOKUP_ITEM(RESERVED5),
    CLK_LOOKUP_ITEM(AHB_DATA_BUS),
    CLK_LOOKUP_ITEM(AHB_CONTROL_BUS),
    CLK_LOOKUP_ITEM(HDMI_INTR_SYNC),
    CLK_LOOKUP_ITEM(HDMI_PCLK),
    CLK_LOOKUP_ITEM(RESERVED6),
    CLK_LOOKUP_ITEM(RESERVED7),
    CLK_LOOKUP_ITEM(RESERVED8),
    CLK_LOOKUP_ITEM(MISC_USB1_TO_DDR),
    CLK_LOOKUP_ITEM(MISC_USB0_TO_DDR),
    CLK_LOOKUP_ITEM(AIU_PCLK),
    CLK_LOOKUP_ITEM(MMC_PCLK),
    CLK_LOOKUP_ITEM(MISC_DVIN),
    CLK_LOOKUP_ITEM(MISC_RDMA),
    CLK_LOOKUP_ITEM(RESERVED9),
    CLK_LOOKUP_ITEM(UART2),
    CLK_LOOKUP_ITEM(VENCI_INT),
    CLK_LOOKUP_ITEM(VIU2),
    CLK_LOOKUP_ITEM(VENCP_INT),
    CLK_LOOKUP_ITEM(VENCT_INT),
    CLK_LOOKUP_ITEM(VENCL_INT),
    CLK_LOOKUP_ITEM(VENC_L_TOP),
    CLK_LOOKUP_ITEM(VCLK2_VENCI),
    CLK_LOOKUP_ITEM(VCLK2_VENCI1),
    CLK_LOOKUP_ITEM(VCLK2_VENCP),
    CLK_LOOKUP_ITEM(VCLK2_VENCP1),
    CLK_LOOKUP_ITEM(VCLK2_VENCT),
    CLK_LOOKUP_ITEM(VCLK2_VENCT1),
    CLK_LOOKUP_ITEM(VCLK2_OTHER),
    CLK_LOOKUP_ITEM(VCLK2_ENCI),
    CLK_LOOKUP_ITEM(VCLK2_ENCP),
    CLK_LOOKUP_ITEM(DAC_CLK),
    CLK_LOOKUP_ITEM(AIU_AOCLK),
    CLK_LOOKUP_ITEM(AIU_AMCLK),
    CLK_LOOKUP_ITEM(AIU_ICE958_AMCLK),
    CLK_LOOKUP_ITEM(VCLK1_HDMI),
    CLK_LOOKUP_ITEM(AIU_AUDIN_SCLK),
    CLK_LOOKUP_ITEM(ENC480P),
    CLK_LOOKUP_ITEM(VCLK2_ENCT),
    CLK_LOOKUP_ITEM(VCLK2_ENCL),
    CLK_LOOKUP_ITEM(VCLK2_VENCL),
    CLK_LOOKUP_ITEM(VCLK2_VENCL1),
    CLK_LOOKUP_ITEM(VCLK2_OTHER1),
};

static int __init meson_clock_init(void)
{
    int od;
    if (init_clock && init_clock != a9_clk.rate) {
        od = sys_clkpll_setting(0, init_clock);
        if (od >= 0) {
            a9_clk.rate = init_clock;
            clk_sys_pll.rate = init_clock << od;
        }
    }

    /* Register the lookups */
    clkdev_add_table(lookups, ARRAY_SIZE(lookups));

    return 0;
}

/* initialize clocking early to be available later in the boot */
core_initcall(meson_clock_init);

unsigned long long clkparse(const char *ptr, char **retptr)
{
    char *endptr;   /* local pointer to end of parsed string */

    unsigned long long ret = simple_strtoull(ptr, &endptr, 0);

    switch (*endptr) {
    case 'G':
    case 'g':
        ret *= 1000;
    case 'M':
    case 'm':
        ret *= 1000;
    case 'K':
    case 'k':
        ret *= 1000;
        endptr++;
    default:
        break;
    }

    if (retptr) {
        *retptr = endptr;
    }

    return ret;
}

static int __init a9_clock_setup(char *ptr)
{
    unsigned long flags;
	int od;
    int ret = 0;

	init_clock = clkparse(ptr, 0);

    od = sys_clkpll_setting(0, init_clock);
    if (od >= 0) {
        local_irq_save(flags);
        a9_clk.rate = init_clock;
        clk_sys_pll.rate = init_clock*od;
        printk("********%s: READ_MPEG_REG(HHI_SYS_PLL_CNTL) = 0x%x\n", __FUNCTION__, READ_MPEG_REG(HHI_SYS_PLL_CNTL));
		printk("********%s: READ_MPEG_REG(HHI_SYS_CPU_CLK_CNTL) = 0x%x\n", __FUNCTION__, READ_MPEG_REG(HHI_SYS_CPU_CLK_CNTL));
        while(ret * CLK_1M != init_clock){
            ret = clk_util_clk_msr(CTS_A9_CLK);
    		printk("********%s: clk_util_clk_msr(%d) = %dMHz\n", __FUNCTION__, CTS_A9_CLK, ret);
    	}
        local_irq_restore(flags);
    }

    return 0;
}
__setup("a9_clk=", a9_clock_setup);

static int __init clk81_clock_setup(char *ptr)
{
    int clock = clkparse(ptr, 0);

    if (misc_pll_setting(0, clock * 4) == 0) {
        /* todo: uart baudrate depends on clk81, assume 115200 baudrate */
        int baudrate = (clock / (115200 * 4)) - 1;

        clk_misc_pll.rate = clock * 4;
        clk81.rate = clock;
        WRITE_MPEG_REG(HHI_MPEG_CLK_CNTL,   // MPEG clk81 set to misc/4
                       (2 << 12) |               // select misc PLL
                       ((4 - 1) << 0) |          // div1
                       (1 << 7) |                // cntl_hi_mpeg_div_en, enable gating
                       (1 << 8) |                // Connect clk81 to the PLL divider output
					   (1 << 15));                // Production clock enable

        CLEAR_CBUS_REG_MASK(UART0_CONTROL, (1 << 19) | 0xFFF);
        SET_CBUS_REG_MASK(UART0_CONTROL, (baudrate & 0xfff));

        CLEAR_CBUS_REG_MASK(UART1_CONTROL, (1 << 19) | 0xFFF);
        SET_CBUS_REG_MASK(UART1_CONTROL, (baudrate & 0xfff));

        WRITE_AOBUS_REG_BITS(AO_UART_CONTROL, baudrate & 0xfff, 0, 12);

        WRITE_CBUS_REG(HHI_MALI_CLK_CNTL,
                       (2 << 9)    |   // select misc pll as clock source
                       (1 << 8)    |   // enable clock gating
                       (3 << 0));      // Misc clk / 4
    }


    return 0;
}
__setup("clk81=", clk81_clock_setup);

int clk_enable(struct clk *clk)
{
    unsigned long flags;

    spin_lock_irqsave(&clockfw_lock, flags);

    if (clk->clock_index >= 0 && clk->clock_index < GCLK_IDX_MAX && clk->clock_gate_reg_adr != 0) {
        if (GCLK_ref[clk->clock_index]++ == 0) {
            SET_CBUS_REG_MASK(clk->clock_gate_reg_adr, clk->clock_gate_reg_mask);
        }
    }

    spin_unlock_irqrestore(&clockfw_lock, flags);

    return 0;
}
EXPORT_SYMBOL(clk_enable);

void clk_disable(struct clk *clk)
{
    unsigned long flags;

    spin_lock_irqsave(&clockfw_lock, flags);

    if (clk->clock_index >= 0 && clk->clock_index < GCLK_IDX_MAX && clk->clock_gate_reg_adr != 0) {
        if ((GCLK_ref[clk->clock_index] != 0) && (--GCLK_ref[clk->clock_index] == 0)) {
            CLEAR_CBUS_REG_MASK(clk->clock_gate_reg_adr, clk->clock_gate_reg_mask);
        }
    }

    spin_unlock_irqrestore(&clockfw_lock, flags);
}
EXPORT_SYMBOL(clk_disable);
