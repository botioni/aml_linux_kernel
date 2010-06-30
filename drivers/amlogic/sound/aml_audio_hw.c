#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <mach/am_regs.h>
#include "aml_audio_hw.h"

#ifndef MREG_AIU_958_chstat0
#define AIU_958_chstat0	AIU_958_CHSTAT_L0
#endif

#ifndef MREG_AIU_958_chstat1
#define AIU_958_chstat1	AIU_958_CHSTAT_L1
#endif

unsigned ENABLE_IEC958 = 0;
void audio_set_aiubuf(u32 addr, u32 size)
{
    WRITE_MPEG_REG(AIU_MEM_I2S_START_PTR, addr & 0xffffffc0);
    WRITE_MPEG_REG(AIU_MEM_I2S_RD_PTR, addr & 0xffffffc0);
    WRITE_MPEG_REG(AIU_MEM_I2S_END_PTR, (addr & 0xffffffc0) + (size & 0xffffffc0) - 64);   //this is for 16bit 2 channel

    WRITE_MPEG_REG_BITS(AIU_MEM_I2S_CONTROL, 1, 0, 1);
    WRITE_MPEG_REG_BITS(AIU_MEM_I2S_CONTROL, 0, 0, 1);

    WRITE_MPEG_REG(AIU_MEM_I2S_BUF_CNTL, 1 | (0 << 1));
    WRITE_MPEG_REG(AIU_MEM_I2S_BUF_CNTL, 0 | (0 << 1));

    memset((void *)addr, 0, size);
}

void audio_set_958outbuf(u32 addr, u32 size)
{
    if (ENABLE_IEC958) {
        WRITE_MPEG_REG(AIU_MEM_IEC958_START_PTR, addr & 0xffffffc0);
        WRITE_MPEG_REG(AIU_MEM_IEC958_RD_PTR, addr & 0xffffffc0);
        WRITE_MPEG_REG(AIU_MEM_IEC958_END_PTR, (addr & 0xffffffc0) + (size & 0xffffffc0) - 64);    // this is for 16bit 2 channel

        WRITE_MPEG_REG_BITS(AIU_MEM_IEC958_CONTROL, 1, 0, 1);
        WRITE_MPEG_REG_BITS(AIU_MEM_IEC958_CONTROL, 0, 0, 1);

        WRITE_MPEG_REG(AIU_MEM_IEC958_BUF_CNTL, 1 | (0 << 1));
        WRITE_MPEG_REG(AIU_MEM_IEC958_BUF_CNTL, 0 | (0 << 1));
    }
    memset((void *)addr, 0, size);
}

void audio_set_i2s_mode(u32 mode)
{
    const unsigned short control[4] = {
        0x14,                   /* AIU_I2S_MODE_2x16 */
        0x30,                   /* AIU_I2S_MODE_2x24 */
        0x11                    /* AIU_I2S_MODE_8x24 */
    };

    const unsigned short mask[4] = {
        0x303,                  /*2 ch in, 2ch out */
        0x303,                  /*2ch in, 2ch out */
        0xffff                  /*8ch in, 8ch out */
    };

    if (mode < sizeof(control) / sizeof(unsigned short)) {
        WRITE_MPEG_REG(AIU_I2S_SOURCE_DESC, 1);

        if (mode == 0) {
            WRITE_MPEG_REG_BITS(AIU_MEM_I2S_CONTROL, 1, 6, 1);
        } else {
            WRITE_MPEG_REG_BITS(AIU_MEM_I2S_CONTROL, 0, 6, 1);
        }
        WRITE_MPEG_REG_BITS(AIU_MEM_I2S_MASKS, mask[mode], 0, 16);

        WRITE_MPEG_REG_BITS(AIU_MEM_I2S_CONTROL, 1, 0, 1);
        WRITE_MPEG_REG_BITS(AIU_MEM_I2S_CONTROL, 0, 0, 1);

        if (ENABLE_IEC958) {
            WRITE_MPEG_REG_BITS(AIU_MEM_IEC958_MASKS, mask[mode], 0,
                                16);
            WRITE_MPEG_REG_BITS(AIU_MEM_IEC958_CONTROL, 1, 0, 1);
            WRITE_MPEG_REG_BITS(AIU_MEM_IEC958_CONTROL, 0, 0, 1);
        }
    }

}

const _aiu_clk_setting_t freq_tab_384fs[7] = {
    {AUDIO_384FS_PLL_192K, AUDIO_384FS_PLL_192K_MUX, AUDIO_384FS_CLK_192K},
    {AUDIO_384FS_PLL_176K, AUDIO_384FS_PLL_176K_MUX, AUDIO_384FS_CLK_176K},
    {AUDIO_384FS_PLL_96K, AUDIO_384FS_PLL_96K_MUX, AUDIO_384FS_CLK_96K},
    {AUDIO_384FS_PLL_88K, AUDIO_384FS_PLL_88K_MUX, AUDIO_384FS_CLK_88K},
    {AUDIO_384FS_PLL_48K, AUDIO_384FS_PLL_48K_MUX, AUDIO_384FS_CLK_48K},
    {AUDIO_384FS_PLL_44K, AUDIO_384FS_PLL_44K_MUX, AUDIO_384FS_CLK_44K},
    {AUDIO_384FS_PLL_32K, AUDIO_384FS_PLL_32K_MUX, AUDIO_384FS_CLK_32K}
};

const _aiu_clk_setting_t freq_tab_256fs[7] = {
    {AUDIO_256FS_PLL_192K, AUDIO_256FS_PLL_192K_MUX, AUDIO_256FS_CLK_192K},
    {AUDIO_256FS_PLL_176K, AUDIO_256FS_PLL_176K_MUX, AUDIO_256FS_CLK_176K},
    {AUDIO_256FS_PLL_96K, AUDIO_256FS_PLL_96K_MUX, AUDIO_256FS_CLK_96K},
    {AUDIO_256FS_PLL_88K, AUDIO_256FS_PLL_88K_MUX, AUDIO_256FS_CLK_88K},
    {AUDIO_256FS_PLL_48K, AUDIO_256FS_PLL_48K_MUX, AUDIO_256FS_CLK_48K},
    {AUDIO_256FS_PLL_44K, AUDIO_256FS_PLL_44K_MUX, AUDIO_256FS_CLK_44K},
    {AUDIO_256FS_PLL_32K, AUDIO_256FS_PLL_32K_MUX, AUDIO_256FS_CLK_32K}
};

void audio_util_set_dac_format(unsigned format)
{
    if (format == AUDIO_ALGOUT_DAC_FORMAT_DSP) {
        WRITE_MPEG_REG_BITS(AIU_CLK_CTRL, 1, 8, 2);
    } else if (format == AUDIO_ALGOUT_DAC_FORMAT_LEFT_JUSTIFY) {
        WRITE_MPEG_REG_BITS(AIU_CLK_CTRL, 0, 8, 2);
    }

}

void audio_set_clk(unsigned freq, unsigned fs_config)
{
    int i;
    if (fs_config == AUDIO_CLK_256FS) {
        //WRITE_MPEG_REG_BITS(MREG_AUDIO_CLK_CTRL, 0, 8, 1);
        WRITE_MPEG_REG(HHI_AUD_PLL_CNTL, freq_tab_256fs[freq].pll);
        for (i = 0; i < 100000; i++) ;
        WRITE_MPEG_REG(HHI_AUD_CLK_CNTL, freq_tab_256fs[freq].mux);
        WRITE_MPEG_REG_BITS(HHI_AUD_CLK_CNTL, 1, 8, 1);

        WRITE_MPEG_REG_BITS(AIU_CLK_CTRL,
                            freq_tab_256fs[freq].devisor, 0, 8);
        WRITE_MPEG_REG(AIU_I2S_DAC_CFG, AUDIO_256FS_DAC_CFG);
    } else if (fs_config == AUDIO_CLK_384FS) {
        //WRITE_MPEG_REG_BITS(MREG_AUDIO_CLK_CTRL, 0, 8, 1);
        WRITE_MPEG_REG(HHI_AUD_PLL_CNTL, freq_tab_384fs[freq].pll);
        for (i = 0; i < 100000; i++) ;
        WRITE_MPEG_REG(HHI_AUD_CLK_CNTL, freq_tab_384fs[freq].mux);
        WRITE_MPEG_REG_BITS(HHI_AUD_CLK_CNTL, 1, 8, 1);
        WRITE_MPEG_REG_BITS(AIU_CLK_CTRL,
                            freq_tab_384fs[freq].devisor, 0, 8);
        WRITE_MPEG_REG(AIU_I2S_DAC_CFG, AUDIO_384FS_DAC_CFG);
    }
}

void audio_enable_ouput(int flag)
{
    if (flag) {

        WRITE_MPEG_REG(AIU_RST_SOFT, 0x05);
        READ_MPEG_REG(AIU_I2S_SYNC);
		//what about this reg mean?
       // WRITE_MPEG_REG_BITS(DDR_TOP_CTL2, 3, 3, 2);
        WRITE_MPEG_REG_BITS(AIU_MEM_I2S_CONTROL, 3, 1, 2);

        if (ENABLE_IEC958) {
            READ_MPEG_REG(AIU_I2S_SYNC);
            WRITE_MPEG_REG(AIU_958_FORCE_LEFT, 0);
            WRITE_MPEG_REG(AIU_958_DCU_FF_CTRL, 1);

            WRITE_MPEG_REG_BITS(AIU_MEM_IEC958_CONTROL, 3, 1, 2);
        }
    } else {
        WRITE_MPEG_REG_BITS(AIU_MEM_I2S_CONTROL, 0, 1, 2);
//              WRITE_MPEG_REG_BITS(MREG_AIU_MEM_I2S_CONTROL, 1, 0, 1);

        if (ENABLE_IEC958) {
            WRITE_MPEG_REG(AIU_958_DCU_FF_CTRL, 0);
            WRITE_MPEG_REG_BITS(AIU_MEM_IEC958_CONTROL, 0, 1, 2);
//              WRITE_MPEG_REG_BITS(MREG_AIU_MEM_IEC958_CONTROL, 1, 0, 1);
        }
    }
}

unsigned int read_i2s_rd_ptr(void)
{
    unsigned int val;
    val = READ_MPEG_REG(AIU_MEM_I2S_RD_PTR);
    return val;
}

void audio_i2s_unmute(void)
{
    WRITE_MPEG_REG_BITS(AIU_I2S_MUTE_SWAP, 0, 8, 8);
    WRITE_MPEG_REG_BITS(AIU_958_CTRL, 0, 3, 2);
}

void audio_i2s_mute(void)
{
    WRITE_MPEG_REG_BITS(AIU_I2S_MUTE_SWAP, 0xff, 8, 8);
    WRITE_MPEG_REG_BITS(AIU_958_CTRL, 3, 3, 2);
}

void audio_hw_958_reset(unsigned slow_domain, unsigned fast_domain)
{
    WRITE_MPEG_REG(AIU_RST_SOFT,
                   (slow_domain << 3) | (fast_domain << 2));
}

void set_958_channel_status(_aiu_958_channel_status_t * set)
{
    if (set) {
        WRITE_MPEG_REG(AIU_958_chstat0, set->chstat0_l);
        WRITE_MPEG_REG(AIU_958_chstat1, set->chstat1_l);
        WRITE_MPEG_REG(AIU_958_CHSTAT_L0, set->chstat0_r);
        WRITE_MPEG_REG(AIU_958_CHSTAT_L1, set->chstat1_r);
    }
}

void audio_hw_set_958_pcm24(_aiu_958_raw_setting_t * set)
{
    WRITE_MPEG_REG(AIU_958_BPF, 0x80); /* in pcm mode, set bpf to 128 */
    set_958_channel_status(set->chan_stat);
}

void audio_hw_set_958_mode(unsigned mode, _aiu_958_raw_setting_t * set)
{
    if (mode == AIU_958_MODE_RAW) {
        // audio_hw_set_958_raw(set);
        if (ENABLE_IEC958) {
            WRITE_MPEG_REG(AIU_958_MISC, 1);
            WRITE_MPEG_REG_BITS(AIU_MEM_IEC958_CONTROL, 1, 8, 1);  // raw
        }
    } else if (mode == AIU_958_MODE_PCM24) {
        audio_hw_set_958_pcm24(set);
        if (ENABLE_IEC958) {
            WRITE_MPEG_REG(AIU_958_MISC, 0x2020 | (1 << 7));
            WRITE_MPEG_REG_BITS(AIU_MEM_IEC958_CONTROL, 0, 8, 1);  // pcm
            WRITE_MPEG_REG_BITS(AIU_MEM_IEC958_CONTROL, 0, 7, 1);  // 16bit
        }
    } else if (mode == AIU_958_MODE_PCM16) {
        audio_hw_set_958_pcm24(set);
        if (ENABLE_IEC958) {
            WRITE_MPEG_REG(AIU_958_MISC, 0x2042);
            WRITE_MPEG_REG_BITS(AIU_MEM_IEC958_CONTROL, 0, 8, 1);  // pcm
            WRITE_MPEG_REG_BITS(AIU_MEM_IEC958_CONTROL, 1, 7, 1);  // 16bit
        }
    }

    audio_hw_958_reset(0, 1);

    WRITE_MPEG_REG(AIU_958_FORCE_LEFT, 1);
}

void audio_hw_958_enable(unsigned flag)
{
    if (ENABLE_IEC958) {
        WRITE_MPEG_REG_BITS(AIU_MEM_IEC958_CONTROL, flag, 2, 1);
        WRITE_MPEG_REG_BITS(AIU_MEM_IEC958_CONTROL, flag, 1, 1);
        WRITE_MPEG_REG_BITS(AIU_MEM_IEC958_CONTROL, flag, 0, 1);
    }
}

unsigned int read_i2s_mute_swap_reg(void)
{
	unsigned int val;
    	val = READ_MPEG_REG(AIU_I2S_MUTE_SWAP);
    	return val;
}

void audio_i2s_swap_left_right(unsigned int flag)
{
	WRITE_MPEG_REG_BITS(AIU_I2S_MUTE_SWAP, flag, 0, 2);
}
