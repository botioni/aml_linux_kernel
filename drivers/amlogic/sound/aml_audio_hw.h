#ifndef __AML_AUDIO_HW_H__
#define __AML_AUDIO_HW_H__

#if defined(AML_A1H)
#define AUDIO_384FS_PLL_192K        0x4e81  /* 36.864M */
#define AUDIO_384FS_PLL_192K_MUX    0x5
#define AUDIO_384FS_CLK_192K        0x5eb

#define AUDIO_384FS_PLL_176K        0x127f  /* 33.8688M */
#define AUDIO_384FS_PLL_176K_MUX    0x9
#define AUDIO_384FS_CLK_176K        0x5eb

#define AUDIO_384FS_PLL_96K         0x4e81  /* 36.864M */
#define AUDIO_384FS_PLL_96K_MUX     0x5
#define AUDIO_384FS_CLK_96K         0x5ef

#define AUDIO_384FS_PLL_88K         0x127f  /* 33.8688M */
#define AUDIO_384FS_PLL_88K_MUX     0x9
#define AUDIO_384FS_CLK_88K         0x5ef

#define AUDIO_384FS_PLL_48K         0x14c0  /* 18.432M */
#define AUDIO_384FS_PLL_48K_MUX     24
#define AUDIO_384FS_CLK_48K_AC3     0x5ed
#define AUDIO_384FS_CLK_48K         0x5ef

#define AUDIO_384FS_PLL_44K         0x16a3  /* 16.9344M */
#define AUDIO_384FS_PLL_44K_MUX     20
#define AUDIO_384FS_CLK_44K         0x5ef

#define AUDIO_384FS_PLL_32K         0x1480  /* 12.288M */
#define AUDIO_384FS_PLL_32K_MUX     24
#define AUDIO_384FS_CLK_32K         0x5ef

#define AUDIO_384FS_DAC_CFG         0x6

#define AUDIO_256FS_PLL_192K        0x50d5  /* 24.576M */
#define AUDIO_256FS_PLL_192K_MUX    12
#define AUDIO_256FS_CLK_192K        0x5c7

#define AUDIO_256FS_PLL_176K        0x108f  /* 22.5792M */
#define AUDIO_256FS_PLL_176K_MUX    18
#define AUDIO_256FS_CLK_176K        0x5c7

#define AUDIO_256FS_PLL_96K         0x50d5  /* 24.576M */
#define AUDIO_256FS_PLL_96K_MUX     12
#define AUDIO_256FS_CLK_96K         0x5db

#define AUDIO_256FS_PLL_88K         0x108f  /* 22.5792M */
#define AUDIO_256FS_PLL_88K_MUX     18
#define AUDIO_256FS_CLK_88K         0x5db

#define AUDIO_256FS_PLL_48K         0x1480  /* 12.288M */
#define AUDIO_256FS_PLL_48K_MUX     24
#define AUDIO_256FS_CLK_48K_AC3     0x5d9
#define AUDIO_256FS_CLK_48K         0x5db

#define AUDIO_256FS_PLL_44K         0x508f  /* 11.2896M */
#define AUDIO_256FS_PLL_44K_MUX     18
#define AUDIO_256FS_CLK_44K         0x5db

#define AUDIO_256FS_PLL_32K         0x508e  /* 8.192M */
#define AUDIO_256FS_PLL_32K_MUX     25
#define AUDIO_256FS_CLK_32K         0x5db
#define AUDIO_256FS_DAC_CFG         0x7

#define AUDIO_DAC_CLK_DIV           (0x4000>>12)

#elif defined(AML_APOLLO)
#define AUDIO_384FS_PLL_192K        0x2247  /* 36.864M */
#define AUDIO_384FS_PLL_192K_MUX    0xc
#define AUDIO_384FS_CLK_192K        0x5eb

#define AUDIO_384FS_PLL_176K        0x0545  /* 33.8688M */
#define AUDIO_384FS_PLL_176K_MUX    0xa
#define AUDIO_384FS_CLK_176K        0x5eb

#define AUDIO_384FS_PLL_96K         0x2247  /* 36.864M */
#define AUDIO_384FS_PLL_96K_MUX     0xc
#define AUDIO_384FS_CLK_96K         0x5ef

#define AUDIO_384FS_PLL_88K         0x0545  /* 33.8688M */
#define AUDIO_384FS_PLL_88K_MUX     0xa
#define AUDIO_384FS_CLK_88K         0x5ef

#define AUDIO_384FS_PLL_48K         0x2447  /* 18.432M */
#define AUDIO_384FS_PLL_48K_MUX     0xc
#define AUDIO_384FS_CLK_48K_AC3     0x5ed
#define AUDIO_384FS_CLK_48K         0x5ef

#define AUDIO_384FS_PLL_44K         0x0d6a  /* 16.9344M */
#define AUDIO_384FS_PLL_44K_MUX     0xc
#define AUDIO_384FS_CLK_44K         0x5ef

#define AUDIO_384FS_PLL_32K         0x2647  /* 12.288M */
#define AUDIO_384FS_PLL_32K_MUX     0xc
#define AUDIO_384FS_CLK_32K         0x5ef

#define AUDIO_384FS_DAC_CFG         0x6

#define AUDIO_256FS_PLL_192K        0x2347  /* 24.576M */
#define AUDIO_256FS_PLL_192K_MUX    0xc
#define AUDIO_256FS_CLK_192K        0x5c7

#define AUDIO_256FS_PLL_176K        0x052e  /* 22.5792M */
#define AUDIO_256FS_PLL_176K_MUX    0xa
#define AUDIO_256FS_CLK_176K        0x5c7

#define AUDIO_256FS_PLL_96K         0x2347  /* 24.576M */
#define AUDIO_256FS_PLL_96K_MUX     0xc
#define AUDIO_256FS_CLK_96K         0x5db

#define AUDIO_256FS_PLL_88K         0x052e  /* 22.5792M */
#define AUDIO_256FS_PLL_88K_MUX     0xa
#define AUDIO_256FS_CLK_88K         0x5db

#define AUDIO_256FS_PLL_48K         0x27df  /* 12.288M */
#define AUDIO_256FS_PLL_48K_MUX     0x22
#define AUDIO_256FS_CLK_48K_AC3     0x5d9
#define AUDIO_256FS_CLK_48K         0x5db

#define AUDIO_256FS_PLL_44K         0x298f  /* 11.2896M */
#define AUDIO_256FS_PLL_44K_MUX     0x12
#define AUDIO_256FS_CLK_44K         0x5db

#define AUDIO_256FS_PLL_32K         0x0fdf  /* 8.192M */
#define AUDIO_256FS_PLL_32K_MUX     0x30
#define AUDIO_256FS_CLK_32K         0x5db

#define AUDIO_256FS_DAC_CFG         0x7

#define AUDIO_DAC_CLK_DIV           (0x4000>>12)
#endif

typedef struct {
    unsigned short pll;
    unsigned short mux;
    unsigned short devisor;
} _aiu_clk_setting_t;

typedef struct {
    unsigned short chstat0_l;
    unsigned short chstat1_l;
    unsigned short chstat0_r;
    unsigned short chstat1_r;
} _aiu_958_channel_status_t;

typedef struct {
    /* audio clock */
    unsigned short clock;
    /* analog output */
    unsigned short i2s_mode;
    unsigned short i2s_dac_mode;
    unsigned short i2s_preemphsis;
    /* digital output */
    unsigned short i958_buf_start_addr;
    unsigned short i958_buf_blksize;
    unsigned short i958_int_flag;
    unsigned short i958_mode;
    unsigned short i958_sync_mode;
    unsigned short i958_preemphsis;
    unsigned short i958_copyright;
    unsigned short bpf;
    unsigned short brst;
    unsigned short length;
    unsigned short paddsize;
    _aiu_958_channel_status_t chan_status;
} audio_output_config_t;

typedef struct {
    unsigned short int_flag;
    unsigned short bpf;
    unsigned short brst;
    unsigned short length;
    unsigned short paddsize;
    _aiu_958_channel_status_t *chan_stat;
} _aiu_958_raw_setting_t;

#define AUDIO_CLK_256FS             0
#define AUDIO_CLK_384FS             1

#define AUDIO_CLK_FREQ_192  0
#define AUDIO_CLK_FREQ_1764 1
#define AUDIO_CLK_FREQ_96   2
#define AUDIO_CLK_FREQ_882  3
#define AUDIO_CLK_FREQ_48   4
#define AUDIO_CLK_FREQ_441  5
#define AUDIO_CLK_FREQ_32   6

#define AIU_958_MODE_RAW    0
#define AIU_958_MODE_PCM24  1
#define AIU_958_MODE_PCM16  2

#define AIU_I2S_MODE_2x16   0
#define AIU_I2S_MODE_2x24   1
#define AIU_I2S_MODE_8x24   2
#define AIU_I2S_MODE_2x32   3

#define AUDIO_ALGOUT_DAC_FORMAT_DSP             0
#define AUDIO_ALGOUT_DAC_FORMAT_LEFT_JUSTIFY    1

void audio_set_aiubuf(u32 addr, u32 size);
void audio_set_958outbuf(u32 addr, u32 size);
void audio_set_i2s_mode(u32 mode);
void audio_set_clk(unsigned freq, unsigned fs_config);
void audio_enable_ouput(int flag);
unsigned int read_i2s_rd_ptr(void);
void audio_i2s_unmute(void);
void audio_i2s_mute(void);
void audio_util_set_dac_format(unsigned format);
void audio_hw_set_958_mode(unsigned mode, _aiu_958_raw_setting_t * set);
unsigned int read_i2s_mute_swap_reg(void);
void audio_i2s_swap_left_right(unsigned int flag);

#endif
