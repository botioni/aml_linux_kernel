#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/clk.h>
#include <linux/timer.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/jack.h>

#include <asm/mach-types.h>
#include <mach/hardware.h>
#include <mach/gpio.h>

#include "aml_dai.h"
#include "aml_pcm.h"
#include "../codecs/rt5621.h"


#define HP_DET	1
#if HP_DET
static struct timer_list timer;
#endif

static int aml_m1_hw_params(struct snd_pcm_substream *substream,
                            struct snd_pcm_hw_params *params)
{
    struct snd_soc_pcm_runtime *rtd = substream->private_data;
    struct snd_soc_dai *codec_dai = rtd->dai->codec_dai;
    struct snd_soc_dai *cpu_dai = rtd->dai->cpu_dai;
    int ret;
    // TODO
    printk("***Entered %s:%s\n", __FILE__,__func__);

    ret = snd_soc_dai_set_fmt(codec_dai, SND_SOC_DAIFMT_I2S|SND_SOC_DAIFMT_NB_NF|SND_SOC_DAIFMT_CBS_CFS);
    if (ret<0)
        return ret;

    ret = snd_soc_dai_set_fmt(cpu_dai, SND_SOC_DAIFMT_I2S|SND_SOC_DAIFMT_NB_NF|SND_SOC_DAIFMT_CBS_CFS);
    if (ret<0)
        return ret;

    return 0;
}

static struct snd_soc_ops aml_m1_ops =
{
    .hw_params = aml_m1_hw_params,
};

static int aml_m1_set_bias_level(struct snd_soc_card *card,
                                 enum snd_soc_bias_level level)
{
    int ret = 0;
    // TODO
    printk("***Entered %s:%s: %d\n", __FILE__,__func__, level);
    switch (level)
    {
    case SND_SOC_BIAS_ON:
    case SND_SOC_BIAS_PREPARE:
        #if HP_DET		
		del_timer_sync(&timer);
		timer.expires = jiffies + HZ*1;
		del_timer(&timer);
		add_timer(&timer);
       #endif
        break;
    case SND_SOC_BIAS_OFF:
    case SND_SOC_BIAS_STANDBY:
        #if HP_DET		
		del_timer(&timer);
        #endif
        break;
    };

    return ret;
}

static const struct snd_soc_dapm_widget aml_m1_dapm_widgets[] =
{
    SND_SOC_DAPM_SPK("Ext Spk", NULL),
    SND_SOC_DAPM_HP("HP", NULL),
    SND_SOC_DAPM_MIC("MIC IN", NULL),
    SND_SOC_DAPM_MIC("HP MIC", NULL),
    SND_SOC_DAPM_LINE("FM IN", NULL),
};

static const struct snd_soc_dapm_route intercon[] =
{

    /* speaker connected to LINEOUT */
    {"Ext Spk", NULL, "LINEOUT1L"},
    {"Ext Spk", NULL, "LINEOUT1R"},
    /* mic is connected to Mic Jack, with WM8731 Mic Bias */
    {"HP", NULL, "HP_L"},
    {"HP", NULL, "HP_R"},
    {"LINPUT2", NULL, "Mic Bias"},
    {"Mic Bias", NULL, "MIC IN"},
    {"RINPUT2", NULL, "Mic Bias"},
    {"Mic Bias", NULL, "HP MIC"},
    {"LINPUT3", NULL, "FM IN"},
    {"RINPUT3", NULL, "FM IN"},
};

#if HP_DET

/* Hook switch */
static struct snd_soc_jack hp_jack;

static struct snd_soc_jack_pin hp_jack_pins[] = {
	{ .pin = "HP", .mask = SND_JACK_HEADSET },
};

static int hp_detect_flag = 0;
static spinlock_t lock;
static void rt5621_hp_detect_queue(struct work_struct*);
static struct rt5621_work_t
{
   unsigned long data;
   struct work_struct rt5621_workqueue;
}rt5621_work;

// use LED_CS1 as detect pin
#define PWM_TCNT        (600-1)
#define PWM_MAX_VAL    (420)
 
static unsigned int inner_cs_input_level()
{
	unsigned int level = 0;
    unsigned int cs_no = 0;
    //pin64 LED_CS0
    //CLEAR_CBUS_REG_MASK(PERIPHS_PIN_MUX_0, (1<<21));
    // Enable VBG_EN
    WRITE_CBUS_REG_BITS(PREG_AM_ANALOG_ADDR, 1, 0, 1);
    // pin mux 
    // wire            pm_gpioA_7_led_pwm          = pin_mux_reg0[22];
    WRITE_CBUS_REG(LED_PWM_REG0, 0);
    WRITE_CBUS_REG(LED_PWM_REG1, 0);
    WRITE_CBUS_REG(LED_PWM_REG2, 0);
    WRITE_CBUS_REG(LED_PWM_REG3, 0);
    WRITE_CBUS_REG(LED_PWM_REG4, 0);
    WRITE_CBUS_REG(LED_PWM_REG0, (0 << 31)    |       // disable the overall circuit
                            (0 << 30)           |       // 1:Closed Loop  0:Open Loop
                            (PWM_TCNT << 16)    |       // PWM total count
                            (0 << 13)           |       // Enable
                            (1 << 12)           |       // enable
                            (0 << 10)           |       // test
                            (7 << 7)            |       // CS0 REF, Voltage FeedBack: about 0.505V
                            (7 << 4)            |       // CS1 REF, Current FeedBack: about 0.505V
                            (0 << 0)                   // DIMCTL Analog dimmer
                      );
     WRITE_CBUS_REG(LED_PWM_REG1, (1 << 30)   |       // enable high frequency clock
                            (PWM_MAX_VAL << 16) |       // MAX PWM value
                            (0 << 0)                  // MIN PWM value
                      );
     WRITE_CBUS_REG(LED_PWM_REG2, (0 << 31)   |       // disable timeout test mode
                            (0 << 30)           |       // timeout based on the comparator output
                            (0 << 16)           |       // timeout = 10uS
                            (0 << 13)           |       // Select oscillator as the clock (just for grins)
                            (1 << 11)           |       // 1:Enable OverCurrent Portection  0:Disable
                            (3 << 8)            |       // Filter: shift every 3 ticks
                            (0 << 6)            |       // Filter: count 1uS ticks
                            (0 << 5)            |       // PWM polarity : negative
                            (0 << 4)            |       // comparator: negative, Different with NikeD3
                            (1 << 0)                    // +/- 1
                      );
    WRITE_CBUS_REG(LED_PWM_REG3, (1 << 16)    |    // Feedback down-sampling = PWM_freq/1 = PWM_freq
                            (1 << 14)           |    // enable to re-write MATCH_VAL
                            (210 << 0)              // preset PWM_duty = 50%
                      );
    WRITE_CBUS_REG(LED_PWM_REG4, (0 << 30)    |    // 1:Digital Dimmer  0:Analog Dimmer
                            (2 << 28)           |    // dimmer_timebase = 1uS
                            (1000 << 14)        |    // Digital dimmer_duty = 0%, the most darkness
                            (1000 << 0)             // dimmer_freq = 1KHz
                      );
    cs_no = READ_CBUS_REG(LED_PWM_REG3);
    
    if  (cs_no &(1<<14))
        level |= (1<<0);
    if  (cs_no &(1<<15))
        level |= (1<<4);

    return level;
}

static void rt5621_hp_detect_queue(struct work_struct* work)
{
	int level = inner_cs_input_level();
	struct rt5621_work_t* pwork = container_of(work,struct rt5621_work_t, rt5621_workqueue);
	int gpio_status = 0;
	struct snd_soc_codec* codec = (struct snd_soc_codec*)(pwork->data);
    //printk("level = %x, hp_detect_flag = %x\n", level, hp_detect_flag);
    if (level == 0x11 && hp_detect_flag != 0x11)
    {        
        //printk("Headphone pluged in\n");		
	    snd_soc_dapm_disable_pin(codec, "Ext Spk");
	    snd_soc_dapm_sync(codec);
	    // pull down the gpio to mute spk
	    //gpio_status = snd_soc_read(codec, RT5621_GPIO_OUTPUT_PIN_CTRL);
	    //gpio_status &= ~(1<<1);	    
	    //snd_soc_write(codec, RT5621_GPIO_OUTPUT_PIN_CTRL, gpio_status);
	    switch_audio(1);
        snd_soc_jack_report(&hp_jack, SND_JACK_HEADSET, SND_JACK_HEADSET);
        hp_detect_flag = level;
    }
    else if (level != hp_detect_flag)  // HDMI
    {
        //printk("Headphone unpluged\n");
	    snd_soc_dapm_enable_pin(codec, "Ext Spk");
	    snd_soc_dapm_sync(codec);
        snd_soc_jack_report(&hp_jack, 0, SND_JACK_HEADSET);
        hp_detect_flag = level;
	    //gpio_status = snd_soc_read(codec, RT5621_GPIO_OUTPUT_PIN_CTRL);	    
	    //gpio_status |= (1<<1);
	    //snd_soc_write(codec, RT5621_GPIO_OUTPUT_PIN_CTRL, gpio_status);
	    switch_audio(0);
    } 
}

static void rt5621_hp_detect_timer(unsigned long data)
{
    //printk("***Entered %s:%s\n", __FILE__,__func__);
	struct snd_soc_codec *codec = (struct snd_soc_codec*) data;
	rt5621_work.data = (unsigned long)codec;
	schedule_work(&rt5621_work.rt5621_workqueue);
	mod_timer(&timer, jiffies + HZ*1);
}

#endif

static int aml_m1_codec_init(struct snd_soc_codec *codec)
{
    struct snd_soc_card *card = codec->socdev->card;
    int err;

    printk("***Entered %s:%s\n", __FILE__,__func__);
    err = snd_soc_dapm_new_controls(codec, aml_m1_dapm_widgets, ARRAY_SIZE(aml_m1_dapm_widgets));
    if (err)
    {
	    dev_warn(card->dev, "Failed to register DAPM widgets\n");
		return 0;
	}
/*    err = snd_soc_dapm_add_routes(codec, intercon, ARRAY_SIZE(intercon));
    if (err)
    {
		dev_warn(card->dev, "Failed to setup dapm widgets routine\n");
		return 0;
	}*/
#if HP_DET
    hp_detect_flag = 0;
    err = snd_soc_jack_new(card, "hp_switch", SND_JACK_HEADSET, &hp_jack);
    if (err)
    {
        dev_warn(card->dev, "Failed to alloc resource for hook switch\n");
    }
    else
    {
        err = snd_soc_jack_add_pins(&hp_jack, ARRAY_SIZE(hp_jack_pins), hp_jack_pins);
        if(err)
        {
            dev_warn(card->dev, "Failed to setup hook hp jack pin\n");
        }
    }

    // create a timer to poll the HP IN status
    spin_lock_init(&lock);
    timer.function = &rt5621_hp_detect_timer;
    timer.data = (unsigned long)codec;
    timer.expires = jiffies + HZ*1;
    init_timer(&timer);
	INIT_WORK(&rt5621_work.rt5621_workqueue, rt5621_hp_detect_queue);
	printk("***Exit %s:%s\n", __FILE__,__func__);
	add_timer(&timer);
#endif

    snd_soc_dapm_nc_pin(codec,"LINPUT1");
    snd_soc_dapm_nc_pin(codec,"RINPUT1");

    snd_soc_dapm_enable_pin(codec, "Ext Spk");
    snd_soc_dapm_enable_pin(codec, "HP");
    snd_soc_dapm_enable_pin(codec, "MIC IN");
    snd_soc_dapm_enable_pin(codec, "HP MIC");
    snd_soc_dapm_enable_pin(codec, "FM IN");

    snd_soc_dapm_sync(codec);
    
    return 0;
}


static struct snd_soc_dai_link aml_m1_dai =
{
    .name = "AML-M1",
    .stream_name = "AML M1 PCM",
    .cpu_dai = &aml_dai[0],  //////
    .codec_dai = &rt5621_dai,
    .init = aml_m1_codec_init,
    .ops = &aml_m1_ops,
};

static struct snd_soc_card snd_soc_aml_m1 =
{
    .name = "AML-M1",
    .platform = &aml_soc_platform,
    .dai_link = &aml_m1_dai,
    .num_links = 1,
    .set_bias_level = aml_m1_set_bias_level,
};

//flove111810_S
static struct rt5621_setup_data aml_rt5621_setup =
{
    .i2c_address = 0x1a,
    .i2c_bus = 0,
};
//flove111810_E

static struct snd_soc_device aml_m1_snd_devdata =
{
    .card = &snd_soc_aml_m1,
    .codec_dev = &soc_codec_dev_rt5621,
    .codec_data = &aml_rt5621_setup,	//flove111810
};

static struct platform_device *aml_m1_snd_device;
static struct platform_device *aml_m1_platform_device;

static int aml_m1_audio_probe(struct platform_device *pdev)
{
    int ret;
    //pdev->dev.platform_data;
    // TODO
    printk("***Entered %s:%s\n", __FILE__,__func__);
    aml_m1_snd_device = platform_device_alloc("soc-audio", -1);
    if (!aml_m1_snd_device)
    {
        printk(KERN_ERR "ASoC: Platform device allocation failed\n");
        ret = -ENOMEM;
    }

    platform_set_drvdata(aml_m1_snd_device,&aml_m1_snd_devdata);
    aml_m1_snd_devdata.dev = &aml_m1_snd_device->dev;

    ret = platform_device_add(aml_m1_snd_device);
    if (ret)
    {
        printk(KERN_ERR "ASoC: Platform device allocation failed\n");
        goto error;
    }

    aml_m1_platform_device = platform_device_register_simple("aml_m1_codec",
                             -1, NULL, 0);
    return 0;
error:
    platform_device_put(aml_m1_snd_device);
    return ret;
}

static int aml_m1_audio_remove(struct platform_device *pdev)
{
    printk("***Entered %s:%s\n", __FILE__,__func__);
    platform_device_unregister(aml_m1_snd_device);
    return 0;
}

static struct platform_driver aml_m1_audio_driver =
{
    .probe  = aml_m1_audio_probe,
    .remove = aml_m1_audio_remove,
    .driver = {
        .name = "aml_m1_audio_rt5621",
        .owner = THIS_MODULE,
    },
};

static int __init aml_m1_init(void)
{
    return platform_driver_register(&aml_m1_audio_driver);
}

static void __exit aml_m1_exit(void)
{
    platform_driver_unregister(&aml_m1_audio_driver);
}

module_init(aml_m1_init);
module_exit(aml_m1_exit);

/* Module information */
MODULE_AUTHOR("AMLogic, Inc.");
MODULE_DESCRIPTION("ALSA SoC AML M1 AUDIO");
MODULE_LICENSE("GPL");