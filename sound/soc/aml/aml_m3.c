#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/clk.h>
#include <linux/timer.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/workqueue.h>

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
#include "aml_m3_codec.h"

#define HP_DET 1


#if HP_DET
static struct timer_list timer;
static int hp_detect_flag = 0;
void mute_spk(struct snd_soc_codec* codec, int flag);
extern int aml_m3_is_hp_pluged(void);
#endif

static int aml_m3_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
		struct snd_soc_pcm_runtime *rtd = substream->private_data;
		struct snd_soc_dai *codec_dai = rtd->dai->codec_dai;
		struct snd_soc_dai *cpu_dai = rtd->dai->cpu_dai;
		int ret;
		// TODO
#ifdef _AML_M3_HW_DEBUG_
printk("***Entered %s:%s\n", __FILE__,__func__);
#endif
		
		ret = snd_soc_dai_set_fmt(codec_dai, SND_SOC_DAIFMT_I2S|SND_SOC_DAIFMT_NB_NF|SND_SOC_DAIFMT_CBS_CFS);
		if(ret<0)
			return ret;
			
		ret = snd_soc_dai_set_fmt(cpu_dai, SND_SOC_DAIFMT_I2S|SND_SOC_DAIFMT_NB_NF|SND_SOC_DAIFMT_CBS_CFS);
		if(ret<0)
			return ret;
		
	return 0;
}
	
static struct snd_soc_ops aml_m3_ops = {
	.hw_params = aml_m3_hw_params,
};

static int aml_m3_set_bias_level(struct snd_soc_card *card,
					enum snd_soc_bias_level level)
{
    int ret = 0;
    struct snd_soc_codec *codec = card->codec;
    // TODO

#ifdef _AML_M3_HW_DEBUG_
printk("***Entered %s:%s: %d\n", __FILE__,__func__, level);
#endif

    switch (level) {
    case SND_SOC_BIAS_ON:
    case SND_SOC_BIAS_PREPARE:
#if HP_DET
        del_timer_sync(&timer);
        timer.expires = jiffies + HZ*5;
        del_timer(&timer);
        add_timer(&timer);
        hp_detect_flag = 0xf0000000;
#endif
        break;
    case SND_SOC_BIAS_OFF:
    case SND_SOC_BIAS_STANDBY:
#if HP_DET
        del_timer(&timer);
        hp_detect_flag = 0xf0000000;        
        mute_spk(codec,1);
#endif
        break;
    };

    return ret;
}

static const struct snd_soc_dapm_widget aml_m3_dapm_widgets[] = {
    SND_SOC_DAPM_SPK("Ext Spk", NULL),
    SND_SOC_DAPM_HP("HP", NULL),
    SND_SOC_DAPM_MIC("MIC IN", NULL),
};

static const struct snd_soc_dapm_route intercon[] = {

	{"Ext Spk", NULL, "LINEOUTL"},
    {"Ext Spk", NULL, "LINEOUTR"},
    {"HP", NULL, "HP_L"},
    {"HP", NULL, "HP_R"},
	//{"LINEINL", "MICBIAS", "MIC IN"},
	//{"LINEINR", "MICBIAS", "MIC IN"},
		
	{"MICBIAS", NULL, "MIC IN"},
	{"LINEINR", NULL, "MICBIAS"},
	{"LINEINL", NULL, "MICBIAS"},
};

#if HP_DET

/* Hook switch */

static spinlock_t lock;
static void aml_m3_hp_detect_queue(struct work_struct*);
static struct aml_m3_work_t{
	unsigned long data;
	struct work_struct aml_m3_workqueue;
}aml_m3_work;


void mute_spk(struct snd_soc_codec* codec, int flag)
{
#ifdef _AML_M3_HW_DEBUG_
	printk("***Entered %s:%s\n", __FILE__,__func__);
#endif
//    int gpio_status = 0;
    if(flag){
//gpio_status = READ_CBUS_REG(PREG_PAD_GPIO2_EN_N);
//printk("$$$$gpio_status1=%#x\n",gpio_status);
		set_gpio_val(GPIOC_bank_bit0_15(4), GPIOC_bit_bit0_15(4), 0);	 // mute speak
		set_gpio_mode(GPIOC_bank_bit0_15(4), GPIOC_bit_bit0_15(4), GPIO_OUTPUT_MODE);
	}else{
//gpio_status = READ_CBUS_REG(PREG_PAD_GPIO2_EN_N);
//printk("$$$$gpio_status2=%#x\n",gpio_status);
		set_gpio_val(GPIOC_bank_bit0_15(4), GPIOC_bit_bit0_15(4), 1);	 // unmute speak
		set_gpio_mode(GPIOC_bank_bit0_15(4), GPIOC_bit_bit0_15(4), GPIO_OUTPUT_MODE);
	}
}
/////
/*
struct aml_m3_platform_data {
    int (*is_hp_pluged)(void);
};


static struct aml_m3_platform_data aml_m3_pdata = {
    .is_hp_pluged = &aml_m3_is_hp_pluged,
};
*/
///

static void aml_m3_hp_detect_queue(struct work_struct* work)
{
	//int flag_changed = 0;
	//static int i = 1;   //save the last status of hp_detect_flag
	struct aml_m3_work_t* pwork = container_of(work,struct aml_m3_work_t, aml_m3_workqueue);
    struct snd_soc_codec* codec = (struct snd_soc_codec*)(pwork->data);
	hp_detect_flag = aml_m3_is_hp_pluged();   //read the HP_DET pin
	//printk("***Entered %s:%s\n", __FILE__,__func__);
//	if (i != hp_detect_flag)
//		flag_changed = 1;
	if(/*flag_changed &&*/ hp_detect_flag){ // HP
        //printk("Headphone pluged in\n");
		//snd_soc_dapm_disable_pin(codec, "Ext Spk");
        //snd_soc_dapm_enable_pin(codec, "HP");
		//snd_soc_dapm_sync(codec);
        mute_spk(codec, 1);
    }else if(/*flag_changed && */!hp_detect_flag){ 
        //printk("Headphone unpluged\n");
		//snd_soc_dapm_enable_pin(codec, "Ext Spk");
        //snd_soc_dapm_disable_pin(codec, "HP");
		//snd_soc_dapm_sync(codec);
        mute_spk(codec, 0);
	}
	//i = hp_detect_flag;
}

static void aml_m3_hp_detect_timer(unsigned long data)
{
    struct snd_soc_codec *codec = (struct snd_soc_codec*) data;
    aml_m3_work.data = (unsigned long)codec;
    schedule_work(&aml_m3_work.aml_m3_workqueue);
    mod_timer(&timer, jiffies + HZ*1);
}

#endif

static int aml_m3_codec_init(struct snd_soc_codec *codec)
{
    struct snd_soc_card *card = codec->socdev->card;
	
	printk("***Entered %s:%s:\n", __FILE__,__func__);

    int err;
    //Add board specific DAPM widgets and routes
    err = snd_soc_dapm_new_controls(codec, aml_m3_dapm_widgets, ARRAY_SIZE(aml_m3_dapm_widgets));
    if(err){
        dev_warn(card->dev, "Failed to register DAPM widgets\n");
        return 0;
    }

    err = snd_soc_dapm_add_routes(codec, intercon,
        ARRAY_SIZE(intercon));
    if(err){
        dev_warn(card->dev, "Failed to setup dapm widgets routine\n");
        return 0;
    }

#if HP_DET
        hp_detect_flag = 1; // If is_hp_pluged function is not registered in bsp, set speaker as default.

    /*err = snd_soc_jack_new(card, "hp_switch",
        SND_JACK_HEADSET, &hp_jack);
    if(err){
        dev_warn(card->dev, "Failed to alloc resource for hook switch\n");
    }else{
        err = snd_soc_jack_add_pins(&hp_jack, ARRAY_SIZE(hp_jack_pins), hp_jack_pins);
        if(err){
            dev_warn(card->dev, "Failed to setup hook hp jack pin\n");
        }
    }
	*/
    // create a timer to poll the HP IN status
    spin_lock_init(&lock);
    timer.function = &aml_m3_hp_detect_timer;
    timer.data = (unsigned long)codec;
    timer.expires = jiffies + HZ*10;
    init_timer(&timer);
    INIT_WORK(&aml_m3_work.aml_m3_workqueue, aml_m3_hp_detect_queue);
#endif

    /*snd_soc_dapm_nc_pin(codec,"LINPUT1");
    snd_soc_dapm_nc_pin(codec,"RINPUT1");

    snd_soc_dapm_enable_pin(codec, "Ext Spk");
    snd_soc_dapm_disable_pin(codec, "HP");
    snd_soc_dapm_enable_pin(codec, "MIC IN");
    snd_soc_dapm_disable_pin(codec, "HP MIC");
    snd_soc_dapm_disable_pin(codec, "FM IN");

    snd_soc_dapm_sync(codec);
	*/
    return 0;
}


static struct snd_soc_dai_link aml_m3_dai = {
	.name = "AML-M3",
	.stream_name = "AML M3 PCM",
	.cpu_dai = &aml_dai[0],  //////
	.codec_dai = &aml_m3_codec_dai,
	.init = aml_m3_codec_init,
	.ops = &aml_m3_ops,
	.rate = 44100,
};

static struct snd_soc_card snd_soc_aml_m3 = {
	.name = "AML-M3",
	.platform = &aml_soc_platform,
	.dai_link = &aml_m3_dai,
	.num_links = 1,
	.set_bias_level = aml_m3_set_bias_level,
};

static struct snd_soc_device aml_m3_snd_devdata = {
	.card = &snd_soc_aml_m3,
	.codec_dev = &soc_codec_dev_aml_m3,
};

static struct platform_device *aml_m3_snd_device;
static struct platform_device *aml_m3_platform_device;

static int aml_m3_audio_probe(struct platform_device *pdev)
{
		int ret;
		//pdev->dev.platform_data;
		// TODO
printk("***Entered %s:%s\n", __FILE__,__func__);
		aml_m3_snd_device = platform_device_alloc("soc-audio", -1);
		if (!aml_m3_snd_device) {
			printk(KERN_ERR "ASoC: Platform device allocation failed\n");
			ret = -ENOMEM;
		}
	
		platform_set_drvdata(aml_m3_snd_device,&aml_m3_snd_devdata);
		aml_m3_snd_devdata.dev = &aml_m3_snd_device->dev;
	
		ret = platform_device_add(aml_m3_snd_device);
		if (ret) {
			printk(KERN_ERR "ASoC: Platform device allocation failed\n");
			goto error;
		}
		
		aml_m3_platform_device = platform_device_register_simple("aml_m3_codec",
								-1, NULL, 0);
		return 0;							
error:								
		platform_device_put(aml_m3_snd_device);								
		return ret;
}

static int aml_m3_audio_remove(struct platform_device *pdev)
{
printk("***Entered %s:%s\n", __FILE__,__func__);

#if HP_DET
    del_timer_sync(&timer);
    
#endif
    platform_device_unregister(aml_m3_snd_device);
    return 0;
}

static struct platform_driver aml_m3_audio_driver = {
	.probe  = aml_m3_audio_probe,
	.remove = aml_m3_audio_remove,
	.driver = {
		.name = "aml_m3_audio",
		.owner = THIS_MODULE,
	},
};

static int __init aml_m3_init(void)
{
		return platform_driver_register(&aml_m3_audio_driver);
}

static void __exit aml_m3_exit(void)
{
		platform_driver_unregister(&aml_m3_audio_driver);
}

module_init(aml_m3_init);
module_exit(aml_m3_exit);

/* Module information */
MODULE_AUTHOR("AMLogic, Inc.");
MODULE_DESCRIPTION("ALSA SoC AML M3 AUDIO");
MODULE_LICENSE("GPL");
