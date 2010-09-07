#include <linux/version.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/mm.h>
#include <linux/major.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/cdev.h>
#include "hdmi_tx_module.h"
#include "hdmi_info_global.h"

void hdmi_tx_set_N_CTS(unsigned N_value, unsigned CTS)
{
}

static void hdmi_tx_construct_aud_packet(Hdmi_tx_audio_para_t* audio_param, char* AUD_DB, unsigned char* CHAN_STAT_BUF)
{
    if(AUD_DB){
        AUD_DB[0] = (audio_param->type<<4)|(audio_param->channel_num-1) ; 
        AUD_DB[1] = (audio_param->sample_rate<<2)|audio_param->sample_size;
        AUD_DB[4] = 0; //CA, 2 channel
        AUD_DB[5] = 0;//DM_INH<<7|LSV<<3
    }
    if(CHAN_STAT_BUF){
        
    }
}

int hdmitx_set_audio(hdmitx_dev_t* hdmitx_device, Hdmi_tx_audio_para_t* audio_param)
{
    int i,ret=-1;
    unsigned char AUD_DB[32];
    unsigned char CHAN_STAT_BUF[128];
    for(i=0;i<32;i++) AUD_DB[i]=0;
    for(i=0;i<128;i++) CHAN_STAT_BUF[i]=0;
    if(hdmitx_device->HWOp.SetAudMode(hdmitx_device, audio_param)>=0){
        hdmi_tx_construct_aud_packet(audio_param, AUD_DB, CHAN_STAT_BUF);
    
        hdmitx_device->HWOp.SetAudioInfoFrame(AUD_DB, CHAN_STAT_BUF);
        ret = 0;
    }
    return ret;
}



