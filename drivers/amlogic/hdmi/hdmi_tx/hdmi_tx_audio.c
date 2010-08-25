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

void hdmi_tx_setting_audio_packet(HDMI_TX_INFO_t *info)
{
}

void hdmitx_audio_enable(hdmitx_dev_t* hdmitx_device)
{
    if(hdmitx_device->HWOp.SetAudMode)
        hdmitx_device->HWOp.SetAudMode(hdmitx_device);

}


