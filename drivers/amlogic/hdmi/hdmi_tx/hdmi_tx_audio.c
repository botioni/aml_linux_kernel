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
#include "hdmi_info_global.h"


void hdmi_tx_set_N_CTS(unsigned N_value, unsigned CTS)
{
#if 0
    unsigned hdmi_wr_data, temp_data, temp_data1, temp_data2;
    // Set CTS
    temp_data = CTS & 0xff;
    temp_data1 = (CTS & 0xff00) >> 8 ;
    temp_data2 = (CTS & 0xf0000) >> 16 ;

    ext_hdmi_wr_reg(TX_SYS0_ACR_CTS_0, temp_data);
    ext_hdmi_wr_reg(TX_SYS0_ACR_CTS_1, temp_data1);

    hdmi_wr_data  = ((0 << 7) | //  Force ACR
                    (0 << 6) | //    Force ACR Ready.
                    (temp_data2 << 0)); //	 [3:0] CTS
    ext_hdmi_wr_reg(TX_SYS0_ACR_CTS_2, hdmi_wr_data); // 0x01

    // Set N  (N is not measured, N must be configured so as to be a reference to clock_meter)
    temp_data = N_value & 0xff;
    temp_data1 = (N_value & 0xff00) >> 8 ;
    temp_data2 = (N_value & 0xf0000) >> 16 ;
    ext_hdmi_wr_reg(TX_SYS1_ACR_N_0, temp_data); // N[7:0]
    ext_hdmi_wr_reg(TX_SYS1_ACR_N_0, temp_data1); // N[7:0]
    hdmi_wr_data  = ((0xa << 4) | //[7:4] Meas Tolerance
                             (temp_data2 << 0)); //	 // [3:0] N[19:16]
    ext_hdmi_wr_reg(TX_SYS1_ACR_N_2, hdmi_wr_data); // 0xa0
#endif    

}

void hdmi_tx_setting_audio_packet(HDMI_TX_INFO_t *info)
{
	unsigned char checksum;
	unsigned char ct, cc, fs, ss, ca, lsv;
	ct = 0;   //audio code type: Refer to Stream Header
	 cc = 0;   //audio channel count: Refer to Stream Header
	ss = 0;   //sample size: Refer to Stream Header
	 fs = 0;   //sampling frequency: Refer to Stream Header
	ca = 0;   //Speaker Mapping: 2 channels(FR & FL) ,applyonlyto multi-channel(Le.,morethantwochannels)uncompressedaudio.
	lsv = 0;  //DTV Monitor how much the source device attenuated the audio during a down-mixing operation. 0db
#if 0
    ext_hdmi_wr_reg(TX_PKT_REG_AUDIO_INFO_BASE_ADDR+0x1B, AUDIO_INFOFRAMES_VERSION); // PB27: Rsrv.
    ext_hdmi_wr_reg(TX_PKT_REG_AUDIO_INFO_BASE_ADDR+0x1C, TYPE_AUDIO_INFOFRAMES); // HB0: Packet Type = 0x84
    ext_hdmi_wr_reg(TX_PKT_REG_AUDIO_INFO_BASE_ADDR+0x1E, AUDIO_INFOFRAMES_LENGTH); // HB2: Payload length in bytes
    ext_hdmi_wr_reg(TX_PKT_REG_AUDIO_INFO_BASE_ADDR+0x1F, 0x80); // Enable audio info frame
    checksum = 0x100 - (AUDIO_INFOFRAMES_VERSION + TYPE_AUDIO_INFOFRAMES + AUDIO_INFOFRAMES_LENGTH + 0x80 + ct + cc + ss + sf + ca + lsv) & 0xff;

    ext_hdmi_wr_reg(TX_PKT_REG_AUDIO_INFO_BASE_ADDR,      checksum); // PB0: Checksum

    if (info->audio_info.format == AF_SPDIF) {
        // TX Channel Status
        //0xB0 - 00000000b;     //0xC8 - 00000000b;
        //0xB1 - 00000000b;     //0xC9 - 00000000b;
        //0xB2 - 00011000b;     //0xCA - 00101000b;
        //0xB3 - 00000000b;     //0xCB - 00000000b;
        //0xB4 - 11111011b;     //0xCC - 11111011b;
        ext_hdmi_wr_reg(TX_IEC60958_SUB1_OFFSET+0x00, 0x00);
        ext_hdmi_wr_reg(TX_IEC60958_SUB1_OFFSET+0x01, 0x00);
        ext_hdmi_wr_reg(TX_IEC60958_SUB1_OFFSET+0x02, 0x18);
        ext_hdmi_wr_reg(TX_IEC60958_SUB1_OFFSET+0x03, 0x00);
        ext_hdmi_wr_reg(TX_IEC60958_SUB1_OFFSET+0x04, 0xfb);
        ext_hdmi_wr_reg(TX_IEC60958_SUB2_OFFSET+0x00, 0x00);
        ext_hdmi_wr_reg(TX_IEC60958_SUB2_OFFSET+0x01, 0x00);
        ext_hdmi_wr_reg(TX_IEC60958_SUB2_OFFSET+0x02, 0x28);
         ext_hdmi_wr_reg(TX_IEC60958_SUB2_OFFSET+0x03, 0x00);
        ext_hdmi_wr_reg(TX_IEC60958_SUB2_OFFSET+0x04, 0xfb);
    }
#endif    
}


void hdmi_tx_audio_enable(HDMI_TX_INFO_t *info)
{


}


void hdmi_tx_audio_disable(HDMI_TX_INFO_t *info)
{


}

