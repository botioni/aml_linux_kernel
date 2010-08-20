#ifndef _HDMI_TX_MODULE_H
#define _HDMI_TX_MODULE_H
#include "hdmi_info_global.h"
/*****************************
*    hdmitx attr management 
******************************/

/************************************
*    hdmitx device structure
*************************************/
#define VIC_MAX_NUM 60
#define AUD_MAX_NUM 60
typedef struct
{
    unsigned char audio_format_code;
    unsigned char channel_num_max;
    unsigned char freq_cc;        
    unsigned char cc3;
} rx_audio_cap_t;

typedef struct rx_cap_
{
    unsigned char native_Mode;
    /*video*/
    unsigned char VIC[VIC_MAX_NUM];
    unsigned char VIC_count;
    unsigned char native_VIC;
    /*audio*/
    rx_audio_cap_t RxAudioCap[AUD_MAX_NUM];
    unsigned char AUD_count;
    unsigned char RxSpeakerAllocation;
    /*vendor*/    
    unsigned long IEEEOUI;
}rx_cap_t;


#define EDID_MAX_BLOCK  4
typedef struct hdmi_tx_dev_s {
    struct cdev cdev;             /* The cdev structure */

    struct proc_dir_entry *proc_file;
    struct task_struct *task;
    struct {
        void (*SetAVI)(unsigned char* AVI_DB, unsigned char* AVI_HB);
        void (*SetAudioInfoFrame)(unsigned char bEnable, unsigned char* AUD_DB);
        unsigned char (*GetEDIDData)(struct hdmi_tx_dev_s* hdmitx_device);
        int (*SetMode)(Hdmi_tx_video_para_t *param);
        void (*SetupIRQ)(struct hdmi_tx_dev_s* hdmitx_device);
    }HWOp;
    
    //wait_queue_head_t   wait_queue;            /* wait queues */
    /*EDID*/
    unsigned char EDID_buf[EDID_MAX_BLOCK*128];    
    rx_cap_t RXCap;
    /*status*/
    unsigned char cur_VIC;
    /**/
    unsigned char hpd_event; /* 1, plugin; 2, plugout */
    
}hdmitx_dev_t;

/************************************
*    hdmitx protocol level interface
*************************************/
extern void hdmitx_init_parameters(HDMI_TX_INFO_t *info);

extern int hdmitx_edid_parse(hdmitx_dev_t* hdmitx_device);

HDMI_Video_Codes_t hdmitx_edid_get_VIC(hdmitx_dev_t* hdmitx_device,char* disp_mode);

extern int hdmitx_edid_dump(hdmitx_dev_t* hdmitx_device, char* buffer, int buffer_len);

extern void hdmitx_edid_clear(hdmitx_dev_t* hdmitx_device);

extern int hdmitx_set_display(hdmitx_dev_t* hdmitx_device, HDMI_Video_Codes_t VideoCode);

/************************************
*    hdmitx hardware level interface
*************************************/
extern void HDMITX_M1A_Init(hdmitx_dev_t* hdmitx_device);

extern void HDMITX_M1B_Init(hdmitx_dev_t* hdmitx_device);

#endif
