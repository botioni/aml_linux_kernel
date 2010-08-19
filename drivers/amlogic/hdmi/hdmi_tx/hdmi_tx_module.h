#ifndef _HDMI_TX_MODULE_H
#define _HDMI_TX_MODULE_H
#include "hdmi_info_global.h"
/*****************************
*    hdmitx attr management 
******************************/

#define  SHOW_INFO(name)      \
	{return snprintf(buf,40, "%s\n", name);}  	

#define  STORE_INFO(name)\
	{	snprintf(name,40,"%s",buf) ;\
	}			

#define    SET_HDMI_CLASS_ATTR(name,op)    \
static  char    name[40] ;				  \
static ssize_t aml_hdmi_attr_##name##_show(struct class  * cla, struct class_attribute *attr, char *buf)   \
{  											\
	SHOW_INFO(name)  	\
} 											\
static ssize_t  aml_hdmi_attr_##name##_store(struct class *cla,  struct class_attribute *attr, \
			    const char *buf, size_t count)    \
{\
	STORE_INFO(name)   						\
	op(name) ;						\
	return strnlen(buf, count);				\
}											\
struct  class_attribute  class_hdmi_attr_##name =  \
__ATTR(name, S_IRUGO|S_IWUSR, aml_hdmi_attr_##name##_show, aml_hdmi_attr_##name##_store) ; 

/************************************
*    hdmitx protocol level interface
*************************************/
extern void hdmitx_init_parameters(HDMI_TX_INFO_t *info);

extern void hdmitx_set_display(HDMI_Video_Codes_t VideoCode);


/************************************
*    hdmitx hardware level interface
*************************************/
extern void HDMITX_HW_SetAVI(unsigned char* AVI_DB, unsigned char* AVI_HB);

extern void HDMITX_HW_SetAudioInfoFrame(unsigned char bEnable, unsigned char* AUD_DB);

extern int HDMITX_HW_GetEDIDData(int block, unsigned char* buf);

extern int HDMITX_HW_SetMode(Hdmi_tx_video_para_t *param);

extern void HDMITX_HW_SetupIRQ(void);

extern void HDMITX_HW_Init(void);

#endif
