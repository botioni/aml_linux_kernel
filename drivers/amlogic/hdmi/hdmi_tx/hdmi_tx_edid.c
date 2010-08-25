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
#include <asm/uaccess.h>

#include "hdmi_tx_module.h"
#include "hdmi_info_global.h"


#define HDMI_EDID_BLOCK_TYPE_RESERVED     0
#define HDMI_EDID_BLOCK_TYPE_AUDIO        1
#define HDMI_EDID_BLOCK_TYPE_VIDEO        2
#define HDMI_EDID_BLOCK_TYPE_VENDER       3
#define HDMI_EDID_BLOCK_TYPE_SPEAKER      4
#define HDMI_EDID_BLOCK_TYPE_VESA         5
#define HDMI_EDID_BLOCK_TYPE_EXTENDED_TAG 7

static int hdmitx_edid_block_parse(hdmitx_dev_t* hdmitx_device, unsigned char *BlockBuf)
{
    unsigned char offset,End ;
    unsigned char count ;
    unsigned char tag ;
    int i ;
    rx_cap_t* pRXCap = &(hdmitx_device->RXCap);

    if( BlockBuf[0] != 0x02 || BlockBuf[1] != 0x03 ) return -1 ; // not a CEA BLOCK.
    End = BlockBuf[2]  ; // CEA description.
    pRXCap->native_Mode = BlockBuf[3] ;

	  pRXCap->VIC_count = 0 ;
    pRXCap->native_VIC = 0xff ;
    
    for( offset = 4 ; offset < End ; ){
        tag = BlockBuf[offset] >> 5 ;
        count = BlockBuf[offset] & 0x1f ;
        switch( tag ){
            case HDMI_EDID_BLOCK_TYPE_AUDIO: 
                pRXCap->AUD_count = count/3 ;
                offset++ ;
                for( i = 0 ; i < pRXCap->AUD_count ; i++, offset+=3 )
                {
                    pRXCap->RxAudioCap[i].audio_format_code = (BlockBuf[offset]>>3)&0xf;
                    pRXCap->RxAudioCap[i].channel_num_max = BlockBuf[offset]&0x7;
                    pRXCap->RxAudioCap[i].freq_cc = BlockBuf[offset+1]&0x7f;
                    pRXCap->RxAudioCap[i].cc3 = BlockBuf[offset+2]&0x7;
                }
                break ;
            
            case HDMI_EDID_BLOCK_TYPE_VIDEO: 
                offset ++ ;
                for( i = 0 ; i < count ; i++, offset++ )
                {
                    unsigned char VIC ;
                    VIC = BlockBuf[offset] & (~0x80) ;
                    pRXCap->VIC[pRXCap->VIC_count] = VIC ;
                    if( BlockBuf[offset] & 0x80 ){
                        pRXCap->native_VIC = VIC;
                    }
                    pRXCap->VIC_count++ ;
                }
                break ;
            
            case HDMI_EDID_BLOCK_TYPE_VENDER: 
                offset ++ ;
                pRXCap->IEEEOUI = (unsigned long)BlockBuf[offset+2] ;
                pRXCap->IEEEOUI <<= 8 ;
                pRXCap->IEEEOUI += (unsigned long)BlockBuf[offset+1] ;
                pRXCap->IEEEOUI <<= 8 ;
                pRXCap->IEEEOUI += (unsigned long)BlockBuf[offset] ;
                offset += count ; // ignore the remaind.
                break ;
            
            case HDMI_EDID_BLOCK_TYPE_SPEAKER: 
                offset ++ ;
                pRXCap->RxSpeakerAllocation = BlockBuf[offset] ;
                offset += 3 ;
                break ;

            case HDMI_EDID_BLOCK_TYPE_VESA: 
                offset += count+1 ;
                break ;

            case HDMI_EDID_BLOCK_TYPE_EXTENDED_TAG: 
                offset += count+1 ; //ignore
                break ;

            default:
                offset += count+1 ; // ignore
        }
    }
    return 0 ;
}



int hdmitx_edid_parse(hdmitx_dev_t* hdmitx_device)
{
    unsigned char CheckSum ;
    unsigned char BlockCount ;
    unsigned char* EDID_buf = hdmitx_device->EDID_buf;
    int i ;
    for( i = 0, CheckSum = 0 ; i < 128 ; i++ )
    {
        CheckSum += hdmitx_device->EDID_buf[i] ; CheckSum &= 0xFF ;
    }
    
    if( CheckSum != 0 || EDID_buf[0] != 0x00 || EDID_buf[1] != 0xFF ||EDID_buf[2] != 0xFF
        || EDID_buf[3] != 0xFF || EDID_buf[4] != 0xFF || EDID_buf[5] != 0xFF 
        || EDID_buf[6] != 0xFF || EDID_buf[7] != 0x00 )
    {
        return -1 ;
    }
	
    BlockCount = EDID_buf[0x7E] ;

    if( BlockCount == 0 ){
        return 0 ; // do nothing.
    }
    else if ( BlockCount > 4 )
    {
        BlockCount = 4 ;
    }
        	
    for( i = 1 ; i <= BlockCount ; i++ )
    {
        if( EDID_buf[i*128+0] == 0x2 && EDID_buf[i*128+1] == 0x3 )
        {
            if(hdmitx_edid_block_parse(hdmitx_device, &(EDID_buf[i*128]))>=0){
                if(hdmitx_device->RXCap.IEEEOUI==0x0c03){
                    break;
                }
            }
        }
    }
    return 0;

}

static struct{
    const char* disp_mode;
    HDMI_Video_Codes_t VIC;
}dispmode_VIC_tab[]=
{
    {"480i", HDMI_480i60}, 
    {"480i", HDMI_480i60_16x9},
    {"480p", HDMI_480p60},
    {"480p", HDMI_480p60_16x9},
    {"720p", HDMI_720p60},
    {"1080i", HDMI_1080i60},
    {"1080p", HDMI_1080p60},
};    

HDMI_Video_Codes_t hdmitx_edid_get_VIC(hdmitx_dev_t* hdmitx_device, char* disp_mode, char force_flag)
{
    rx_cap_t* pRXCap = &(hdmitx_device->RXCap);
	  int  i,j,count=ARRAY_SIZE(dispmode_VIC_tab);
	  HDMI_Video_Codes_t vic=HDMI_Unkown;
    for(i=0;i<count;i++){
        if(strncmp(disp_mode, dispmode_VIC_tab[i].disp_mode, strlen(dispmode_VIC_tab[i].disp_mode))==0){
            if(force_flag){
                vic = dispmode_VIC_tab[i].VIC;
                break;
            }
            else{
                for( j = 0 ; j < pRXCap->VIC_count ; j++ ){
                    if(pRXCap->VIC[j]==dispmode_VIC_tab[i].VIC)
                        break;    
                }
                if(j<pRXCap->VIC_count){
                    vic = dispmode_VIC_tab[i].VIC;
                    break;        
                }
            }
        }
    }    
    return vic;
}    

void hdmitx_edid_clear(hdmitx_dev_t* hdmitx_device)
{
    rx_cap_t* pRXCap = &(hdmitx_device->RXCap);
    pRXCap->VIC_count = 0;
    pRXCap->AUD_count = 0;
    pRXCap->IEEEOUI = 0;
    pRXCap->native_Mode = 0;
    pRXCap->native_VIC = 0xff;
    pRXCap->RxSpeakerAllocation = 0;
}

int hdmitx_edid_dump(hdmitx_dev_t* hdmitx_device, char* buffer, int buffer_len)
{
    int i,pos=0;
    rx_cap_t* pRXCap = &(hdmitx_device->RXCap);
    pos+=snprintf(buffer+pos, buffer_len-pos, "native Mode %x, VIC (native %d):\r\n",
        pRXCap->native_Mode, pRXCap->native_VIC);
    for( i = 0 ; i < pRXCap->VIC_count ; i++ )
    {
        pos+=snprintf(buffer+pos, buffer_len-pos,"%d ", pRXCap->VIC[i]);
    }
    pos+=snprintf(buffer+pos, buffer_len-pos,"\r\n");
    pos+=snprintf(buffer+pos, buffer_len-pos, "Audio {format, channel, freq, cce}\r\n");
    for( i =0; i< pRXCap->AUD_count; i++)
    {
        pos+=snprintf(buffer+pos, buffer_len-pos, "{%d, %d, %x, %x}\r\n", pRXCap->RxAudioCap[i].audio_format_code,
            pRXCap->RxAudioCap[i].channel_num_max, pRXCap->RxAudioCap[i].freq_cc, pRXCap->RxAudioCap[i].cc3);
    }
    pos+=snprintf(buffer+pos,buffer_len-pos,"Speaker Allocation: %x\r\n", pRXCap->RxSpeakerAllocation);
    pos+=snprintf(buffer+pos,buffer_len-pos,"Vendor: %x\r\n", pRXCap->IEEEOUI);
    return pos;        
}    

