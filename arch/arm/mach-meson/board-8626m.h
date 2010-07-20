#ifndef __BOARD_8626M_H
#define __BOARD_8626M_H

#include <asm/page.h>

#define PHYS_MEM_START		(0x80000000)
#define PHYS_MEM_SIZE		(256*1024*1024)
#define PHYS_MEM_END		(PHYS_MEM_START + PHYS_MEM_SIZE -1 )

/******** CODEC memory setting ************************/
//	Codec need 16M for 1080p decode
//	4M for sd decode;
#define ALIGN_MSK			((1<<15)-1)
#define U_ALIGN(x)			((x+ALIGN_MSK)&(~ALIGN_MSK))
#define D_ALIGN(x)			((x)&(~ALIGN_MSK))

/******** Frame buffer memory configuration ***********/
#define OSD_480_PIX			(640*480)
#define OSD_576_PIX			(768*576)
#define OSD_720_PIX			(1280*720)
#define OSD_1080_PIX		(1920*1080)
#define B16BpP	(2)
#define B32BpP	(4)
#define DOUBLE_BUFFER	(2)

#define OSD1_MAX_MEM		U_ALIGN(OSD_720_PIX*B32BpP)
#define OSD2_MAX_MEM		U_ALIGN(OSD_720_PIX*B32BpP)

/******** Reserved memory configuration ***************/
#define OSD2_ADDR_END		(PHYS_MEM_END - 1)
#define OSD2_ADDR_START		D_ALIGN(OSD2_ADDR_END + 1 - OSD2_MAX_MEM)
#define OSD1_ADDR_END		(OSD2_ADDR_START - 1)
#define OSD1_ADDR_START		D_ALIGN(OSD1_ADDR_END + 1 - OSD1_MAX_MEM)

#if defined(CONFIG_AM_VDEC_H264)
#define CODEC_MEM_SIZE		U_ALIGN(32*1024*1024)
#else
#define CODEC_MEM_SIZE		U_ALIGN(16*1024*1024)
#endif
#define CODEC_ADDR_END		(OSD1_ADDR_START - 1)
#define CODEC_ADDR_START	D_ALIGN(CODEC_ADDR_END + 1 - CODEC_MEM_SIZE)

#endif
