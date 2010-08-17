
#ifndef DSP_CONTROL_HEADER
#define DSP_CONTROL_HEADER
#include <asm/cacheflush.h>
#include "audiodsp_module.h"
#include "dsp_microcode.h"
#include <asm/system.h>
#include <linux/dma-mapping.h>

void halt_dsp( struct audiodsp_priv *priv);
void reset_dsp( struct audiodsp_priv *priv);
 int dsp_start( struct audiodsp_priv *priv, struct auidodsp_microcode *mcode);
 int dsp_stop( struct audiodsp_priv *priv);

// #define DSP_RD(reg)	({static dma_addr_t addr_map;addr_map = dma_map_single(NULL,(void*)reg,4,DMA_FROM_DEVICE); 
 //   (*((unsigned long *)addr_map));});

//#define DSP_WD(reg,val)	({(*((unsigned long *)(reg)))=val;})
#endif

