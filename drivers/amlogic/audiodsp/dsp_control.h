
#ifndef DSP_CONTROL_HEADER
#define DSP_CONTROL_HEADER
#include <asm/cacheflush.h>
#include "audiodsp_module.h"
#include "dsp_microcode.h"
#include <asm/system.h>

void halt_dsp( struct audiodsp_priv *priv);
void reset_dsp( struct audiodsp_priv *priv);
 int dsp_start( struct audiodsp_priv *priv, struct auidodsp_microcode *mcode);
 int dsp_stop( struct audiodsp_priv *priv);


#define DSP_RD(reg)	({dma_cache_inv((unsigned long)reg,4);(*((unsigned long *)reg));})
#define DSP_WD(reg,val)	({(*((unsigned long *)(reg)))=val;dma_cache_wback(( unsigned long)reg,4);dma_cache_inv((unsigned long)reg,4);})
#endif

