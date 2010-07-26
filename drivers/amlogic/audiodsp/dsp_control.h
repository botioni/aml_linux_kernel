
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

#define DSP_RD(reg) READ_MPEG_REG(reg)
#define DSP_WD(reg,val)	WRITE_MPEG_REG(reg, val)

#endif

