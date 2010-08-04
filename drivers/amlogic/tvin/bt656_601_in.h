    
#ifndef __BT656_601_INPUT_H
#define __BT656_601_INPUT_H

#include "vframe.h"
#include "vframe_provider.h"    
//#define DEBUG

extern void set_next_field_bt656in_anci_address(unsigned char index);
extern void start_amvdec_656in(void);
extern void stop_amvdec_656in(void);
extern vframe_t * amvdec_656in_run(void);

#endif				//__BT656_601_INPUT_H
    
