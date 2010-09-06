
#ifndef __BT656_601_INPUT_H
#define __BT656_601_INPUT_H
#include "../amports/vframe.h"
#include "../amports/vframe_provider.h"
//#define DEBUG

extern void set_next_field_656_601_camera_in_anci_address(unsigned char index);

//input_mode is 0 or 1,         NTSC or PAL input(interlace mode): CLOCK + D0~D7(with SAV + EAV )
//                              0:656--PAL ; 1:656--NTSC   ccir656 input
//input_mode is 2 or 3,         NTSC or PAL input(interlace mode): CLOCK + D0~D7 + HSYNC + VSYNC + FID
//                              2:601--PAL ; 3:601--NTSC   ccir656 input
//input_mode is more than 3,    CAMERA input(progressive mode): CLOCK + D0~D7 + HREF + VSYNC
//                              4:640x480 camera inout(progressive)
//                              5:800x600 camera inout(progressive)
//                              6:1024x768 camera inout(progressive)
//                              .....
//                              0xff: disable 656in/601/camera decode;
extern void start_amvdec_656_601_camera_in(unsigned char input_mode);
extern void stop_amvdec_656_601_camera_in(unsigned char input_mode);

extern int amvdec_656_601_camera_in_run(vframe_t *info);

#endif				//__BT656_601_INPUT_H

