#ifndef PPMGR_DEV_INCLUDE_H
#define PPMGR_DEV_INCLUDE_H

typedef  struct {
	struct class 		*cla;
	struct device		*dev;
	char  			name[20];
	unsigned int 		open_count;
	int	 			major;
	unsigned  int 		dbg_enable;
	char* buffer_start;
	unsigned int buffer_size;
	int angle;
	int bypass;
	int disp_width;
	int disp_height;

    const vinfo_t *vinfo;
	int left;
	int top;
	int width;
	int height;
}ppmgr_device_t;

extern ppmgr_device_t  ppmgr_device;
#endif /* PPMGR_DEV_INCLUDE_H. */
