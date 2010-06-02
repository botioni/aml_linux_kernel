/*
 * linux/drivers/input/irremote/apollo_remote_kbd.c
 *
 * apollo Keypad Driver
 *
 * Copyright (C) 2009 Amlogic Corporation
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 * author :   jianfeng_wang
 */
 /*
 * !!caution: if you use remote ,you should disable card1 used for  ata_enable pin.
 */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/types.h>
#include <linux/input.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/errno.h>
#include <asm/irq.h>
#include <asm/io.h>
#include <asm/arch/am_regs.h>
#include "amkbd_remote.h"



static int  pulse_index=0; 
extern  char *remote_log_buf;


static int  get_pulse_width(unsigned long data)
{
	struct apollo_kp *apollo_kp_data = (struct apollo_kp *) data;
	int  pulse_width;
	char buf[100];
	const char* state;

	pulse_width     = ( (READ_PERIPHS_REG( PREG_IR_DEC_CONTROL)) & 0x1FFF0000 ) >> 16 ;
	state = apollo_kp_data->step==REMOTE_STATUS_WAIT?"wait":\
				apollo_kp_data->step==REMOTE_STATUS_LEADER?"leader":\
				apollo_kp_data->step==REMOTE_STATUS_DATA?"data":\
				apollo_kp_data->step==REMOTE_STATUS_SYNC?"sync":NULL;
				
	sprintf(buf,"%d:pulse_wdith:%d==>%s\r\n",pulse_index++,pulse_width,state);
	if(strlen(remote_log_buf)>REMOTE_LOG_BUF_LEN)
	{
		remote_log_buf[0]='\0';
	}
	strcat(remote_log_buf,buf);
	//sometimes we found remote  pulse width==0.	in order to sync machine state we modify it .
	if(pulse_width==0)
	{
		switch(apollo_kp_data->step)
		{
			case REMOTE_STATUS_LEADER:
			pulse_width=apollo_kp_data->time_window[0] +1;	
			break;
			case REMOTE_STATUS_DATA:
			pulse_width= apollo_kp_data->time_window[2] +1 ;	
			break;	
		}
	}
	return pulse_width;
}
	
static inline void kbd_software_mode_remote_wait(unsigned long data)
{
	unsigned short pulse_width;
	struct apollo_kp *apollo_kp_data = (struct apollo_kp *) data;

	pulse_width     = get_pulse_width(data) ;
	apollo_kp_data->step   = REMOTE_STATUS_LEADER;
	apollo_kp_data->cur_keycode = 0 ;
	apollo_kp_data->bit_num = apollo_kp_data->bit_count ;
}
static inline void kbd_software_mode_remote_leader(unsigned long data)
{
	unsigned short pulse_width;
	struct apollo_kp *apollo_kp_data = (struct apollo_kp *) data;

	pulse_index=0;
   	pulse_width     = get_pulse_width(data) ;
	if((pulse_width > apollo_kp_data->time_window[0]) && (pulse_width <apollo_kp_data->time_window[1])) {
	    	apollo_kp_data->step   = REMOTE_STATUS_DATA;
	}
    	else {
      		apollo_kp_data->step    = REMOTE_STATUS_WAIT ;
	}

	//RemoteStatus   = REMOTE_STATUS_DATA;
	apollo_kp_data->cur_keycode = 0 ;
	apollo_kp_data->bit_num = apollo_kp_data->bit_count ;
}
static inline void kbd_software_mode_remote_send_key(unsigned long data)
{
  	struct apollo_kp *apollo_kp_data = (struct apollo_kp *) data;
	unsigned int  reort_key_code=apollo_kp_data->cur_keycode>>16&0xffff;
	
	apollo_kp_data->step   = REMOTE_STATUS_SYNC ;
    	if(apollo_kp_data->repeate_flag)
    	{
		if(apollo_kp_data->custom_code != (apollo_kp_data->cur_keycode&0xffff ))
		{
			return ;
		}
		if(((reort_key_code&0xff)^(reort_key_code>>8&0xff))==0xff)
		input_report_key(apollo_kp_data->input, (apollo_kp_data->cur_keycode>>16)&0xff, 2);
		input_dbg("key repeate,scan code :0x%x\r\n", apollo_kp_data->cur_keycode);
    	}else{
		if(apollo_kp_data->custom_code != (apollo_kp_data->cur_keycode&0xffff ))
		{
		       input_dbg("Wrong custom code is 0x%04x\n", apollo_kp_data->cur_keycode&0xffff);
			return ;
		}
		if(((reort_key_code&0xff)^(reort_key_code>>8&0xff))==0xff)
		input_report_key(apollo_kp_data->input, (apollo_kp_data->cur_keycode>>16)&0xff, 1);	
		input_dbg("key pressed,scan code :0x%x\r\n", apollo_kp_data->cur_keycode);
    	}
		
    	
}
static inline void kbd_software_mode_remote_data(unsigned long data)
{
	unsigned short pulse_width;
	struct apollo_kp *apollo_kp_data = (struct apollo_kp *) data;
       
    	pulse_width     = get_pulse_width(data) ;
	apollo_kp_data->step   = REMOTE_STATUS_DATA ;
	
	if((pulse_width > apollo_kp_data->time_window[2]) && (pulse_width < apollo_kp_data->time_window[3])) {
            	apollo_kp_data->bit_num--;
	}
	else if((pulse_width > apollo_kp_data->time_window[4]) && (pulse_width < apollo_kp_data->time_window[5])) {
        	apollo_kp_data->cur_keycode |= 1<<(apollo_kp_data->bit_count-apollo_kp_data->bit_num) ;       //1
        	apollo_kp_data->bit_num--;
	}
    	else {
      		apollo_kp_data->step   = REMOTE_STATUS_WAIT ;
    	}
    	if(apollo_kp_data->bit_num == 0)
    	{
     	 	apollo_kp_data->repeate_flag= 0;
		kbd_software_mode_remote_send_key(data);
    	}
	
}
static inline void kbd_software_mode_remote_sync(unsigned long data)
{
	unsigned short pulse_width;
	struct apollo_kp *apollo_kp_data = (struct apollo_kp *) data;
    
   	pulse_width     = get_pulse_width(data) ;
	
	if((pulse_width > apollo_kp_data->time_window[6]) && (pulse_width < apollo_kp_data->time_window[7])) {
        	apollo_kp_data->repeate_flag=1;
              if(apollo_kp_data->repeat_enable)
    	            kbd_software_mode_remote_send_key(data);
              else{
                    apollo_kp_data->step  = REMOTE_STATUS_SYNC ;
                    return;
                }
	}
    	apollo_kp_data->step  = REMOTE_STATUS_SYNC ;
	apollo_kp_data->timer.data=(unsigned long)apollo_kp_data;
	mod_timer(&apollo_kp_data->timer,jiffies+msecs_to_jiffies(apollo_kp_data->release_delay));//6
	
}
void apollo_kp_sw_reprot_key(unsigned long data)
{
	struct apollo_kp *apollo_kp_data = (struct apollo_kp *) data;
	int	   current_jiffies=jiffies;
    	if((current_jiffies -apollo_kp_data->last_jiffies > 12) && (apollo_kp_data->step  <=  REMOTE_STATUS_SYNC)) {
      		  apollo_kp_data->step = REMOTE_STATUS_WAIT ;
    	}
    	apollo_kp_data->last_jiffies = current_jiffies ;  //ignore a little nsecs.
	switch( apollo_kp_data->step)
    	{
        case REMOTE_STATUS_WAIT:
            kbd_software_mode_remote_wait(data) ;
            break;
        case REMOTE_STATUS_LEADER:
            kbd_software_mode_remote_leader(data);
            break;
        case REMOTE_STATUS_DATA:
            kbd_software_mode_remote_data(data) ;
            break;
        case REMOTE_STATUS_SYNC:
            kbd_software_mode_remote_sync(data) ;
            break;
        default:
            break;
    	}
}



