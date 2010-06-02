#ifndef   _AMKBD_REMOTE_H
#define   _AMKBD_REMOTE_H
#include  <asm/ioctl.h>
//remote config  ioctl  cmd
#define   REMOTE_IOC_SET_REPEAT_ENABLE			_IOW('I',8,sizeof(short))
#define	REMOTE_IOC_SET_DEBUG_ENABLE			_IOW('I',9,sizeof(short)) 
#define	REMOTE_IOC_SET_MODE					_IOW('I',10,sizeof(short)) 

#define   REMOTE_IOC_SET_RELEASE_DELAY		_IOW('I',99,sizeof(short))
#define   REMOTE_IOC_SET_CUSTOMCODE   			_IOW('I',100,sizeof(short))
//reg
#define   REMOTE_IOC_SET_REG_BASE_GEN			_IOW('I',101,sizeof(short))
#define   REMOTE_IOC_SET_REG_CONTROL			_IOW('I',102,sizeof(short))
#define   REMOTE_IOC_SET_REG_LEADER_ACT 		_IOW('I',103,sizeof(short))
#define   REMOTE_IOC_SET_REG_LEADER_IDLE 		_IOW('I',104,sizeof(short))
#define   REMOTE_IOC_SET_REG_REPEAT_LEADER 	_IOW('I',105,sizeof(short))
#define   REMOTE_IOC_SET_REG_BIT0_TIME		 	_IOW('I',106,sizeof(short))

//sw
#define   REMOTE_IOC_SET_BIT_COUNT			 	_IOW('I',107,sizeof(short))
#define   REMOTE_IOC_SET_TW_LEADER_ACT		_IOW('I',108,sizeof(short))
#define   REMOTE_IOC_SET_TW_BIT0_TIME			_IOW('I',109,sizeof(short))
#define   REMOTE_IOC_SET_TW_BIT1_TIME			_IOW('I',110,sizeof(short))
#define   REMOTE_IOC_SET_TW_REPEATE_LEADER	_IOW('I',111,sizeof(short))

#define   REMOTE_IOC_GET_TW_LEADER_ACT		_IOR('I',112,sizeof(short))
#define   REMOTE_IOC_GET_TW_BIT0_TIME			_IOR('I',113,sizeof(short))
#define   REMOTE_IOC_GET_TW_BIT1_TIME			_IOR('I',114,sizeof(short))
#define   REMOTE_IOC_GET_TW_REPEATE_LEADER	_IOR('I',115,sizeof(short))


#define   REMOTE_IOC_GET_REG_BASE_GEN			_IOR('I',121,sizeof(short))
#define   REMOTE_IOC_GET_REG_CONTROL			_IOR('I',122,sizeof(short))
#define   REMOTE_IOC_GET_REG_LEADER_ACT 		_IOR('I',123,sizeof(short))
#define   REMOTE_IOC_GET_REG_LEADER_IDLE 		_IOR('I',124,sizeof(short))
#define   REMOTE_IOC_GET_REG_REPEAT_LEADER 	_IOR('I',125,sizeof(short))
#define   REMOTE_IOC_GET_REG_BIT0_TIME		 	_IOR('I',126,sizeof(short))
#define   REMOTE_IOC_GET_REG_FRAME_DATA		_IOR('I',127,sizeof(short))
#define   REMOTE_IOC_GET_REG_FRAME_STATUS	_IOR('I',128,sizeof(short))

#define   REMOTE_WORK_MODE_SW 		0
#define   REMOTE_WORK_MODE_HW		1


#define REMOTE_STATUS_WAIT       0
#define REMOTE_STATUS_LEADER     1
#define REMOTE_STATUS_DATA       2
#define REMOTE_STATUS_SYNC       3
#define REMOTE_LOG_BUF_LEN		8192
#define REMOTE_LOG_BUF_ORDER		1


typedef  int   (*type_printk)(const char *fmt, ...) ;


struct apollo_kp {
	struct input_dev *input;
	struct timer_list timer;
	int irq;
	int work_mode ;
	unsigned int cur_keycode;
	unsigned int 	repeate_flag;
	unsigned int repeat_enable;
	unsigned int debounce;
	unsigned int custom_code;
	unsigned int release_delay;
	unsigned int debug_enable;
//sw
	unsigned int delay;
	unsigned int   step;
	unsigned int 	bit_count;
	unsigned int   bit_num;
	unsigned int	last_jiffies;
	unsigned int 	time_window[8];
	int			last_pulse_width;
	int			repeat_time_count;
//config 	
	int			config_major;
	char 		config_name[20];
	struct class *config_class;
	struct device *config_dev;
	
};

extern type_printk input_dbg;


void apollo_kp_sw_reprot_key(unsigned long data);

#endif   //_AMKBD_REMOTE_H
