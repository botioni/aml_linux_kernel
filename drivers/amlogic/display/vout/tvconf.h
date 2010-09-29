#ifndef   _TV_CONF_H
#define   _TV_CONF_H

#include "tvoutc.h"
#include  <linux/vout/vout_notify.h>

#define    DISPLAY_CLASS_NAME   "display"
#define  MAX_NUMBER_PARA   10



typedef enum   {
DISP_ATTR_ENABLE=0 ,
DISP_ATTR_MODE,
DISP_ATTR_AXIS,
DISP_ATTR_VDAC_SETTING,
DISP_ATTR_WR_REG,
DISP_ATTR_RD_REG,
DISP_ATTR_MAX
}disp_attr_t ;

typedef  enum  {
ENC_TV=0,
//ENC_HDMI,	
ENC_TYPE_MAX
}enc_type_t;
typedef  struct {
	int x ;
	int y ;
	int w ;
	int h ;
}disp_rect_t;

typedef  struct {
	unsigned int  addr;
	unsigned int  value;
}tv_reg_t ;

typedef  struct {
	unsigned int  major;  //dev major number
	const vinfo_t *vinfo;
	disp_rect_t   disp_rect[2];
	char 	     name[20] ;
	struct class *base_class;
	struct device  *device[ENC_TYPE_MAX];
}disp_module_info_t ;
static void  parse_vdac_setting(char *para);
static  void  read_reg(char *para);
static  void  write_reg(char *para);
static  void  set_disp_mode(char *mode) ;
static void  set_disp_window(char *para) ;
static  void   func_default_null(char  *str);

#define  SHOW_INFO(name)      \
	{return snprintf(buf,40, "%s\n", name);}  	

#define  STORE_INFO(name)\
	{mutex_lock(&tvconf_module_mutex);\
	snprintf(name,40,"%s",buf) ;\
	mutex_unlock(&tvconf_module_mutex); }			
		

#define	SET_DISP_DEVICE_ATTR(name,op)       \
static  char    name[40] ;				  \
static ssize_t aml_display_##name##_show(struct device *dev, struct device_attribute *attr, char *buf) \
{					\
	SHOW_INFO(name)	       \
}					\
static ssize_t  aml_display_##name##_store(struct device *dev, struct device_attribute *attr,  \
			    const char *buf, size_t count) {    \
	 STORE_INFO(name)\
	 op(name) ; 				\
	 return strnlen(buf, count);		\
}									\
struct  device_attribute  class_device_display_##name =  \
__ATTR(name, S_IRUGO|S_IWUSR, aml_display_##name##_show, aml_display_##name##_store) ; 

#define    SET_DISP_CLASS_ATTR(name,op)    \
static  char    name[40] ;				  \
static ssize_t aml_display_attr_##name##_show(struct class  * cla, struct class_attribute *attr, char *buf)   \
{  											\
	SHOW_INFO(name)  	\
} 											\
static ssize_t  aml_display_attr_##name##_store(struct class *cla,  struct class_attribute *attr, \
			    const char *buf, size_t count)    \
{\
	STORE_INFO(name)   						\
	op(name) ;						\
	return strnlen(buf, count);				\
}											\
struct  class_attribute  class_display_attr_##name =  \
__ATTR(name, S_IRUGO|S_IWUSR, aml_display_attr_##name##_show, aml_display_attr_##name##_store) ; 




#endif
