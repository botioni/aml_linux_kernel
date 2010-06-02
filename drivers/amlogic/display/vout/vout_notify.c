/*
 *  linux/drivers/video/apollo/vout_notify.c
 *
 *  Copyright (C) 2009 amlogic
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file COPYING in the main directory of this archive
 * for more details.
 * author :   
 *		 jianfeng_wang@amlogic
 */
#include <linux/module.h>
#include <linux/vout/vout_notify.h>


static BLOCKING_NOTIFIER_HEAD(vout_notifier_list);
static INIT_TV_MODULE_SERVER(vout_module_server) ;
static  DEFINE_MUTEX(vout_mutex)  ;
/**
 *	vout_register_client - register a client notifier
 *	@nb: notifier block to callback on events
 */
int vout_register_client(struct notifier_block *nb)
{
	return blocking_notifier_chain_register(&vout_notifier_list, nb);
}
EXPORT_SYMBOL(vout_register_client);

/**
 *	vout_unregister_client - unregister a client notifier
 *	@nb: notifier block to callback on events
 */
int vout_unregister_client(struct notifier_block *nb)
{
	return blocking_notifier_chain_unregister(&vout_notifier_list, nb);
}
EXPORT_SYMBOL(vout_unregister_client);

/**
 * vout_notifier_call_chain - notify clients of fb_events
 *
 */
int vout_notifier_call_chain(unsigned long val, void *v)
{
	return blocking_notifier_call_chain(&vout_notifier_list, val, v);
}
EXPORT_SYMBOL_GPL(vout_notifier_call_chain);

/*
*interface export to client who want to get current vinfo.
*/
const vinfo_t *get_current_vinfo(void)
{
	const vinfo_t *info;

	mutex_lock(&vout_mutex);

	BUG_ON(vout_module_server.op.get_vinfo == NULL);
	
	info = vout_module_server.op.get_vinfo();

	mutex_unlock(&vout_mutex);

	return info;
}
EXPORT_SYMBOL(get_current_vinfo);

/*
*interface export to client who want to get current vmode.
*/
vmode_t get_current_vmode(void)
{
	const vinfo_t *info;

	mutex_lock(&vout_mutex);

	BUG_ON(vout_module_server.op.get_vinfo == NULL);
	
	info = vout_module_server.op.get_vinfo();

	mutex_unlock(&vout_mutex);

	return info->mode;
}
EXPORT_SYMBOL(get_current_vmode);

/*
*interface export to client who want to set current vmode.
*/
int set_current_vmode(vmode_t mode)
{
	int r;

	mutex_lock(&vout_mutex);

	BUG_ON(vout_module_server.op.set_vmode == NULL);
	
	r = vout_module_server.op.set_vmode(mode);

	mutex_unlock(&vout_mutex);

	return r;
}
EXPORT_SYMBOL(set_current_vmode);

/*
*interface export to client who want to set current vmode.
*/
vmode_t validate_vmode(char *name)
{
	vmode_t r;

	mutex_lock(&vout_mutex);

	BUG_ON(vout_module_server.op.validate_vmode == NULL);
	
	r = vout_module_server.op.validate_vmode(name);

	mutex_unlock(&vout_mutex);

	return r;
}
EXPORT_SYMBOL(validate_vmode);

/*
*here below we offer two functions to get and register tv module server
*tv module server will set and store tvmode attributes for tv encoder
*we can ensure TVMOD SET MODULE independent with these two function.
*/


int vout_register_server(vout_server_t*  new_server)
{
	mutex_lock(&vout_mutex);
	memcpy(&vout_module_server,new_server,sizeof(vout_server_t));
	mutex_unlock(&vout_mutex);
	return  0 ;
}
EXPORT_SYMBOL(vout_register_server);
int vout_unregister_server(void)
{
	mutex_lock(&vout_mutex);
	memset(&vout_module_server.op,0,sizeof(vout_module_server.op));
	mutex_unlock(&vout_mutex);
	return 0;
}
EXPORT_SYMBOL(vout_unregister_server);
