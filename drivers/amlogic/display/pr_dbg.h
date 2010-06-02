#ifndef  _PR_DBG_H
#define _PR_DBG_H

typedef  int   (*type_printk)(const char *fmt, ...) ;
extern int  normal_printk(const char *fmt, ...);
extern int  mute_printk(const char *fmt, ...);
extern type_printk   pr_dbg,pr_err;

#endif
