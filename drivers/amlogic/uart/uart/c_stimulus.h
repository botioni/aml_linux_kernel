#ifndef C_STIMULUS_H
#define C_STIMULUS_H

extern void stimulus_print(char *pStr);
extern void stimulus_print_without_timestamp(char *pStr);
extern void stimulus_print_num_hex(unsigned long data);
extern void stimulus_print_num_dec(unsigned long data);
extern void stimulus_event(unsigned long event_num, unsigned long data);
extern void stimulus_wait_event_done(unsigned long event_num);
extern void    stimulus_display(char* fmt, unsigned long data );
extern void    stimulus_display2(char* fmt, unsigned long dat1, unsigned long dat2);

#define stimulus_finish_pass()    stimulus_event( 99, 0 )
#define stimulus_finish_fail(val) stimulus_event( 98, val )

#define _Interrupt1
#define _Interrupt2

#endif
