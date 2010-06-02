#include "register.h"

// --------------------------------------------------------------------------
// This utility can be used to with $top/common/c_stimulus.v in order to
// allow C-code to trigger events in stimulus.v
//
// The utility supports basic functions like printing a string
// or printing a number.  The C-code / stimulus package also
// allows the user to trigger up to 32 events in stimulus.v
// using calls from the C-code
//
// See $top/common/template_arc for an example
//
// Chris Maaslyar
// 
void    stimulus_event( unsigned long event_num, unsigned long data );

#define stimulus_finish_pass()    stimulus_event( 99, 0 )
#define stimulus_finish_fail(val)    stimulus_event( 98, val )

// ----------------------------------------------
void    stimulus_print( char *pStr )
{
    unsigned long   code_base_offset  = 0;

// For ARC2, it's base address is not zero.  Instead it's base is remapped
// using 12 bits in an ISA register.  If compiling ARC2 code, then take 
// into account this code base offset when using print
#ifdef ARC
    // #define AUDARCCTL           (volatile unsigned long *)0xc1000590
    // bits [15:4] become bits [31:20]
    code_base_offset  = (Rd(AUD_ARC_CTL) >> 4) & 0x3FF;
    code_base_offset  = code_base_offset << 20;
#endif    

    stimulus_event( 97, (unsigned long)(pStr + code_base_offset) );
}

// ----------------------------------------------
void    stimulus_print_without_timestamp( char *pStr )
{
    unsigned long   code_base_offset  = 0;

// For ARC2, it's base address is not zero.  Instead it's base is remapped
// using 12 bits in an ISA register.  If compiling ARC2 code, then take 
// into account this code base offset when using print
#ifdef ARC
    // #define AUDARCCTL           (volatile unsigned long *)0xc1000590
    // bits [15:4] become bits [31:20]
    code_base_offset  = (Rd(AUD_ARC_CTL) >> 4) & 0x3FF;
    code_base_offset  = code_base_offset << 20;
#endif    

    stimulus_event( 94, (unsigned long)(pStr + code_base_offset) );
}

// ----------------------------------------------
void    stimulus_print_num_hex( unsigned long data )
{
    stimulus_event( 96, data );
}

// ----------------------------------------------
void    stimulus_print_num_dec( unsigned long data )
{
    stimulus_event( 95, data );
}

// ----------------------------------------------
void    stimulus_display(char* fmt, unsigned long data )
{
    unsigned int trigger = (1<<31) | 93; 
    // load the value to print
    Wr( ISA_DEBUG_REG0,  data);
    // trigger a print string action
    Wr( ISA_DEBUG_REG1, trigger  );
    // load format string 
    while(*fmt)
    {
            trigger = trigger^(1<<31);;
            Wr( ISA_DEBUG_REG0, *(fmt++) );
            Wr( ISA_DEBUG_REG1, trigger );
    }
    trigger = trigger^(1<<31);;
    Wr( ISA_DEBUG_REG0, 0 );
    Wr( ISA_DEBUG_REG1, trigger );
    Wr( ISA_DEBUG_REG1, 0 );
}

void    stimulus_display2(char* fmt, unsigned long dat1, unsigned long dat2)
{
    unsigned int trigger = 92; 

    trigger = trigger^(1<<31);;
    Wr( ISA_DEBUG_REG0, dat1);
    Wr( ISA_DEBUG_REG1, trigger  );

    trigger = trigger^(1<<31);;
    Wr( ISA_DEBUG_REG0, dat2);
    Wr( ISA_DEBUG_REG1, trigger  );

    while (*fmt) {
	trigger = trigger^(1<<31);;
	Wr( ISA_DEBUG_REG0, *(fmt++) );
	Wr( ISA_DEBUG_REG1, trigger );
    }

    trigger = trigger^(1<<31);;
    Wr( ISA_DEBUG_REG0, 0 );
    Wr( ISA_DEBUG_REG1, trigger );

    Wr( ISA_DEBUG_REG1, 0 );
}

// ----------------------------------------------
void    stimulus_event( unsigned long event_num, unsigned long data )
{
    // load the value to print
    Wr( ISA_DEBUG_REG0, data );
    // trigger a print string action
    Wr( ISA_DEBUG_REG1, (1 << 31) | (event_num & 0xFF) );
    Wr( ISA_DEBUG_REG1, 0 );
}
// ----------------------------------------------
void    stimulus_wait_event_done( unsigned long event_num )
{
    while( Rd( ISA_DEBUG_REG2 ) & (1 << event_num) ) {}
}

