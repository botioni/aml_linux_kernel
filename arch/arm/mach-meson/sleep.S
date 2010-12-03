/*
 * (C) Copyright 2010, Amlogic, Inc. http://www.amlogic.com/
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR /PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#include <linux/linkage.h>
#include <asm/assembler.h>
#include <mach/memory.h>

#define DDR_SUSPEND
#define WFI_SUSPEND

    .text
/*
 * Move Meson into deep sleep state
 *
 * Note: This code is copied to internal SRAM by PM code. When the Meson
 *     wakes up it continues execution at the point it went to sleep.
 */
ENTRY(meson_cpu_suspend)
    stmfd   sp!, {r0-r12, lr}        @ save registers on stack

    ldmia   r0, {r0-r5}

#ifdef DDR_SUSPEND
    /*
     * Switch DDR to self-refresh mode.
     */
    mov     r8, #0x3
    str     r8, [r0]

    /* Disable DDR PHY */
    ldr     r8, [r1]
    orr     r8, r8, #0x1
    str     r8, [r1]

    /* scale down PLL */
    ldr     r6, [r2]
    str     r5, [r2]

    /* ddr power gate */
    ldr     r8, [r3]
    bic     r8, r8, #0x41
    str     r8, [r3]
#endif

    /* System goes to sleep beyond after this instruction */
#ifdef WFI_SUSPEND
    wfi
    nop
    nop
    nop
#else
1:  ldr     r8, [r4]
    and     r8, r8, #4
    cmp     r8, #4
    beq     1b
#endif
    /* Wake up from sleep */

#ifdef DDR_SUSPEND
    /* ddr power gate */
    ldr     r8, [r3]
    orr     r8, r8, #0x41
    str     r8, [r3]

    /* scale PLL up */
    str     r6, [r2]

    /* Wait for PLL to lock */
    mov     r8, #0x1000
5:  subs    r8, r8, #0x1
    bne     5b
    
    /* turn on DDR PHY */
    ldr     r8, [r1]
    bic     r8, r8, #0x1
    str     r8, [r1]

    /* ddr back to normal mode */
    mov     r8, #0x4
    str     r8, [r0]
#endif

    /* Restore registers and return */
    ldmfd   sp!, {r0-r12, pc}

ENDPROC(meson_cpu_suspend)

CACHE_FLUSH:
    .word    v7_flush_kern_cache_all

ENTRY(meson_cpu_suspend_sz)
    .word    . - meson_cpu_suspend
ENDPROC(meson_cpu_suspend_sz)
