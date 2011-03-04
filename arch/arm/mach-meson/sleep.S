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
#include <mach/pctl.h>
#include <mach/dmc.h>

#define WFI_SUSPEND

#define VOLTAGE_DLY     0x40
#define MS_DLY          0x10

#define HIU_GCLK_MPEG       (0x50<<2)
#define HHI_MPEG_CLK_CNTL   (0x5d<<2)
#define HIU_DDR_PLL_CTRL    (0x68<<2)
#define A9_AUTO_CLK0        (0x78<<2)
#define HIU_DDR_RESET       (0x102<<2)
#define LED_PWM_REG0        (0x11da<<2)

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

    mov     r9, #0
    movw    r9, #LED_PWM_REG0
    add     r9, r9, r2

    /* turn on auto byte bypass and cmd iddq */
    ldr     r8, [r0, #PCTL_IOCR_ADDR]
    orr     r8, r8, #(0x3<<28)
    orr     r8, r8, #(1<<25)
    str     r8, [r0, #PCTL_IOCR_ADDR]

    /*
     * Switch DDR to self-refresh mode.
     */
    mov     r8, #0x3
    str     r8, [r0, #PCTL_SCTL_ADDR]

    mov     r8, #MS_DLY
0:  subs    r8, r8, #0x1
    bne     0b

    /* DDR PHY power saving*/
    ldr     r8, [r1, #MMC_PHY_CTRL-0x1000]
    orr     r8, r8, #(1<<0)
    orr     r8, r8, #(1<<8)
    orr     r8, r8, #(1<<13)
    str     r8, [r1, #MMC_PHY_CTRL-0x1000]

    /* turn off ddr pll */
    ldr     r8, [r2, #HIU_DDR_PLL_CTRL]
    orr     r8, r8, #(1<<15)
    str     r8, [r2, #HIU_DDR_PLL_CTRL]

    /* ddr power gate */
    ldr     r8, [r2, #HIU_GCLK_MPEG]
    bic     r8, r8, #0x41
    str     r8, [r2, #HIU_GCLK_MPEG]

    /* lower core voltage */
    ldr     r8, [r9]
    bic     r8, r8, #0x0f
    orr     r8, r8, #5
    str     r8, [r9]

    /* System goes to sleep beyond after this instruction */
#ifdef WFI_SUSPEND
    ldr     r8, [r2, #A9_AUTO_CLK0]
    orr     r8, r8, #(1<<1)
    str     r8, [r2, #A9_AUTO_CLK0]
    bic     r8, r8, #(1<<1)
    str     r8, [r2, #A9_AUTO_CLK0]
    wfi
    ldr     r8, [r2, #A9_AUTO_CLK0]
    bic     r8, r8, #(1<<0)
    str     r8, [r2, #A9_AUTO_CLK0]
#else
1:  ldr     r8, [r3]
    and     r8, r8, #4
    cmp     r8, #4
    beq     1b
#endif
    /* Wake up from sleep */

    /* restore core voltage */
    ldr     r8, [r9]
    bic     r8, r8, #0x0f
    str     r8, [r9]
    mov     r8, #VOLTAGE_DLY
2:  subs    r8, r8, #0x1
    bne     2b

    /* ddr power gate */
    ldr     r8, [r2, #HIU_GCLK_MPEG]
    orr     r8, r8, #0x41
    str     r8, [r2, #HIU_GCLK_MPEG]

    /* turn on ddr pll */
    ldr     r8, [r2, #HIU_DDR_PLL_CTRL]
    bic     r8, r8, #(1<<15)
    str     r8, [r2, #HIU_DDR_PLL_CTRL] 

    /* Wait for PLL to lock */
    mov     r8, #MS_DLY
3:  subs    r8, r8, #0x1
    bne     3b

    /* DDR PHY leave power saving mode*/
    ldr     r8, [r1, #MMC_PHY_CTRL-0x1000]
    bic     r8, r8, #(1<<8)
    str     r8, [r1, #MMC_PHY_CTRL-0x1000]

    /* ddr back to normal mode */
    mov     r8, #0x4
    str     r8, [r0, #PCTL_SCTL_ADDR]

    mov     r8, #MS_DLY
4:  subs    r8, r8, #0x1
    bne     4b

    /* Restore registers and return */
    ldmfd   sp!, {r0-r12, pc}
ENDPROC(meson_cpu_suspend)

ENTRY(meson_cpu_suspend_sz)
    .word    . - meson_cpu_suspend
ENDPROC(meson_cpu_suspend_sz)
