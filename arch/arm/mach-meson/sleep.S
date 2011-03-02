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

#define DDR_SUSPEND
#define WFI_SUSPEND
//#define DDR_PLL_OFF

#define HIU_GCLK_MPEG       (0x50<<2)
#define HIU_DDR_PLL_CTRL    (0x68<<2)
#define A9_AUTO_CLK0        (0x78<<2)
#define HIU_DDR_RESET       (0x102<<2)

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
#ifdef DDR_PLL_OFF
    ldr     r8, [r0, #PCTL_TOGCNT1U_ADDR]
    str     r8, [r5]

    ldr     r8, [r0, #PCTL_TOGCNT100N_ADDR]
    str     r8, [r5, #4]

    ldr     r8, [r0, #PCTL_TREFI_ADDR]
    str     r8, [r5, #8]

    ldr     r8, [r0, #PCTL_ZQCR_ADDR]
    str     r8, [r5, #12]

    ldr     r8, [r0, #PCTL_ODTCFG_ADDR]
    str     r8, [r5, #16]

    ldr     r8, [r0, #PCTL_TMRD_ADDR]
    str     r8, [r5, #20]

    ldr     r8, [r0, #PCTL_TRFC_ADDR]
    str     r8, [r5, #24]

    ldr     r8, [r0, #PCTL_TRP_ADDR]
    str     r8, [r5, #28]

    ldr     r8, [r0, #PCTL_TAL_ADDR]
    str     r8, [r5, #32]

    ldr     r8, [r0, #PCTL_TCWL_ADDR]
    str     r8, [r5, #36]

    ldr     r8, [r0, #PCTL_TCL_ADDR]
    str     r8, [r5, #40]

    ldr     r8, [r0, #PCTL_TRAS_ADDR]
    str     r8, [r5, #44]

    ldr     r8, [r0, #PCTL_TRC_ADDR]
    str     r8, [r5, #48]

    ldr     r8, [r0, #PCTL_TRCD_ADDR]
    str     r8, [r5, #52]

    ldr     r8, [r0, #PCTL_TRRD_ADDR]
    str     r8, [r5, #56]

    ldr     r8, [r0, #PCTL_TRTP_ADDR]
    str     r8, [r5, #60]

    ldr     r8, [r0, #PCTL_TWR_ADDR]
    str     r8, [r5, #64]

    ldr     r8, [r0, #PCTL_TWTR_ADDR]
    str     r8, [r5, #68]

    ldr     r8, [r0, #PCTL_TEXSR_ADDR]
    str     r8, [r5, #72]

    ldr     r8, [r0, #PCTL_TXP_ADDR]
    str     r8, [r5, #76]

    ldr     r8, [r0, #PCTL_TDQS_ADDR]
    str     r8, [r5, #80]

    ldr     r8, [r0, #PCTL_MCFG_ADDR]
    str     r8, [r5, #84]

    ldr     r8, [r0, #PCTL_RSLR0_ADDR]
    str     r8, [r5, #88]

    ldr     r8, [r0, #PCTL_RDGR0_ADDR]
    str     r8, [r5, #92]

    ldr     r8, [r1, #MMC_DDR_CTRL-0x1000]
    str     r8, [r5, #96]
#endif

    /*
     * Switch DDR to self-refresh mode.
     */
    mov     r8, #0x3
    str     r8, [r0, #PCTL_SCTL_ADDR]

    /* DDR PHY power saving*/
    ldr     r8, [r1, #MMC_PHY_CTRL-0x1000]
    orr     r8, r8, #(1<<0)
    orr     r8, r8, #(1<<8)
    str     r8, [r1, #MMC_PHY_CTRL-0x1000]

#ifdef DDR_PLL_OFF
    ldr     r8, [r0, #PCTL_DLLCR9_ADDR]
    orr     r8, r8, #(1<<31)
    str     r8, [r0, #PCTL_DLLCR9_ADDR]
    
    ldr     r8, [r1, #MMC_APB3_CTRL-0x1000]
    bic     r8, r8, #(1<<16)
    str     r8, [r1, #MMC_APB3_CTRL-0x1000]
    
    /* turn off DDR pll */
    ldr     r8, [r2, #HIU_DDR_PLL_CTRL]
    orr     r8, r8, #(1<<15)
    str     r8, [r2, #HIU_DDR_PLL_CTRL]
#else

    mov     r8, #0x1000
0:  subs    r8, r8, #0x1
    bne     0b

    /* scale down PLL */
    ldr     r6, [r2, #HIU_DDR_PLL_CTRL]
    str     r4, [r2, #HIU_DDR_PLL_CTRL]
#endif

    /* ddr power gate */
    ldr     r8, [r2, #HIU_GCLK_MPEG]
    bic     r8, r8, #0x41
    str     r8, [r2, #HIU_GCLK_MPEG]
#endif

    /* System goes to sleep beyond after this instruction */
#ifdef WFI_SUSPEND
    ldr     r8, [r2, #A9_AUTO_CLK0]
    orr     r8, r8, #(1<<1)
    str     r8, [r2, #A9_AUTO_CLK0]
    nop
    nop
    nop
    bic     r8, r8, #(1<<1)
    str     r8, [r2, #A9_AUTO_CLK0]
    wfi
    nop
    nop
    nop
    ldr     r8, [r2, #A9_AUTO_CLK0]
    bic     r8, r8, #(1<<1)
    str     r8, [r2, #A9_AUTO_CLK0]
#else
1:  ldr     r8, [r3]
    and     r8, r8, #4
    cmp     r8, #4
    beq     1b
#endif
    /* Wake up from sleep */

#ifdef DDR_SUSPEND
    /* ddr power gate */
    ldr     r8, [r2, #HIU_GCLK_MPEG]
    orr     r8, r8, #0x41
    str     r8, [r2, #HIU_GCLK_MPEG]

#ifdef DDR_PLL_OFF
    ldr     r8, [r2, #HIU_DDR_PLL_CTRL]
    bic     r8, r8, #(1<<15)
    str     r8, [r2, #HIU_DDR_PLL_CTRL]    

    mov     r8, #0x1000
2:  subs    r8, r8, #0x1
    bne     2b
    
    mov     r8, #(1<<3)
    str     r8, [r2, #HIU_DDR_RESET]    
    
    ldr     r8, [r1, #MMC_PHY_CTRL-0x1000]
    bic     r8, r8, #(1<<8)
    str     r8, [r1, #MMC_PHY_CTRL-0x1000]

    ldr     r8, [r0, #PCTL_DLLCR9_ADDR]
    bic     r8, r8, #(1<<31)
    str     r8, [r0, #PCTL_DLLCR9_ADDR]

    ldr     r8, [r0, #PCTL_IOCR_ADDR]
    bic     r8, r8, #(0xff<<8)
    orr     r8, r8, #(0x0f<<8)
    str     r8, [r0, #PCTL_IOCR_ADDR]
    
    mov     r8, #2
    str     r8, [r0, #PCTL_TINIT_ADDR]

    ldr     r8, [r5]
    str     r8, [r0, #PCTL_TOGCNT1U_ADDR]

    ldr     r8, [r5, #4]
    str     r8, [r0, #PCTL_TOGCNT100N_ADDR]

    ldr     r8, [r5, #8]
    str     r8, [r0, #PCTL_TREFI_ADDR]

    ldr     r8, [r5, #12]
    str     r8, [r0, #PCTL_ZQCR_ADDR]    

    mov     r8, #0
    str     r8, [r0, #PCTL_TRSTH_ADDR]    
3:  ldr     r8, [r0, #PCTL_POWSTAT_ADDR]
    and     r8, r8, #2
    cmp     r8, #0
    beq     3b

    mov     r8, #1
    str     r8, [r0, #PCTL_POWCTL_ADDR]    
4:  ldr     r8, [r0, #PCTL_POWSTAT_ADDR]
    and     r8, r8, #1
    cmp     r8, #0
    beq     4b

    ldr     r8, [r5, #16]
    str     r8, [r0, #PCTL_ODTCFG_ADDR]    
    
    ldr     r8, [r5, #20]
    str     r8, [r0, #PCTL_TMRD_ADDR]

    ldr     r8, [r5, #24]
    str     r8, [r0, #PCTL_TRFC_ADDR]

    ldr     r8, [r5, #28]
    str     r8, [r0, #PCTL_TRP_ADDR]

    ldr     r8, [r5, #32]
    str     r8, [r0, #PCTL_TAL_ADDR]

    ldr     r8, [r5, #36]
    str     r8, [r0, #PCTL_TCWL_ADDR]

    ldr     r8, [r5, #40]
    str     r8, [r0, #PCTL_TCL_ADDR]

    ldr     r8, [r5, #44]
    str     r8, [r0, #PCTL_TRAS_ADDR]

    ldr     r8, [r5, #48]
    str     r8, [r0, #PCTL_TRC_ADDR]

    ldr     r8, [r5, #52]
    str     r8, [r0, #PCTL_TRCD_ADDR]

    ldr     r8, [r5, #56]
    str     r8, [r0, #PCTL_TRRD_ADDR]

    ldr     r8, [r5, #60]
    str     r8, [r0, #PCTL_TRTP_ADDR]

    ldr     r8, [r5, #64]
    str     r8, [r0, #PCTL_TWR_ADDR]
    
    ldr     r8, [r5, #68]
    str     r8, [r0, #PCTL_TWTR_ADDR]

    ldr     r8, [r5, #72]
    str     r8, [r0, #PCTL_TEXSR_ADDR]

    ldr     r8, [r5, #76]
    str     r8, [r0, #PCTL_TXP_ADDR]

    ldr     r8, [r5, #80]
    str     r8, [r0, #PCTL_TDQS_ADDR]

    ldr     r8, [r5, #84]
    str     r8, [r0, #PCTL_MCFG_ADDR]

    ldr     r8, [r5, #96]
    str     r8, [r1, #MMC_DDR_CTRL-0x1000]

    mov     r8, #1
    str     r8, [r0, #PCTL_SCTL_ADDR]
5:	ldr     r8, [r0, #PCTL_STAT_ADDR]
	and     r8, r8, #7
	cmp     r8, #1
	bne     5b

    ldr     r8, [r5, #88]
    str     r8, [r0, #PCTL_RSLR0_ADDR]

    ldr     r8, [r5, #92]
    str     r8, [r0, #PCTL_RDGR0_ADDR]
    
    mov     r8, #2
    str     r8, [r0, #PCTL_SCTL_ADDR]
6:	ldr     r8, [r0, #PCTL_STAT_ADDR]
	and     r8, r8, #7
	cmp     r8, #3
	bne     6b

#else //!DDR_PLL_OFF
    /* scale PLL up */
    str     r6, [r2, #HIU_DDR_PLL_CTRL]

    /* Wait for PLL to lock */
    mov     r8, #0x1000
7:  subs    r8, r8, #0x1
    bne     7b

    /* DDR PHY leave power saving mode*/
    ldr     r8, [r1, #MMC_PHY_CTRL-0x1000]
    bic     r8, r8, #(1<<8)
    str     r8, [r1, #MMC_PHY_CTRL-0x1000]

    /* ddr back to normal mode */
    mov     r8, #0x4
    str     r8, [r0, #PCTL_SCTL_ADDR]
#endif //DDR_PLL_OFF

    mov     r8, #0x1000
8:  subs    r8, r8, #0x1
    bne     8b

#endif //DDR_SUSPEND

    /* Restore registers and return */
    ldmfd   sp!, {r0-r12, pc}
ENDPROC(meson_cpu_suspend)

ENTRY(meson_cpu_suspend_sz)
    .word    . - meson_cpu_suspend
ENDPROC(meson_cpu_suspend_sz)
