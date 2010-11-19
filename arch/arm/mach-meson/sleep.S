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

#define DDR_PHY_OFFSET 0x1380

	.text
/*
 * Move Meson into deep sleep state
 *
 * Note: This code is copied to internal SRAM by PM code. When the Meson
 *	 wakes up it continues execution at the point it went to sleep.
 */
ENTRY(meson_cpu_suspend)
	stmfd	sp!, {r0-r12, lr}		@ save registers on stack

	ldmia	r0, {r0-r4}

	/*
	 * Switch DDR to self-refresh mode.
	 */
	mov	r5, #0x3
	str	r5, [r0]

	/* Disable DDR PHY */
	ldr	r5, [r1]
	orr	r5, r5, #0x1
	str	r5, [r1]

	ldr	r5, [r3]
	bic	r5, r5, #0x41
	str	r5, [r3]

	/* scale down PLL */
	ldr	r6, [r2]
	mov	r5, #(4<<9)
	orr	r5, r5, r6
	str	r5, [r2]

	mov	r5, #0x1000
1:  subs	r5, r5, #0x1
	bne	1b

	/* System goes to sleep beyond after this instruction */
	wfi

	/* Wake up from sleep */

	ldr	r5, [r3]
	orr	r5, r5, #0x41
	str	r5, [r3]

	/* scale PLL up */
	str	r6, [r2]

	/* Wait for PLL to lock */

	/* turn on DDR PHY */
	ldr	r5, [r1]
	bic	r5, r5, #0x1
	str	r5, [r1]

    mov	r5, #0x4
	str	r5, [r0]

    mov	r5, #0x1000
5:  subs	r5, r5, #0x1
    bne	5b

	/* Restore registers and return */
	ldmfd   sp!, {r0-r12, pc}

ENDPROC(meson_cpu_suspend)

CACHE_FLUSH:
	.word	v7_flush_kern_cache_all

ENTRY(meson_cpu_suspend_sz)
	.word	. - meson_cpu_suspend
ENDPROC(meson_cpu_suspend_sz)
