/*
 *
 * arch/arm/mach-meson/meson.c
 *
 *  Copyright (C) 2010 AMLOGIC, INC.
 *
 * License terms: GNU General Public License (GPL) version 2
 * Platform machine definition.
 */
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/mm.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/ioport.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <mach/hardware.h>
#include <mach/platform.h>
#include <mach/memory.h>
#include <mach/memory.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>

#define MACH_MESON_STRING "AMLOGIC MESON-M1 Board"

static __init void m1_init_machine(void)
{
	/* todo: load device drivers */
}

static __init void m1_map_io(void)
{
	meson_map_io();
}

static __init void m1_irq_init(void)
{
	meson_init_irq();
}

MACHINE_START(MESON, MACH_MESON_STRING)
	.phys_io		= MESON_PERIPHS1_PHYS_BASE,
	.io_pg_offst	= (MESON_PERIPHS1_PHYS_BASE >> 18) & 0xfffc,
	.boot_params	= BOOT_PARAMS_OFFSET,
	.map_io			= m1_map_io,
	.init_irq		= m1_irq_init,
	.timer			= &meson_sys_timer,
	.init_machine	= m1_init_machine,
MACHINE_END
