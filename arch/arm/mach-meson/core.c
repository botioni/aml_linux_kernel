/*
 *  arch/arm/mach-meson/core.c
 *
 *  Copyright (C) 2010 AMLOGIC, INC.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/io.h>
#include <linux/mm.h>
#include <linux/clockchips.h>
#include <linux/clocksource.h>
 
#include <mach/hardware.h>
#include <asm/memory.h>
#include <asm/mach/map.h>
#include <asm/mach/time.h>
#include <asm/mach/irq.h>

/***********************************************************************
 * IRQ
 **********************************************************************/
#define IRQ_BIT(irq)			((irq) & 0x1f)
#define IRQ_MASK_REG(irq)		((CBUS_REG_ADDR(A9_IRQ_IN0_INTR_MASK)) + ((irq) >> 5))
#define IRQ_STATUS_REG(irq)		((CBUS_REG_ADDR(A9_IRQ_IN0_INTR_STAT)) + ((irq) >> 5))
#define IRQ_CLR_REG(irq)		((CBUS_REG_ADDR(A9_IRQ_IN0_INTR_STAT_CLR)) + ((irq) >> 5))
#define IRQ_FIQSEL_REG(irq) 	((CBUS_REG_ADDR(A9_IRQ_IN0_INTR_FIRQ_SEL)) + ((irq) >> 5))

/* Enable interrupt */
static void meson_unmask_irq(unsigned int irq)
{
	unsigned int mask;

	if (irq >= NR_IRQS)
		return;

	mask = 1 << IRQ_BIT(irq);

	SET_MPEG_REG_MASK(IRQ_MASK_REG(irq), mask);
}

/* Disable interrupt */
static void meson_mask_irq(unsigned int irq)
{
	unsigned int mask;

	if (irq >= NR_IRQS)
		return;

	mask = 1 << IRQ_BIT(irq);

	CLEAR_MPEG_REG_MASK(IRQ_MASK_REG(irq), mask);
}

/* Clear interrupt */
static void meson_ack_irq(unsigned int irq)
{
	unsigned int mask;

	if (irq >= NR_IRQS)
		return;

	mask = 1 << IRQ_BIT(irq);

	WRITE_MPEG_REG(IRQ_CLR_REG(irq), mask);
}

static struct irq_chip meson_irq_chip = {
	.name	= "MESON-INTC",
	.ack	= meson_ack_irq,
	.mask	= meson_mask_irq,
	.unmask = meson_unmask_irq,
};

/* ARM Interrupt Controller Initialization */
void __init meson_init_irq(void)
{
	unsigned i;

	/* Disable all interrupt requests */
	WRITE_MPEG_REG(A9_IRQ_IN0_INTR_MASK, 0);
	WRITE_MPEG_REG(A9_IRQ_IN1_INTR_MASK, 0);
	WRITE_MPEG_REG(A9_IRQ_IN2_INTR_MASK, 0);
	WRITE_MPEG_REG(A9_IRQ_IN3_INTR_MASK, 0);

	/* Clear all interrupts */
	WRITE_MPEG_REG(A9_IRQ_IN0_INTR_STAT_CLR, ~0);
	WRITE_MPEG_REG(A9_IRQ_IN1_INTR_STAT_CLR, ~0);
	WRITE_MPEG_REG(A9_IRQ_IN2_INTR_STAT_CLR, ~0);
	WRITE_MPEG_REG(A9_IRQ_IN3_INTR_STAT_CLR, ~0);

	/* Set all interrupts to IRQ */
	WRITE_MPEG_REG(A9_IRQ_IN0_INTR_FIRQ_SEL, 0);
	WRITE_MPEG_REG(A9_IRQ_IN1_INTR_FIRQ_SEL, 0);
	WRITE_MPEG_REG(A9_IRQ_IN2_INTR_FIRQ_SEL, 0);
	WRITE_MPEG_REG(A9_IRQ_IN3_INTR_FIRQ_SEL, 0);

	/* set up genirq dispatch */
	for (i = 0; i < NR_IRQS; i++) {
		set_irq_chip(i, &meson_irq_chip);
		set_irq_handler(i, handle_level_irq);
		set_irq_flags(i, IRQF_VALID);
	}
}

/***********************************************************************
 * IO Mapping
 **********************************************************************/
static __initdata struct map_desc meson_io_desc[] = {
	{
		.virtual	= IO_CBUS_BASE,
		.pfn		= __phys_to_pfn(IO_CBUS_BASE),
		.length		= SZ_2M,
		.type		= MT_DEVICE,
	} , {
		.virtual	= IO_AXI_BUS_BASE,
		.pfn		= __phys_to_pfn(IO_AXI_BUS_BASE),
		.length		= SZ_1M,
		.type		= MT_DEVICE,
	} , {
		.virtual	= IO_AHB_BUS_BASE,
		.pfn		= __phys_to_pfn(IO_AHB_BUS_BASE),
		.length		= SZ_16M,
		.type		= MT_DEVICE,
	} , {
		.virtual	= IO_APB_BUS_BASE,
		.pfn		= __phys_to_pfn(IO_APB_BUS_BASE),
		.length		= SZ_512K,
		.type		= MT_DEVICE,
	} ,
};

void __init meson_map_io(void)
{
	iotable_init(meson_io_desc, ARRAY_SIZE(meson_io_desc));
}

/***********************************************************************
 * System timer
 **********************************************************************/

/********** Clock Source Device, Timer-A *********/

static cycle_t cycle_read_timerE(struct clocksource *cs)
{
    return (cycles_t) READ_MPEG_REG(ISA_TIMERE);
}

static struct clocksource clocksource_timer_e = {
    .name   = "Timer-E",
    .rating = 300,
    .read   = cycle_read_timerE,
    .mask   = CLOCKSOURCE_MASK(24),
    .mult	= 1000,
    .shift	= 0,
    .flags  = CLOCK_SOURCE_IS_CONTINUOUS,
};

static void __init meson_clocksource_init(void)
{
	CLEAR_MPEG_REG_MASK(ISA_TIMER_MUX, TIMER_A_INPUT_MASK);
	SET_MPEG_REG_MASK(ISA_TIMER_MUX, TIMERE_UNIT_1us << TIMER_E_INPUT_BIT);
	WRITE_MPEG_REG(ISA_TIMERE, 0);

    clocksource_register(&clocksource_timer_e);
}

/********** Clock Event Device, Timer-A *********/

static struct clock_event_device clockevent_meson_1mhz = {
	.name           = "TIMER-A",
	.rating         = 300, /* Reasonably fast and accurate clock event */

	/* todo: CLOCK_EVT_FEAT_ONESHOT with TIMER-C? */
	.features       = CLOCK_EVT_FEAT_PERIODIC,
    .mult			= 1000,
	.shift          = 0,
#if 0
	.set_next_event = meson_set_next_event,
	.set_mode       = meson_set_mode,
#endif
};

/* Clock event timer interrupt handler */
static irqreturn_t meson_timer_interrupt(int irq, void *dev_id)
{
	struct clock_event_device *evt = &clockevent_meson_1mhz;

	meson_mask_irq(irq);

	evt->event_handler(evt);

	return IRQ_HANDLED;
}

static struct irqaction meson_timer_irq = {
	.name           = "Meson Timer Tick",
	.flags          = IRQF_DISABLED | IRQF_TIMER | IRQF_IRQPOLL,
	.handler        = meson_timer_interrupt,
};

static void __init meson_clockevent_init(void)
{
	/* setup Timer A as 1ms timer */
	WRITE_MPEG_REG(ISA_TIMERA, 1);
	CLEAR_MPEG_REG_MASK(ISA_TIMER_MUX, TIMER_A_INPUT_MASK);
	SET_MPEG_REG_MASK(ISA_TIMER_MUX, TIMER_UNIT_1ms << TIMER_A_INPUT_BIT);

	/* 24bit counter, so 24bits delta is max */
	clockevent_meson_1mhz.max_delta_ns =
		clockevent_delta2ns(0xffffff, &clockevent_meson_1mhz);
	/* This timer is slow enough to set for 1 cycle == 1 MHz */
	clockevent_meson_1mhz.min_delta_ns =
		clockevent_delta2ns(1, &clockevent_meson_1mhz);
	clockevent_meson_1mhz.cpumask = cpumask_of(0);
	clockevents_register_device(&clockevent_meson_1mhz);

	/* Set up the IRQ handler */
	setup_irq(INT_TIMER_A, &meson_timer_irq);
}

/*
 * This sets up the system timers, clock source and clock event.
 */
static void __init meson_timer_init(void)
{
	meson_clocksource_init();
    meson_clockevent_init();
}

struct sys_timer meson_sys_timer =
{
	.init	= meson_timer_init,
};
