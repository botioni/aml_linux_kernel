/*
 * This confidential and proprietary software may be used only as
 * authorised by a licensing agreement from AMLOGIC, INC.
 * (C) COPYRIGHT 2011 AMLOGIC, INC.
 * ALL RIGHTS RESERVED
 * The entire notice above must be reproduced on all authorised
 * copies and copies may only be made to the extent permitted
 * by a licensing agreement from AMLOGIC, INC.
 */

/**
 * @file mali_platform.c
 * Platform specific Mali driver functions for meson platform
 */
#include "mali_kernel_common.h"
#include "mali_osk.h"
#include "mali_platform.h"

#include <linux/kernel.h>
#include <mach/am_regs.h>
#include <mach/clock.h>

_mali_osk_errcode_t mali_platform_init(void)
{
    MALI_SUCCESS;
}

_mali_osk_errcode_t mali_platform_deinit(void)
{
    MALI_SUCCESS;
}

_mali_osk_errcode_t mali_platform_power_mode_change(mali_power_mode power_mode)
{
    /* turn off MALI clock gating */
    unsigned long flags;
    unsigned cpu_divider, mali_divider;
    unsigned ddr_pll_setting, sys_pll_setting;
    unsigned cpu_freq, ddr_freq;
    int mali_flag;

    switch (power_mode) {
        case MALI_POWER_MODE_LIGHT_SLEEP:
            break;
	    case MALI_POWER_MODE_DEEP_SLEEP:
            /* turn on MALI clock gating */
			local_irq_save(flags);
			CLEAR_CBUS_REG_MASK(HHI_MALI_CLK_CNTL, 1 << 8);

			sys_pll_setting = READ_MPEG_REG(HHI_SYS_PLL_CNTL);
			cpu_freq = ((sys_pll_setting&0x1ff)*24)>>(sys_pll_setting>>16); // assume 24M xtal
			cpu_divider = READ_MPEG_REG_BITS(HHI_SYS_CPU_CLK_CNTL, 2, 2);
			if (cpu_divider == 3)
				cpu_divider = 2; // now fix at /4
			cpu_freq >>= cpu_divider;

			ddr_pll_setting = READ_MPEG_REG(HHI_DDR_PLL_CNTL);
			ddr_freq = ((ddr_pll_setting&0x1ff)*24)>>((ddr_pll_setting>>16)&3);

			mali_divider = 1;
			while ((mali_divider * cpu_freq < ddr_freq) || (264 * mali_divider < ddr_freq)) // assume mali max 264M
				mali_divider++;
			mali_flag = ((mali_divider-1) != (READ_MPEG_REG(HHI_MALI_CLK_CNTL)&0x7f));
			if (mali_flag){
				WRITE_CBUS_REG(HHI_MALI_CLK_CNTL,
					(3 << 9)    |                   // select ddr pll as clock source
					((mali_divider-1) << 0)); // ddr clk / divider
				READ_CBUS_REG(HHI_MALI_CLK_CNTL); // delay
			}
			SET_CBUS_REG_MASK(HHI_MALI_CLK_CNTL, 1 << 8);
			local_irq_restore(flags);
			if (mali_flag)
				printk("(CTS_MALI_CLK) = %d/%d = %dMHz --- when mali gate on\n", ddr_freq, mali_divider, ddr_freq/mali_divider);
            break;
        case MALI_POWER_MODE_ON:
            /* turn off MALI clock gating */
            SET_CBUS_REG_MASK(HHI_MALI_CLK_CNTL, 1 << 8);
            break;
    }

    MALI_SUCCESS;
}

void mali_gpu_utilization_handler(u32 utilization)
{
}

void set_mali_parent_power_domain(void* dev)
{
}

