#ifndef _MACH_MESON_PM_H
#define _MACH_MESON_PM_H

/*
 * Caution: Assembly code in sleep.S makes assumtion on the order
 * of the members of this structure.
 */
struct meson_pm_config {
	void __iomem *ddr2_reg_refresh;
	void __iomem *ddr2_reg_phy;
	void __iomem *ddr_pll_ctrl;
    void __iomem *hiu_base;
    unsigned ddr_clk;
	/*
	 * Note on SLEEPCOUNT:
	 * The SLEEPCOUNT feature is mainly intended for cases in which
	 * the internal oscillator is used. The internal oscillator is
	 * fully disabled in deep sleep mode.  When you exist deep sleep
	 * mode, the oscillator will be turned on and will generate very
	 * small oscillations which will not be detected by the deep sleep
	 * counter.  Eventually those oscillations will grow to an amplitude
	 * large enough to start incrementing the deep sleep counter.
	 * In this case recommendation from hardware engineers is that the
	 * SLEEPCOUNT be set to 4096.  This means that 4096 valid clock cycles
	 * must be detected before the clock is passed to the rest of the
	 * system.
	 * In the case that the internal oscillator is not used and the
	 * clock is generated externally, the SLEEPCOUNT value can be very
	 * small since the clock input is assumed to be stable before SoC
	 * is taken out of deepsleep mode.  A value of 128 would be more than
	 * adequate.
	 */
	int sleepcount;
	void (*set_vccx2)(int power_on);
};

extern unsigned int meson_cpu_suspend_sz;
extern void meson_cpu_suspend(struct meson_pm_config *);

#endif
