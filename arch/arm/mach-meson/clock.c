/*
 *
 * arch/arm/mach-meson/clock.c
 *
 *  Copyright (C) 2010 AMLOGIC, INC.
 *
 * License terms: GNU General Public License (GPL) version 2
 * Define clocks in the app platform.
 *
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/clk.h>
#include <linux/init.h>
#include <linux/spinlock.h>

#include <asm/clkdev.h>
#include <mach/clock.h>
#include <mach/hardware.h>

static DEFINE_SPINLOCK(clockfw_lock);

unsigned long clk_get_rate(struct clk *clk)
{
    if (!clk)
        return 0;

    return clk->rate;
}
EXPORT_SYMBOL(clk_get_rate);

int clk_set_rate(struct clk *clk, unsigned long rate)
{
    unsigned long flags;
    int ret = -EINVAL;

    if (clk == NULL)
        return ret;

    spin_lock_irqsave(&clockfw_lock, flags);

    ret = clk->set_rate(clk, rate);

    spin_unlock_irqrestore(&clockfw_lock, flags);

    return ret;
}
EXPORT_SYMBOL(clk_set_rate);

long clk_round_rate(struct clk *clk, unsigned long rate)
{
    unsigned long ret = 0;

    if (clk->pll_table) {
        const pll_t *pll = clk->pll_table;

        while (pll->rate) {
            if (pll->rate <= rate) {
                ret = pll->rate;
                break;
            }

            pll++;
        }

        /* first */
        if (pll == clk->pll_table)
            return ret;

        /* last */
        if (ret == 0) {
            pll--;
            return pll->rate;
        }

        /* in-between */
        if ((ret - rate) < ((pll-1)->rate - ret))
            return ret;
        else
            return (pll-1)->rate;
    }

    return -EINVAL;
}
EXPORT_SYMBOL(clk_round_rate);

static const pll_t syspll_table_25[] =
{
    {1200000000, 0x00230, 1},
    {1000000000, 0x00228, 1},
    { 900000000, 0x00224, 1},
    { 800000000, 0x00220, 1},
    { 700000000, 0x0021c, 1},
    { 600000000, 0x10230, 1},
    { 500000000, 0x10228, 1},
    {0}
};

static const pll_t otherpll_table_25[] =
{
    { 180000000, 0x00a6c, 3},
    {0}
};

static const pll_t *find_pll(struct clk *clk, int rate)
{
    if (clk->pll_table) {
        const pll_t *pll = clk->pll_table;

        while (pll->rate) {
            if (pll->rate == rate)
                return pll;

            pll++;
        }
    }

    return NULL;
}
static char * clock_src_name[]={
		"XTAL input",
		"XTAL input divided by 2",
		"other PLL",
		"DDR PLL",
		"dmod PLL"
};
int clk_set_usb_phy_clk(struct clk *clk,unsigned long rate)
{

    int divider =0;
    int clk_sel = rate;
    int i;
    int time_dly = 50000;


	// ------------------------------------------------------------
	//  CLK_SEL: These bits select the source for the 12Mhz: 
	// 0 = XTAL input (24, 25, 27Mhz)
	// 1 = XTAL input divided by 2
	// 2 = other PLL output
	// 3 = DDR pll clock (typically 400mhz)
	// 4 = demod 240Mhz PLL output
	CLEAR_CBUS_REG_MASK(PREI_USB_PHY_REG, PREI_USB_PHY_CLK_SEL);
	//clk_sel = 0; // 24M CLK 
	//clk_sel = 1; // 12M, Phy default setting is 12Mhz
	//clk_sel = 2; // other PLL, 540M
	//clk_sel = 3; // DDR, 369M
	//clk_sel = 4; // demod, 240M
	
	printk(KERN_NOTICE"USB PHY clock souce: %s\n",clock_src_name[clk_sel]);
	SET_CBUS_REG_MASK(PREI_USB_PHY_REG, (clk_sel<<5 ));

    if(clk_sel == 3)//DDR runing 396MHz (396/(32+1)=12)
    {
 		CLEAR_CBUS_REG_MASK(PREI_USB_PHY_REG,PREI_USB_PHY_CLK_DIV);
 		SET_CBUS_REG_MASK(PREI_USB_PHY_REG, (32 << 24));
    }else if(clk_sel == 2)//Other PLL running at 540M (540/(44+1)=12)
    {
 		CLEAR_CBUS_REG_MASK(PREI_USB_PHY_REG,PREI_USB_PHY_CLK_DIV);
 		SET_CBUS_REG_MASK(PREI_USB_PHY_REG, (44 << 24));
    }else if(clk_sel == 4)// demod 240M (240/(19+1) = 12)
    {
 		CLEAR_CBUS_REG_MASK(PREI_USB_PHY_REG,PREI_USB_PHY_CLK_DIV);
 		SET_CBUS_REG_MASK(PREI_USB_PHY_REG, (19 << 24));
    }
	// Open clock gate, to enable CLOCK to usb phy 
    SET_CBUS_REG_MASK(PREI_USB_PHY_REG, PREI_USB_PHY_CLK_GATE);
    i=0;
    while(i++<time_dly){};
	
    /*  Reset USB PHY A  */
    SET_CBUS_REG_MASK(PREI_USB_PHY_REG, PREI_USB_PHY_A_AHB_RSET);
    i=0;
    while(i++<time_dly){};  
    CLEAR_CBUS_REG_MASK(PREI_USB_PHY_REG, PREI_USB_PHY_A_AHB_RSET);
    i=0;
    while(i++<time_dly){};
    SET_CBUS_REG_MASK(PREI_USB_PHY_REG, PREI_USB_PHY_A_CLK_RSET);
    i=0;
    while(i++<time_dly){};      
    CLEAR_CBUS_REG_MASK(PREI_USB_PHY_REG, PREI_USB_PHY_A_CLK_RSET);
    i=0;
    while(i++<time_dly){};
    SET_CBUS_REG_MASK(PREI_USB_PHY_REG, PREI_USB_PHY_A_PLL_RSET);
    i=0;
    while(i++<time_dly){};
    CLEAR_CBUS_REG_MASK(PREI_USB_PHY_REG, PREI_USB_PHY_A_PLL_RSET);
    i=0;
    while(i++<time_dly){};

    // ------------------------------------------------------------ 
    // Reset the PHY A by setting POR high for 10uS.
    SET_CBUS_REG_MASK(PREI_USB_PHY_REG, PREI_USB_PHY_A_POR);
    i=0;
    while(i++<time_dly){};
    // Set POR to the PHY high

    CLEAR_CBUS_REG_MASK(PREI_USB_PHY_REG, PREI_USB_PHY_A_POR);
    i=0;
    while(i++<time_dly){};
    
    /* Reset USB PHY B */
    SET_CBUS_REG_MASK(PREI_USB_PHY_REG, PREI_USB_PHY_B_AHB_RSET);
    i=0;
    while(i++<time_dly){};
    CLEAR_CBUS_REG_MASK(PREI_USB_PHY_REG, PREI_USB_PHY_B_AHB_RSET);
    i=0;
    while(i++<time_dly){};
    SET_CBUS_REG_MASK(PREI_USB_PHY_REG, PREI_USB_PHY_B_CLK_RSET);
    i=0;
    while(i++<time_dly){};
    CLEAR_CBUS_REG_MASK(PREI_USB_PHY_REG, PREI_USB_PHY_B_CLK_RSET);
    i=0;
    while(i++<time_dly){};
    SET_CBUS_REG_MASK(PREI_USB_PHY_REG, PREI_USB_PHY_B_PLL_RSET);
    i=0;
    while(i++<time_dly){};
    CLEAR_CBUS_REG_MASK(PREI_USB_PHY_REG, PREI_USB_PHY_B_PLL_RSET);
    i=0;
    while(i++<time_dly){};

    // ------------------------------------------------------------ 
    // Reset the PHY B by setting POR high for 10uS.
    SET_CBUS_REG_MASK(PREI_USB_PHY_REG, PREI_USB_PHY_B_POR);
    i=0;
    while(i++<time_dly){};

    // Set POR to the PHY high
    CLEAR_CBUS_REG_MASK(PREI_USB_PHY_REG, PREI_USB_PHY_B_POR);
    i=0;
    while(i++<time_dly){};

    return 0;
}
static struct clk usb_clk = {
    .name       = "usb_clk",
    .pll_table  = 0,
    .rate       = 2,
    .set_rate   = clk_set_usb_phy_clk,
};
int clk_set_rate_clk81(struct clk *clk, unsigned long rate)
{
    unsigned long r = rate;
    const pll_t *pll;

    if (r < 1000)
        r = r * 1000000;

    pll = find_pll(clk, r);

    if (clk) {
        WRITE_MPEG_REG(HHI_OTHER_PLL_CNTL, pll->setting); // other PLL
        WRITE_MPEG_REG(HHI_MPEG_CLK_CNTL,   // MPEG clk81 set to other/2
            (1 << 12) |                     // select other PLL
            ((pll->devidor - 1) << 0 ) |    // div1
            (1 << 7 ) |                     // cntl_hi_mpeg_div_en, enable gating
            (1 << 8 ));                     // Connect clk81 to the PLL divider output
    }

    return 0;
}

static struct clk clk81 = {
    .name       = "clk81",
    .pll_table  = otherpll_table_25,
    .rate       = 180000000,
    .set_rate   = clk_set_rate_clk81,
};

int clk_set_rate_a9_clk(struct clk *clk, unsigned long rate)
{
    unsigned long r = rate;
    const pll_t *pll;

    if (r < 1000)
        r = r * 1000000;

    r = r * 2;  /* system PLL is fixed to double a9_clk */

    pll = find_pll(clk, r);

    if (pll) {
        WRITE_MPEG_REG(HHI_SYS_PLL_CNTL, pll->setting); // system PLL
        WRITE_MPEG_REG(HHI_A9_CLK_CNTL,  0);            // A9 clk set to sys_pll/2
    }

    return 0;
}

static struct clk a9_clk = {
    .name       = "a9_clk",
    .pll_table  = syspll_table_25,
    .rate       = 600000000,
    .set_rate   = clk_set_rate_a9_clk,
};

/*
 * Here we only define clocks that are meaningful to
 * look up through clockdevice.
 */
static struct clk_lookup lookups[] = {
    {
        .dev_id = "a9_clk",
        .clk    = &a9_clk,
    }, {
        .dev_id = "clk81",
        .clk    = &clk81,
    }, {
       .dev_id = "usb_clk",
       .clk    = &usb_clk,
    }
};

static int __init meson_clock_init(void)
{
    int i;

    /* Register the lookups */
    for (i = 0; i < ARRAY_SIZE(lookups); i++)
        clkdev_add(&lookups[i]);

    return 0;
}

/* initialize clocking early to be available later in the boot */
core_initcall(meson_clock_init);
