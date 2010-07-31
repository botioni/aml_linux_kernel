#include <linux/module.h>
#include <linux/delay.h>
#include <mach/am_regs.h>

/*
	select clk:
	5,6,7 sata
	4-extern pad
	3-other_pll_clk
	2-ddr_pll_clk
	1-APLL_CLK_OUT_400M
	0----sys_pll_div3 (333~400Mhz)

	clk_freq:50M=50000000
	output_clk:50000000;
	aways,maybe changed for others?
	
*/
int  eth_clk_set(int selectclk,unsigned long clk_freq,unsigned long out_clk)
{
	int n;
	printk("select eth clk-%d,source=%ld,out=%ld\n",selectclk,clk_freq,out_clk);
	if(((clk_freq)%out_clk)!=0)
		{
			printk(KERN_ERR "ERROR:source clk must n times of out_clk=%ld ,source clk=%ld\n",out_clk,clk_freq);
			return -1;
		}
	else
		{
			n=(int)((clk_freq)/out_clk);
		}
	
	WRITE_CBUS_REG(HHI_ETH_CLK_CNTL,
		(n-1)<<0 |
		selectclk<<9 |
		1<<8//enable clk
		); 
	
	//writel(0x70b,(0xc1100000+0x1076*4));  // enable Ethernet clocks   for other clock 600/12 
	//writel(0x107,(0xc1100000+0x1076*4));  // enable Ethernet clocks   for sys clock 1200/3/8
	udelay(100);
	return 0;
}
int auto_select_eth_clk(void)
{
	return -1;
}

