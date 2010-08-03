#include <linux/module.h>
#include <linux/delay.h>
#include <mach/am_regs.h>
#include <mach/clk_set.h>
/*
Get two number's max common divisor;
*/
static unsigned long base_clock=24000000;//24M crystral
#ifdef CONFIG_INIT_A9_CLOCK_FREQ
static unsigned long __initdata init_clock=CONFIG_INIT_A9_CLOCK;
#else
static unsigned long __initdata init_clock=0;
#endif
	
static int get_max_common_divisor(int a,int b){
        while(b){
                int temp=b;
                b=a%b;
                a=temp;
        }
        return a;
}

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

/*
out_freq=crystal_req*m/n
out_freq=crystal_req*m/n-->
n=crystal_req*m/out_freq
m=out_freq*n/crystal_req
*/
int demod_apll_setting(unsigned crystal_req,unsigned out_freq)
{
	int n,m;
	unsigned long crys_M,out_M,middle_freq;
	if(!crystal_req)  crystal_req=base_clock;
	crys_M=crystal_req/1000000;
	out_M=out_freq/1000000;
	middle_freq=get_max_common_divisor(crys_M,out_M);
	n=crys_M/middle_freq;
	m=out_M/(middle_freq);
	
	if(n>(1<<5)-1)
		{
		printk(KERN_ERR "demod_apll_setting setting error, n is too bigger n=%d,crys_M=%ldM,out_M=%ldM\n",
			   n,crys_M,out_M);
		return -1;
		}
	if(m>(1<<9)-1)
		{
		printk(KERN_ERR "demod_apll_setting setting error, m is too bigger m=%d,crys_M=%ldM,out_M=%ldM\n",
			   m,crys_M,out_M);
		return -2;
		}
	printk("demod_apll_setting crystal_req=%ld,out_freq=%ld,n=%d,m=%dM\n",crys_M,out_M,n,m);
	/*==========Set Demod PLL, should be in system setting===========*/
	//Set 1.2G PLL
	CLEAR_CBUS_REG_MASK(HHI_DEMOD_PLL_CNTL,0xFFFFFFFF);
	SET_CBUS_REG_MASK(HHI_DEMOD_PLL_CNTL,n<<9 | m<< 0);//

	//Set 400M PLL
	CLEAR_CBUS_REG_MASK(HHI_DEMOD_PLL_CNTL3,0xFFFF0000);
	SET_CBUS_REG_MASK(HHI_DEMOD_PLL_CNTL3,0x0C850000);//400M 300M 240M enable
	return 0;
	
}

int sys_clkpll_setting(unsigned crystal_freq,unsigned  out_freq)
{
	int n,m;
	unsigned long crys_M,out_M,middle_freq,flags;
	if(!crystal_freq)  crystal_freq=base_clock;
	crys_M=crystal_freq/1000000;
	out_M=out_freq/1000000;
	middle_freq=get_max_common_divisor(crys_M,out_M);
	n=crys_M/middle_freq;
	m=out_M/(middle_freq);
	if(n>(1<<5)-1)
	{
		printk(KERN_ERR "sys_clk_setting  error, n is too bigger n=%d,crys_M=%ldM,out=%ldM\n",
		n,crys_M,out_M);
		return -1;
	}
	if(m>(1<<9)-1)
	{
		printk(KERN_ERR "sys_clk_setting  error, m is too bigger m=%d,crys_M=%ldM,out=%ldM\n",
		m,crys_M,out_M);
		return -2;
	}
	if(out_freq>1300*CLK_1M ||out_freq<700*CLK_1M)
	{
		printk(KERN_WARNING"sys_clk_setting  warning,VCO may no support out_freq,crys_M=%ldM,out=%ldM\n",crys_M,out_M);
	}
	printk("a9_clk_setting crystal_req=%ld,out_freq=%ld,n=%d,m=%d\n",crys_M,out_M,n,m);
	local_irq_save(flags);
	WRITE_MPEG_REG(HHI_SYS_PLL_CNTL, m<<0 | n<<9); // system PLL
	WRITE_MPEG_REG(HHI_A9_CLK_CNTL, // A9 clk set to system clock/2
                        (0 << 10) | // 0 - sys_pll_clk, 1 - audio_pll_clk
                        (1 << 0 ) | // 1 - sys/audio pll clk, 0 - XTAL
                        (1 << 4 ) | // APB_CLK_ENABLE
                        (1 << 5 ) | // AT_CLK_ENABLE
                        (0 << 2 ) | // div1
                        (1 << 7 )); // Connect A9 to the PLL divider output
	udelay(10);
	local_irq_restore(flags);
	return 0;
}
unsigned long long clkparse(const char *ptr, char **retptr)
{
	char *endptr;	/* local pointer to end of parsed string */

	unsigned long long ret = simple_strtoull(ptr, &endptr, 0);

	switch (*endptr) {
	case 'G':
	case 'g':
		ret *= 1000;
	case 'M':
	case 'm':
		ret *= 1000;
	case 'K':
	case 'k':
		ret *= 1000;
		endptr++;
	default:
		break;
	}

	if (retptr)
		*retptr = endptr;

	return ret;
}
static int __init base_clk_set(char *ptr)
{
	base_clock=clkparse(ptr,0);
	return 0;
}
__setup("baseclock=",base_clk_set);

static int __init a9_clock_setup(char *ptr)
{
	init_clock=clkparse(ptr,0);
	sys_clkpll_setting(0,init_clock<<1);
	return 0;
}
__setup("a9_clk=",a9_clock_setup);

