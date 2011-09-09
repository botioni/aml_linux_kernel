#include <linux/module.h>
#include <linux/delay.h>
#include <mach/am_regs.h>
#include <mach/clk_set.h>
#include <mach/clock.h>

struct pll_reg_table {
    unsigned  long xtal_clk;
    unsigned  long out_clk;
    unsigned  long settings;
};
unsigned long get_xtal_clock(void)
{
    unsigned long clk;

    clk = READ_CBUS_REG_BITS(PREG_CTLREG0_ADDR, 4, 5);
    clk = clk * 1000 * 1000;
    return clk;
}

/*
Get two number's max common divisor;
*/

static int get_max_common_divisor(int a, int b)
{
    while (b) {
        int temp = b;
        b = a % b;
        a = temp;
    }
    return a;
}


/*
    select clk:
    7-SYS_PLL_DIV2_CLK
    6-VID2_PLL_CLK
    5-VID_PLL_CLK
    4-AUDIO_PLL_CLK
    3-DDR_PLL_CLK
    2-MISC_PLL_CLK
    1-SYS_PLL_CLK
    0-XTAL (25Mhz)

    clk_freq:50M=50000000
    output_clk:50000000;
    aways,maybe changed for others?
*/


int eth_clk_set(int selectclk, unsigned long clk_freq, unsigned long out_clk, unsigned int clk_invert)
{
    int n;
    printk("select eth clk-%d,source=%ld,out=%ld\n", selectclk, clk_freq, out_clk);
    if (((clk_freq) % out_clk) != 0) {
        printk(KERN_ERR "ERROR:source clk must n times of out_clk=%ld ,source clk=%ld\n", out_clk, clk_freq);
        return -1;
    } else {
        n = (int)((clk_freq) / out_clk);
    }

    WRITE_CBUS_REG(HHI_ETH_CLK_CNTL,
                   (n - 1) << 0 |
                   selectclk << 9 |
                   1 << 8 //enable clk
                  );

    udelay(100);
    return 0;
}
int auto_select_eth_clk(void)
{
    return -1;
}

int sys_clkpll_setting(unsigned crystal_freq, unsigned out_freq)
{
    int i, n, m, lock_flag;
    int od=0;
    unsigned lock_time=0;
    unsigned long result_freq, target_freq;
    unsigned long crys_M, out_M, middle_freq, flags;
    unsigned long freq_log[64];
    int log_index;

    if (!crystal_freq) {
        crystal_freq = get_xtal_clock();
    }
    while (out_freq < 650 * CLK_1M){
        out_freq*=2;
        od++;
    }
    if (od>2){
        printk(KERN_ERR "sys_clk_setting error, od is too big od=%d\n",od);
        return -1;
    }
    crys_M = crystal_freq / 1000000;
    out_M = out_freq / 1000000;
    middle_freq = get_max_common_divisor(crys_M, out_M);
    n = crys_M / middle_freq;
    m = out_M / middle_freq;
    target_freq = out_M >> od;
    if (n > (1 << 5) - 1) {
        printk(KERN_ERR "sys_clk_setting  error, n is too bigger n=%d,crys_M=%ldM,out=%ldM\n",n,crys_M,out_M);
        return -2;
    }
    if (m > (1 << 9) - 1) {
        printk(KERN_ERR "sys_clk_setting  error, m is too bigger m=%d,crys_M=%ldM,out=%ldM\n",m,crys_M,out_M);
        return -3;
    }
    if (out_freq > 1600 * CLK_1M || out_freq < 650 * CLK_1M) {
        printk(KERN_WARNING"sys_clk_setting  warning,VCO may no support out_freq,crys_M=%ldM,out=%ldM\n", crys_M, out_M);
    }
    local_irq_save(flags);
    WRITE_MPEG_REG(HHI_SYS_PLL_CNTL, m << 0 | n << 9 | od << 16); // disable and set sys PLL
    lock_flag = 0;
    log_index = 0;
    for (i=0;i<64;i++){
        result_freq = clk_util_clk_msr(SYS_PLL_CLK);
        if ((result_freq <= target_freq+1)&&(result_freq >= target_freq-1)){
            lock_flag++;
            if (lock_flag>=3)
                break;
        }
        if (log_index<64) 
            freq_log[log_index++]=result_freq;
        else 
            break;
        lock_time+=64;
    }
    local_irq_restore(flags);
    //printk("sys clk changed");
    //for (i=0;i<log_index;i++)
    //    printk("-%d", freq_log[i]);
    //printk("\nsys_clk_setting crystal_req=%ld,out_freq=%ld,n=%d,m=%d,od=%d,locktime=%dus\n",crys_M,out_M,n,m,od,lock_time);
    return 0;
}

int misc_pll_setting(unsigned crystal_freq, unsigned  out_freq)
{
    int i, n, m, od;
    unsigned long crys_M, out_M, middle_freq;
    unsigned long flags;
    unsigned lock_flag, lock_time=0;
    unsigned result_freq=0;
    unsigned long freq_log[64];
    int log_index;
    
    if (!crystal_freq) {
        crystal_freq = get_xtal_clock();
    }
    crys_M = crystal_freq / 1000000;
    out_M = out_freq / 1000000;
    if (out_M < 400) {
        /*if <400M, Od=1*/
        od = 1;/*out=pll_out/(1<<od)
                 */
        out_M = out_M << 1;
    } else {
        od = 0;
    }

    middle_freq = get_max_common_divisor(crys_M, out_M);
    n = crys_M / middle_freq;
    m = out_M / (middle_freq);
    if (n > (1 << 5) - 1) {
        printk(KERN_ERR "misc_pll_setting  error, n is too bigger n=%d,crys_M=%ldM,out=%ldM\n",
               n, crys_M, out_M);
        return -1;
    }
    if (m > (1 << 9) - 1) {
        printk(KERN_ERR "misc_pll_setting  error, m is too bigger m=%d,crys_M=%ldM,out=%ldM\n",
               m, crys_M, out_M);
        return -2;
    }
    local_irq_save(flags);
    WRITE_MPEG_REG(HHI_OTHER_PLL_CNTL,
                   m |
                   n << 9 |
                   (od & 1) << 16
                  ); // misc PLL
    lock_flag = 0;
    log_index = 0;
    for (i=0;i<64;i++){
        result_freq = clk_util_clk_msr(MISC_PLL_CLK);
        if ((result_freq <= out_M+1)&&(result_freq >= out_M-1)){
            lock_flag++;
            if (lock_flag>=3)
                break;
        }
        if (log_index<64) 
            freq_log[log_index++]=result_freq;
        else 
            break;
        lock_time+=64;
    }
    WRITE_AOBUS_REG_BITS(AO_UART_CONTROL, (((out_freq/4) / (115200*4)) - 1) & 0xfff, 0, 12);

    local_irq_restore(flags);
    printk("misc pll changed");
    for (i=0;i<log_index;i++)
        printk("-%d", freq_log[i]);
    printk("\nmisc pll setting to crystal_req=%ld,out_freq=%ld,n=%d,m=%d,od=%d,locktime=%dus\n", crys_M, out_M / (od + 1), n, m, od, lock_time);
    return 0;
}


int audio_pll_setting(unsigned crystal_freq, unsigned  out_freq)
{
    int n, m, od;
    unsigned long crys_M, out_M, middle_freq;
    /*
    FIXME:If we need can't exact setting this clock,Can used a pll table?
    */
    if (!crystal_freq) {
        crystal_freq = get_xtal_clock();
    }
    crys_M = crystal_freq / 1000000;
    out_M = out_freq / 1000000;
    if (out_M < 400) {
        /*if <400M, Od=1*/
        od = 1;/*out=pll_out/(1<<od)
                 */
        out_M = out_M << 1;
    } else {
        od = 0;
    }
    middle_freq = get_max_common_divisor(crys_M, out_M);
    n = crys_M / middle_freq;
    m = out_M / (middle_freq);
    if (n > (1 << 5) - 1) {
        printk(KERN_ERR "audio_pll_setting  error, n is too bigger n=%d,crys_M=%ldM,out=%ldM\n",
               n, crys_M, out_M);
        return -1;
    }
    if (m > (1 << 9) - 1) {
        printk(KERN_ERR "audio_pll_setting  error, m is too bigger m=%d,crys_M=%ldM,out=%ldM\n",
               m, crys_M, out_M);
        return -2;
    }
    WRITE_MPEG_REG(HHI_AUD_PLL_CNTL,
                   m |
                   n << 9 |
                   (od & 1) << 14
                  ); // audio PLL
    printk(KERN_INFO "audio_pll_setting to crystal_req=%ld,out_freq=%ld,n=%d,m=%d,od=%d\n", crys_M, out_M / (od + 1), n, m, od);
    return 0;
}

int video_pll_setting(unsigned crystal_freq, unsigned  out_freq, int powerdown, int flags)
{
    int n, m, od;
    unsigned long crys_M, out_M, middle_freq;
    int ret = 0;
    /*
    flags can used for od1/xd settings
    FIXME:If we needn't exact setting this clock,Can used a pll table?
    */
    if (!crystal_freq) {
        crystal_freq = get_xtal_clock();
    }
    crys_M = crystal_freq / 1000000;
    out_M = out_freq / 1000000;

    if (out_M < 750) {
        /* if <750M, Od=1 */
        od = 1;/* out=pll_out/(1<<od) */
        out_M = out_M << 1;
    } else {
        od = 0;
    }
    middle_freq = get_max_common_divisor(crys_M, out_M);
    n = crys_M / middle_freq;
    m = out_M / (middle_freq);
    if (n > (1 << 5) - 1) {
        printk(KERN_ERR "video_pll_setting  error, n is too bigger n=%d,crys_M=%ldM,out=%ldM\n",
               n, crys_M, out_M);
        ret = -1;
    }
    if (m > (1 << 9) - 1) {
        printk(KERN_ERR "video_pll_setting  error, m is too bigger m=%d,crys_M=%ldM,out=%ldM\n",
               m, crys_M, out_M);
        ret = -2;
    }
    if (ret) {
        return ret;
    }
	/* There are some differents between M1 and M3*/
    WRITE_MPEG_REG(HHI_VID_PLL_CNTL,
                   m |
                   n << 10 |
                   (od & 0x3) << 20 |
                   (!!powerdown) << 30 /*is power down mode?*/
                  ); // video PLL
    printk(KERN_INFO "video_pll_setting to crystal_req=%ld,out_freq=%ld,n=%d,m=%d,od=%d\n", crys_M, out_M / (od + 1), n, m, od);
    return 0;
}


