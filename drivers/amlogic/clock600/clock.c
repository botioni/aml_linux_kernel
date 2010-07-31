#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/module.h>
#include <linux/platform_device.h>


#include <mach/am_regs.h>

#define DRIVER_NAME "clock600"
#define MODULE_NAME "clock600"

static u32 clock = 450;

static int __init clock600_init_module(void)
{
    u32 eth_clk = READ_CBUS_REG(HHI_ETH_CLK_CNTL);

    if (clock == 450) {
        WRITE_CBUS_REG(HHI_SYS_PLL_CNTL, 0x44b);
        if (((eth_clk >> 9) & 7) == 0) {
            WRITE_MPEG_REG(HHI_ETH_CLK_CNTL, (eth_clk & ~0xff) | ((900/3/50)-1));
        }
        printk("A9 clock set to 450M, module quits by itself, it's not an error\n");

        return -1;
    }
    
    if (clock == 600) {
        WRITE_CBUS_REG(HHI_SYS_PLL_CNTL, 0x232);
        if (((eth_clk >> 9) & 7) == 0) {
            WRITE_MPEG_REG(HHI_ETH_CLK_CNTL, (eth_clk & ~0xff) | ((1200/3/50)-1));
        }
        printk("A9 clock set to 600, module quits by itself, it's not an error\n");
        
        return -1;
    }

    printk("Unsupported clock, usage : insmod clock600.ko clock=450 or 600\n");
    return -1;
}

module_param(clock, uint, 0664);
MODULE_PARM_DESC(clock, "\n ARM clock \n");

module_init(clock600_init_module);
