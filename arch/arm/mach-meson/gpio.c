/*
Linux gpio.C

*/
#include <linux/module.h>
#include <mach/am_regs.h>
#include <mach/gpio.h>

struct gpio_addr
{
	unsigned long mode_addr;
	unsigned long out_addr;
	unsigned long in_addr;
};
static struct gpio_addr gpio_addrs[]=
{
	[PREG_EGPIO]={PREG_EGPIO_EN_N,PREG_EGPIO_O,PREG_EGPIO_I},
	[PREG_FGPIO]={PREG_FGPIO_EN_N,PREG_FGPIO_O,PREG_FGPIO_I},
	[PREG_GGPIO]={PREG_GGPIO_EN_N,PREG_GGPIO_O,PREG_GGPIO_I},
	[PREG_HGPIO]={PREG_HGPIO_EN_N,PREG_HGPIO_O,PREG_HGPIO_I},
};

int set_gpio_mode(gpio_bank_t bank,int bit,gpio_mode_t mode)
{
	unsigned long addr=gpio_addrs[bank].mode_addr;
	WRITE_CBUS_REG_BITS(addr,mode,bit,1);
	return 0;
}
gpio_mode_t get_gpio_mode(gpio_bank_t bank,int bit)
{
	unsigned long addr=gpio_addrs[bank].mode_addr;
	return (READ_CBUS_REG_BITS(addr,bit,1)>0)?(GPIO_INPUT_MODE):(GPIO_OUTPUT_MODE);
}


int set_gpio_val(gpio_bank_t bank,int bit,unsigned long val)
{
	unsigned long addr=gpio_addrs[bank].out_addr;
	WRITE_CBUS_REG_BITS(addr,val,bit,1);
}
unsigned long  get_gpio_val(gpio_bank_t bank,int bit)
{
	unsigned long addr=gpio_addrs[bank].in_addr;
	return READ_CBUS_REG_BITS(addr,bit,1);
}



