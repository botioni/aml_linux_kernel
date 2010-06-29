#ifndef __MESON_GPIO_H__
#define	 __MESON_GPIO_H__
/*
// Pre-defined GPIO addresses
// ----------------------------
#define PREG_EGPIO_EN_N                            0x200c
#define PREG_EGPIO_O                               0x200d
#define PREG_EGPIO_I                               0x200e
// ----------------------------
#define PREG_FGPIO_EN_N                            0x200f
#define PREG_FGPIO_O                               0x2010
#define PREG_FGPIO_I                               0x2011
// ----------------------------
#define PREG_GGPIO_EN_N                            0x2012
#define PREG_GGPIO_O                               0x2013
#define PREG_GGPIO_I                               0x2014
// ----------------------------
#define PREG_HGPIO_EN_N                            0x2015
#define PREG_HGPIO_O                               0x2016
#define PREG_HGPIO_I                               0x2017
// ----------------------------
*/

typedef enum gpio_bank
{
	PREG_EGPIO=0,
	PREG_FGPIO,
	PREG_GGPIO,
	PREG_HGPIO
}gpio_bank_t;


typedef enum gpio_mode
{
	GPIO_OUTPUT_MODE,
	GPIO_INPUT_MODE,
}gpio_mode_t;

int set_gpio_mode(gpio_bank_t bank,int bit,gpio_mode_t mode);
gpio_mode_t get_gpio_mode(gpio_bank_t bank,int bit);

int set_gpio_val(gpio_bank_t bank,int bit,unsigned long val);
unsigned long  get_gpio_val(gpio_bank_t bank,int bit);



#endif
