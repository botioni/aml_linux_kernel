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

#ifdef CONFIG_TCA6424
#include <linux/tca6424.h>
#define CONFIG_EXGPIO
#endif

typedef enum gpio_bank
{
	PREG_EGPIO=0,
	PREG_FGPIO,
	PREG_GGPIO,
	PREG_HGPIO,
#ifdef CONFIG_EXGPIO
	EXGPIO_BANK0,
	EXGPIO_BANK1,
	EXGPIO_BANK2,
	EXGPIO_BANK3
#endif
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


#define GPIOA_bank_bit(bit)		(PREG_EGPIO)
#define GPIOA_bit_bit23_26(bit)	(bit-23)
#define GPIOA_bit_bit0_14(bit)		(bit+4)

										
#define GPIOB_bank_bit0_7(bit)		(PREG_EGPIO)
#define GPIOB_bit_bit0_7(bit)			(bit+19)		

#define GPIOC_bank_bit0_26(bit)		(PREG_FGPIO)
#define GPIOC_bit_bit0_26(bit)			(bit)		

#define GPIOD_bank_bit2_24(bit)		(PREG_GGPIO)
#define GPIOD_bit_bit2_24(bit)			(bit-2)		

#define GPIOE_bank_bit0_15(bit)		(PREG_HGPIO)
#define GPIOE_bit_bit0_15(bit)			(bit)		

#define GPIOE_bank_bit16_21(bit)		(PREG_HGPIO)
#define GPIOE_bit_bit16_21(bit)			(bit)		

enum {
    GPIOA_IDX = 0,
    GPIOB_IDX = 15,
    GPIOC_IDX = 23,
    GPIOD_IDX = 50,
    GPIOE_IDX = 75,
    GPIOA_26_IDX = 97,
    GPIOA_25_IDX = 98,
    GPIOA_24_IDX = 99,
    GPIOA_23_IDX = 100,
    JTAG_TCK_IDX = 101,
    JTAG_TMS_IDX = 102,
    JTAG_TDI_IDX = 103,
    JTAG_TDO_IDX = 104,
};

extern int gpio_to_idx(unsigned gpio);

/**
 * enable gpio edge interrupt
 *	
 * @param [in] pin  index number of the chip, start with 0 up to 255 
 * @param [in] flag rising(0) or falling(1) edge 
 * @param [in] group  this interrupt belong to which interrupt group  from 0 to 7
 */
extern void gpio_enable_edge_int(int pin , int flag, int group);
/**
 * enable gpio level interrupt
 *	
 * @param [in] pin  index number of the chip, start with 0 up to 255 
 * @param [in] flag high(0) or low(1) level 
 * @param [in] group  this interrupt belong to which interrupt group  from 0 to 7
 */
extern void gpio_enable_level_int(int pin , int flag, int group);

/**
 * enable gpio interrupt filter
 *
 * @param [in] filter from 0~7(*222ns)
 * @param [in] group  this interrupt belong to which interrupt group  from 0 to 7
 */
extern void gpio_enable_int_filter(int filter, int group);

extern int gpio_is_valid(int number);
extern int gpio_request(unsigned gpio, const char *label);
extern void gpio_free(unsigned gpio);
extern int gpio_direction_input(unsigned gpio);
extern int gpio_direction_output(unsigned gpio, int value);
extern void gpio_set_value(unsigned gpio, int value);
extern int gpio_get_value(unsigned gpio);


#ifdef CONFIG_EXGPIO
static inline int set_exgpio_mode(gpio_bank_t bank,int bit,gpio_mode_t mode)
{
    int bank_mode = get_configIO(bank);
    
    bank_mode &= ~(1 << bit);
    bank_mode |= mode << bit;
    configIO(bank, bank_mode);
    return 0;
}

static inline gpio_mode_t get_exgpio_mode(gpio_bank_t bank,int bit)
{
    return (get_configIO(bank) >> bit) & 1;
}

static inline int set_exgpio_val(gpio_bank_t bank,int bit,unsigned long val)
{
    int bank_val = getIO_level(bank);
    
    bank_val &= ~(1 << bit);
    bank_val |= val << bit;
    setIO_level(bank, bank_val);
    return 0;
}

static inline unsigned long  get_exgpio_val(gpio_bank_t bank,int bit)
{
    return (getIO_level(bank) >> bit) & 1;
}
#endif

#endif
