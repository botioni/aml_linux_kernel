#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>
#include <linux/fs.h>
#include <mach/am_regs.h>
#include <asm/fiq.h>
#include <asm/uaccess.h>

static u8 fiq_init_flag = 0;

/* 4K size of FIQ stack size */
static u8 fiq_stack[4096];

typedef void (*fiq_routine)(void);

static u8 fiq_index[4] = {0xff, 0xff, 0xff, 0xff};
static fiq_routine fiq_func[4] = {NULL, NULL, NULL, NULL};

static void __attribute__ ((naked)) fiq_vector(void)
{
	asm __volatile__ ("mov pc, r8 ;");
}

static void __attribute__ ((naked)) fiq_isr(void)
{
    int i;

    asm __volatile__ (
	"mov    ip, sp;\n"
	"stmfd	sp!, {r0-r12, lr};\n"
	"sub    sp, sp, #256;\n"
	"sub    fp, sp, #256;\n");
	
	for (i=0;i<4;i++){
    	if ((fiq_index[i]!=0xff)&&(fiq_func[i]!=NULL)){
    	    if (IRQ_STATUS_REG(fiq_index[i])&(1<<IRQ_BIT(fiq_index[i])))
    	        fiq_func[i]();
    	}
    }

    dsb();
    asm __volatile__ (
	"add	sp, sp, #256 ;\n"
	"ldmia	sp!, {r0-r12, lr};\n"
	"subs	pc, lr, #4;\n");
}

void init_fiq(void)
{
	struct pt_regs regs;
    int i;
	/* prep the special FIQ mode regs */
	for (i=0;i<4;i++){
	    fiq_index[i] = 0xff;
	    fiq_func[i] = NULL;
	}
	memset(&regs, 0, sizeof(regs));
	regs.ARM_r8 = (unsigned long)fiq_isr;
	regs.ARM_sp = (unsigned long)fiq_stack + sizeof(fiq_stack) - 4;
	set_fiq_regs(&regs);
	set_fiq_handler(fiq_vector, 8);
	fiq_init_flag = 1;
	printk("init FIQ\n");
}
EXPORT_SYMBOL(init_fiq);

void request_fiq(int fiq, void (*isr)(void))
{
	int i;
	unsigned int mask = 1 << IRQ_BIT(fiq);

    if (!fiq_init_flag)
        init_fiq();
    for (i=0;i<4;i++){
        if (fiq_index[i]==0xff){
            printk("request FIQ %d %x\n", fiq, (unsigned)isr);
            disable_fiq(fiq);
            fiq_index[i] = fiq;
            fiq_func[i] = isr;
        	SET_CBUS_REG_MASK(IRQ_FIQSEL_REG(fiq), mask);
        	printk("enable FIQ %d\n", fiq);
        	enable_fiq(fiq);
            break;
        }
    }
}
EXPORT_SYMBOL(request_fiq);

void free_fiq(int fiq)
{
    int i;
	unsigned int mask = 1 << IRQ_BIT(fiq);

    for (i=0;i<4;i++){
        if (fiq_index[i] == fiq){
            disable_fiq(fiq);
            CLEAR_CBUS_REG_MASK(IRQ_FIQSEL_REG(fiq), mask);
            fiq_index[i] = 0xff;
            fiq_func[i] = NULL;
        }
    }
}
EXPORT_SYMBOL(free_fiq);
