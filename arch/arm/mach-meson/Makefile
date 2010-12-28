#
# Makefile for the linux kernel.
#

# Common support
obj-y := core.o clock.o common.o
obj-m :=
obj-n :=
obj-  :=

obj-$(CONFIG_CPU_FREQ) 		+= cpu.o
obj-$(CONFIG_ARCH_MESON)	+= pinmux.o
obj-$(CONFIG_ARCH_MESON)	+= gpio.o
obj-$(CONFIG_ARCH_MESON)	+= power_gate.o

obj-$(CONFIG_ARCH_MESON)	+= lm.o usbclock.o sataclock.o clk_set.o sram.o sleep.o

obj-$(CONFIG_MACH_MESON_8726M_REFA00)	+= board-8726m-REFA00.o

obj-$(CONFIG_MACH_MESON_8726M_REFB01)	+= board-8726m-REFB01.o
obj-$(CONFIG_MACH_MESON_8726M_REFB02)	+= board-8726m-REFB02.o
obj-$(CONFIG_MACH_MESON_8726M_REFB03)	+= board-8726m-REFB03.o
obj-$(CONFIG_MACH_MESON_8726M_REFB04)	+= board-8726m-REFB04.o
obj-$(CONFIG_MACH_MESON_8726M_REFB05)	+= board-8726m-REFB05.o
obj-$(CONFIG_MACH_MESON_8726M_REFB06)	+= board-8726m-REFB06.o
obj-$(CONFIG_MACH_MESON_8726M_REFB07)	+= board-8726m-REFB07.o
obj-$(CONFIG_MACH_MESON_8726M_REFB08)	+= board-8726m-REFB08.o
obj-$(CONFIG_MACH_MESON_8726M_REFB09)	+= board-8726m-REFB09.o

obj-$(CONFIG_MACH_MESON_8726M_REFC01)	+= board-8726m-REFC01.o
obj-$(CONFIG_MACH_MESON_8626M_REFC02)	+= board-8626m-REFC02.o
obj-$(CONFIG_MACH_MESON_8726M_REFC03)	+= board-8726m-REFC03.o

obj-$(CONFIG_MACH_MESON_6236M)	+= board-6236m.o
obj-$(CONFIG_MACH_MESON_8226M)	+= board-8226m.o

obj-$(CONFIG_MACH_MESON_8726M_DVBC_SKYWORTH)	+= board-8726m-dvbc-skyworth.o
obj-$(CONFIG_MACH_MESON_8726M_DVBC_HISENSE)	+= board-8726m-dvbc-hisense.o
obj-$(CONFIG_MACH_MESON_8726M_G7)	+= board-8726m-g7.o
obj-$(CONFIG_MACH_MESON_6236M_SH)	+= board-6236m-sh.o
obj-$(CONFIG_MACH_MESON_8626M_SH)	+= board-8626m-sh.o


# Power Management
obj-$(CONFIG_SUSPEND)			+= pm.o
obj-$(CONFIG_BCM_BT)	+= bcm-bt.o
