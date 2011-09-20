#
# Makefile for the linux kernel.
#

# Common support
obj-y := core.o clock.o common.o fiq.o fiq_bridge.o
obj-m :=
obj-n :=
obj-  :=

obj-$(CONFIG_CPU_FREQ) 		+= cpu.o
obj-$(CONFIG_ARCH_MESON3)	+= pinmux.o
obj-$(CONFIG_ARCH_MESON3)	+= gpio.o
obj-$(CONFIG_ARCH_MESON3)	+= power_gate.o
obj-$(CONFIG_ARCH_MESON3)	+= sd_pad.o
obj-$(CONFIG_AML_SUSPEND) += power_suspend.o
obj-$(CONFIG_ARCH_MESON3)	+= lm.o usbclock.o clk_set.o sram.o sleep.o

obj-$(CONFIG_MACH_MESON3_REFF01)	+= board-m3-reff01.o
obj-$(CONFIG_MACH_MESON3_REFF02)	+= board-m3-reff02.o
obj-$(CONFIG_MACH_MESON3_REFF03)	+= board-m3-reff03.o
obj-$(CONFIG_MACH_MESON3_REFF04)	+= board-m3-reff04.o
obj-$(CONFIG_MACH_MESON3_REFF05)	+= board-m3-reff05.o
obj-$(CONFIG_MACH_MESON3_REFF07)	+= board-m3-reff07.o

# Power Management
obj-$(CONFIG_SUSPEND)			+= pm.o
