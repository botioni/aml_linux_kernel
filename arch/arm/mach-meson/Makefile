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

obj-$(CONFIG_MACH_MESON_6236M)	+= board-6236m.o
obj-$(CONFIG_MACH_MESON_8626M)	+= board-8626m.o
obj-$(CONFIG_MACH_MESON_6236M_SH)	+= board-6236m-sh.o
obj-$(CONFIG_MACH_MESON_8626M_SH)	+= board-8626m-sh.o
obj-$(CONFIG_MACH_MESON_8226M)	+= board-8226m.o
obj-$(CONFIG_MACH_MESON_8726M)	+= board-8726m.o
obj-$(CONFIG_MACH_MESON_8726M_ARM)	+= board-8726m-arm.o
obj-$(CONFIG_MACH_MESON_8726M_DVBC)	+= board-8726m-dvbc.o
obj-$(CONFIG_MACH_MESON_8726M_ML)	+= board-8726m-ml.o
obj-$(CONFIG_MACH_MESON_8726M_MLMBX)	+= board-8726m-mlmbx.o
obj-$(CONFIG_MACH_MESON_8726M_W10)	+= board-8726m-w10.o
obj-$(CONFIG_MACH_MESON_8726M_W7)	+= board-8726m-w7.o
obj-$(CONFIG_MACH_MESON_8726M_W8)	+= board-8726m-w8.o
obj-$(CONFIG_MACH_MESON_8726M_K7)	+= board-8726m-k7.o
obj-$(CONFIG_MACH_MESON_8726M_C7)	+= board-8726m-c7.o
obj-$(CONFIG_MACH_MESON_8726M_K8)	+= board-8726m-k8.o
obj-$(CONFIG_MACH_MESON_8726M_DVBC_SKYWORTH)	+= board-8726m-dvbc-skyworth.o
obj-$(CONFIG_MACH_MESON_8726M_DVBC_HISENSE)	+= board-8726m-dvbc-hisense.o
# Power Management
obj-$(CONFIG_SUSPEND)			+= pm.o
obj-$(CONFIG_BCM_BT)	+= bcm-bt.o
