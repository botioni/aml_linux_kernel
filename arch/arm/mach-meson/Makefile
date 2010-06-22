#
# Makefile for the linux kernel.
#

# Common support
obj-y := core.o clock.o
obj-m :=
obj-n :=
obj-  :=

obj-$(CONFIG_CPU_FREQ) 			+= cpu.o
obj-$(CONFIG_MACH_MESON)		+= board-m1.o
obj-$(CONFIG_MACH_MESON)		+= pinmux.o

