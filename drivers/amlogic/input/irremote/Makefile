#
# Makefile for the input core drivers.
#

# Each configuration option enables a list of files.

obj-$(CONFIG_VIRTUAL_REMOTE)		+= virtual_remote.o
obj-$(CONFIG_KEYPADS_APOLLO)		+= apollo_remote.o
apollo_remote-objs :=apollo_remote_kbd.o   apollo_sw_remote_kbd.o

ifeq ($(CONFIG_KEYPADS_APOLLO_DEBUG),y)
EXTRA_CFLAGS += -DDEBUG
endif
