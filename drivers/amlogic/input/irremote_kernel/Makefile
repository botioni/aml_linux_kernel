#
# Makefile for the input core drivers.
#

# Each configuration option enables a list of files.
ifeq ($(CONFIG_KEYPADS_APOLLO_DECODER_HW),y)
obj-$(CONFIG_KEYPADS_APOLLO)		+= apollo_remote_kbd.o
else
obj-$(CONFIG_KEYPADS_APOLLO)		+= apollo_sw_remote_kbd.o
endif

ifeq ($(CONFIG_KEYPADS_APOLLO_DEBUG),y)
EXTRA_CFLAGS += -DDEBUG
endif
