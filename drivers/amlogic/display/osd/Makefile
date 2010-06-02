#
# Makefile for the AML Apollo framebuffer device driver
#


obj-$(CONFIG_FB_AML_APOLLO) += apollofb.o 
apollofb-objs := apollodev.o 

ifeq ($(CONFIG_AMLOGIC_BOARD_APOLLO_H),y)
apollofb-objs +=  osd_a1h.o apollo_h_main.o
else
apollofb-objs +=   osd.o apollo_main.o
endif

EXTRA_CFLAGS += -Idrivers/amlogic/display/	
EXTRA_CFLAGS += -Idrivers/amlogic/display/ge2d

#apollofb-y    := apollo_main.o apollodev.o  osd.o 


