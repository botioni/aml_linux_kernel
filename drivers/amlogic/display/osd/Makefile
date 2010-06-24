#
# Makefile for the AML Apollo framebuffer device driver
#


obj-$(CONFIG_FB_AM) += apollofb.o 
apollofb-objs := apollodev.o 


apollofb-objs +=  osd_a1h.o apollo_h_main.o


EXTRA_CFLAGS += -Idrivers/amlogic/display/	
EXTRA_CFLAGS += -Idrivers/amlogic/display/ge2d

#apollofb-y    := apollo_main.o apollodev.o  osd.o 


