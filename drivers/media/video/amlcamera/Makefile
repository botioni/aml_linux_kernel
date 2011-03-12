#
# Makefile for the amlogic platform camera interface device drivers.
#

gt2005dri-objs := gt2005.o
gc0308dri-objs := gc0308.o
amlvm-objs := common/vm.o common/plat_ctrl.o

obj-$(CONFIG_VIDEO_AMLOGIC_CAPTURE) += gt2005dri.o gc0308dri.o
obj-$(CONFIG_AMLOGIC_VIDEOIN_MANAGER)  += amlvm.o
