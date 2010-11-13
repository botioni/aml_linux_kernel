##########################################
######This Amlogic Drivers#######################

##########################################
##	THis the Boot Sequence	Do not changed  unbending
##########################################

###For Display, this is for quick logo show##############
obj-y += display/

#obj-y  += hdmi/


#####for Basic drivers#####


obj-y += uart/

obj-y += nand/

obj-y += net/

obj-y += usb/

obj-$(CONFIG_SATA_DWC_AHCI) += sata/

obj-y += canvas/

obj-y += amports/

obj-y += i2c/

obj-$(CONFIG_TCA6424) += extern_io/

obj-y += input/

obj-y += spi_nor/

obj-$(CONFIG_CMEM) += cmem/

obj-$(CONFIG_JPEGLOGO) += jpeglogo/

obj-$(CONFIG_CARDREADER) += cardreader/

obj-$(CONFIG_MALI) += mali/

obj-$(CONFIG_AML_AUDIO_DSP) += audiodsp/

obj-$(CONFIG_AML_RTC)+= rtc/

obj-$(CONFIG_AMLOGIC_PM)+= power/

obj-y += tvin/

#####Sound system is later init in kernel default..######################
obj-y += sound/

obj-y += hdmi/

obj-$(CONFIG_AM_DVB) += dvb/

obj-y += wifi/
