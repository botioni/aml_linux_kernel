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

#obj-y += nand/

obj-y += net/

obj-y += usb/

obj-$(CONFIG_SATA_DWC_AHCI) += sata/

obj-y += canvas/

obj-y += amports/

obj-y += i2c/

obj-y += input/

obj-y += spi_nor/

obj-$(CONFIG_CMEM) += cmem/

obj-$(CONFIG_JPEGLOGO) += jpeglogo/

obj-$(CONFIG_CARDREADER) += cardreader/


obj-$(CONFIG_AML_AUDIO_DSP) += audiodsp/

#####Sound system is later init in kernel default..######################
obj-y += sound/

obj-$(CONFIG_CLOCK600) += clock600/

