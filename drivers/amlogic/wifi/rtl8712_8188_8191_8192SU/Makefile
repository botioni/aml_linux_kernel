obj-$(CONFIG_RTL8712_8188_8191_8192SU_USB_WIFI)	+= rtl8712.o

rtl8712-objs	:= cmd/rtl8712_cmd.o \
								 cmd/rtl871x_cmd.o \
								 crypto/rtl871x_security.o \
								 debug/rtl871x_debug.o \
								 eeprom/rtl871x_eeprom.o \
								 efuse/rtl8712_efuse.o \
								 hal/rtl8712/hal_init.o \
								 hal/rtl8712/usb_halinit.o \
								 hal/rtl8712/usb_ops.o \
								 hal/rtl8712/usb_ops_linux.o \
								 io/rtl8712_io.o \
								 io/rtl871x_io.o \
								 ioctl/rtl871x_ioctl_linux.o \
								 ioctl/rtl871x_ioctl_query.o \
								 ioctl/rtl871x_ioctl_rtl.o \
								 ioctl/rtl871x_ioctl_set.o \
								 led/rtl8712_led.o \
								 mlme/ieee80211.o \
								 mlme/rtl871x_mlme.o \
								 mp/rtl871x_mp.o \
								 mp/rtl871x_mp_ioctl.o \
								 os_dep/linux/cmd_linux.o \
								 os_dep/linux/io_linux.o \
								 os_dep/linux/mlme_linux.o \
								 os_dep/linux/recv_linux.o \
								 os_dep/linux/xmit_linux.o \
								 os_intf/osdep_service.o \
								 os_intf/linux/os_intfs.o \
								 os_intf/linux/usb_intf.o \
								 pwrctrl/rtl871x_pwrctrl.o \
								 recv/rtl8712_recv.o \
								 recv/rtl871x_recv.o \
								 rf/rtl8712_rf.o \
								 rf/rtl871x_rf.o \
								 sta_mgt/rtl871x_sta_mgt.o \
								 xmit/rtl8712_xmit.o \
								 xmit/rtl871x_xmit.o

EXTRA_CFLAGS	+= -Wno-unused-variable -Wno-unused-value -Wno-unused-label -Wno-unused-parameter -Wno-uninitialized -Wno-unused -Wno-unused-function