obj-$(CONFIG_RTL8192CU_USB_WIFI)	+= rtl8192cu.o

rtl8192cu-objs	:= core/ieee80211.o \
								 core/rtw_cmd.o \
								 core/rtw_debug.o \
								 core/rtw_io.o \
								 core/rtw_ioctl_set.o \
								 core/rtw_mlme.o \
								 core/rtw_mlme_ext.o \
								 core/rtw_pwrctrl.o \
								 core/rtw_recv.o \
								 core/rtw_rf.o \
								 core/rtw_security.o \
								 core/rtw_sta_mgt.o \
								 core/rtw_wlan_util.o \
								 core/rtw_xmit.o \
								 core/efuse/rtl8712_efuse.o \
								 core/led/rtl8192c_led.o \
								 hal/hal_init.o \
								 hal/rtl8192c_d_hal_init.o \
								 hal/rtl8192c/rtl8192c_dm.o \
								 hal/rtl8192c/rtl8192c_phycfg.o \
								 hal/rtl8192c/rtl8192c_rf6052.o \
								 hal/rtl8192c/rtl8192c_rxdesc.o \
								 hal/rtl8192c/usb/Hal8192CUHWImg.o \
								 hal/rtl8192c/usb/rtl8192c_cmd.o \
								 hal/rtl8192c/usb/rtl8192cu_recv.o \
								 hal/rtl8192c/usb/rtl8192cu_xmit.o \
								 hal/rtl8192c/usb/usb_halinit.o \
								 hal/rtl8192c/usb/usb_ops_linux.o \
								 os_dep/osdep_service.o \
								 os_dep/linux/ioctl_linux.o \
								 os_dep/linux/mlme_linux.o \
								 os_dep/linux/os_intfs.o \
								 os_dep/linux/recv_linux.o \
								 os_dep/linux/usb_intf.o \
								 os_dep/linux/xmit_linux.o

EXTRA_CFLAGS	+= -Wno-unused-variable -Wno-unused-value -Wno-unused-label -Wno-unused-parameter -Wno-uninitialized -Wno-unused -Wno-unused-function