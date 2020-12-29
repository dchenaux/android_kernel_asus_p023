#
# Makefile for the rest of the USB drivers
# (the ones that don't fit into any other categories)
#
ifeq ($(TARGET_BUILD_VARIANT),eng)
	KBUILD_CFLAGS += -D_ENG_BUILD_
else ifeq ($(TARGET_BUILD_VARIANT),user)
	KBUILD_CFLAGS += -D_USER_BUILD_
else ifeq ($(TARGET_BUILD_VARIANT),userdebug)
	KBUILD_CFLAGS += -D_USERDEBUG_BUILD_
else
	KBUILD_CFLAGS += -D_OTHER_BUILD_
endif

obj-$(CONFIG_USB_ADUTUX)		+= adutux.o
obj-$(CONFIG_USB_APPLEDISPLAY)		+= appledisplay.o
obj-$(CONFIG_USB_CYPRESS_CY7C63)	+= cypress_cy7c63.o
obj-$(CONFIG_USB_CYTHERM)		+= cytherm.o
obj-$(CONFIG_USB_EMI26)			+= emi26.o
obj-$(CONFIG_USB_EMI62)			+= emi62.o
obj-$(CONFIG_USB_EZUSB_FX2)		+= ezusb.o
obj-$(CONFIG_USB_FTDI_ELAN)		+= ftdi-elan.o
obj-$(CONFIG_USB_IDMOUSE)		+= idmouse.o
obj-$(CONFIG_USB_IOWARRIOR)		+= iowarrior.o
obj-$(CONFIG_USB_ISIGHTFW)		+= isight_firmware.o
obj-$(CONFIG_USB_LCD)			+= usblcd.o
obj-$(CONFIG_USB_LD)			+= ldusb.o
obj-$(CONFIG_USB_LED)			+= usbled.o
obj-$(CONFIG_USB_LEGOTOWER)		+= legousbtower.o
obj-$(CONFIG_USB_RIO500)		+= rio500.o
obj-$(CONFIG_USB_TEST)			+= usbtest.o
obj-$(CONFIG_USB_EHSET_TEST_FIXTURE)    += ehset.o
obj-$(CONFIG_USB_TRANCEVIBRATOR)	+= trancevibrator.o
obj-$(CONFIG_USB_USS720)		+= uss720.o
obj-$(CONFIG_USB_SEVSEG)		+= usbsevseg.o
obj-$(CONFIG_USB_YUREX)			+= yurex.o
obj-$(CONFIG_USB_HSIC_USB3503)		+= usb3503.o
obj-$(CONFIG_USB_ASUS_AUDIO_STAND)	+= asus_audio_stand.o

obj-$(CONFIG_USB_SISUSBVGA)		+= sisusbvga/
