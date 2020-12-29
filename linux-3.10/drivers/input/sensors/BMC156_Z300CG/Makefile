#
# Makefile for Bosch sensor driver.
#
obj-y += bstclass.o

#obj-$(CONFIG_SENSORS_BMA2X2)	+= bma2x2.o
obj-y	+= bma2x2.o

#ifeq ($(CONFIG_BOSCH_BMA2X2_ENABLE_INT2),y)
	EXTRA_CFLAGS += -DBMA2X2_ENABLE_INT2
#endif

#obj-$(CONFIG_SENSORS_BMM050)    += bmm050_driver.o bmm050.o
obj-y    += bmm050_driver.o bmm050.o
EXTRA_CFLAGS += -DBMM_USE_BASIC_I2C_FUNC -DCONFIG_BMM_USE_PLATFORM_DATA -DCONFIG_DOUBLE_TAP





