#
#  Copyright (c) 2013, ASUSTek, Inc. All Rights Reserved.
#  Written by chris chang chris1_chang@asus.com
#
ifeq ($(TARGET_BUILD_VARIANT),eng)
    KBUILD_CFLAGS += -DME372CG_ENG_BUILD
else ifeq ($(TARGET_BUILD_VARIANT),user)
    KBUILD_CFLAGS += -DME372CG_USER_BUILD
else ifeq ($(TARGET_BUILD_VARIANT),userdebug)
    KBUILD_CFLAGS += -DME372CG_USERDEBUG_BUILD
else
    KBUILD_CFLAGS += -DME372CG_OTHER_BUILD
endif

obj-y += ti/
obj-$(CONFIG_Z380C) += battery_others/
obj-$(CONFIG_Z380C) += battery_smb347.o
obj-$(CONFIG_Z300C)  += z300c_smb347.o
obj-$(CONFIG_Z300CG) += z300c_smb347.o
