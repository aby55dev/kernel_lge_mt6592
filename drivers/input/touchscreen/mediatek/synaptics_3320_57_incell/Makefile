include $(srctree)/drivers/misc/mediatek/Makefile.custom


ccflags-y += -I$(MTK_PROJECT_PATH_ROOT)/touchpanel/synaptics_3320_57_incell/

# Linux driver folder
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/$(MTK_PLATFORM)/$(ARCH_MTK_PROJECT)/synaptics_3320_57_incell/
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/


ifeq ($(CONFIG_MTK_PLATFORM),"mt6582")
    ccflags-y += -DMT6582
endif
obj-y +=  lgtp_common_driver.o
obj-y +=  lgtp_device_td4191.o
obj-y +=  lgtp_model_config.o
obj-y +=  lgtp_platform_api.o
obj-y +=  touch_synaptics_ds5_fw_upgrade.o
obj-y +=  RefCode_CustomerImplementation.o
obj-y +=  RefCode_F54.o