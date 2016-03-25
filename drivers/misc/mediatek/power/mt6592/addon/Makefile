include $(srctree)/drivers/misc/mediatek/Makefile.custom
ccflags-y +=-I$(MTK_PATH_PLATFORM)/drivers/power/

ifeq ($(CONFIG_MTK_BQ24196_SUPPORT),y) 
  obj-$(CONFIG_MTK_SMART_BATTERY) += charging_hw_bq24196.o bq24196.o tbl_helper_func.o
else
  ifeq ($(CONFIG_MTK_BQ24297_SUPPORT),y)
    obj-$(CONFIG_MTK_SMART_BATTERY) += charging_hw_bq24297.o bq24297.o tbl_helper_func.o
  endif
endif