#
_XDCBUILDCOUNT = 
ifneq (,$(findstring path,$(_USEXDCENV_)))
override XDCPATH = E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source;E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/kernel/tirtos/packages;E:/ti/ccsv7/ccs_base;E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack
override XDCROOT = E:/ti/xdctools_3_50_01_12_core
override XDCBUILDCFG = ./config.bld
endif
ifneq (,$(findstring args,$(_USEXDCENV_)))
override XDCARGS = 
override XDCTARGETS = 
endif
#
ifeq (0,1)
PKGPATH = E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source;E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/kernel/tirtos/packages;E:/ti/ccsv7/ccs_base;E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack;E:/ti/xdctools_3_50_01_12_core/packages;..
HOSTOS = Windows
endif
