################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
build-462217313:
	@$(MAKE) --no-print-directory -Onone -f TOOLS/subdir_rules.mk build-462217313-inproc

build-462217313-inproc: ../TOOLS/app_ble.cfg
	@echo 'Building file: "$<"'
	@echo 'Invoking: XDCtools'
	"C:/ti/xdctools_3_50_04_43_core/xs" --xdcpath="C:/ti/ccsv7/ccs_base;/source/ti/ble5stack;" xdc.tools.configuro -o configPkg -t ti.targets.arm.elf.M3 -p ti.platforms.simplelink:CC2640R2F -r release -c "C:/ti/ccsv7/tools/compiler/ti-cgt-arm_16.9.6.LTS" --compileOptions "-mv7M3 --code_state=16 -me -O4 --opt_for_speed=0 --include_path=\"C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Projects/ble/CC2640r2f-BLEMouse-master/ble5_hid_gyromouse_app\" --include_path=\"C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Projects/ble/CC2640r2f-BLEMouse-master/ble5_hid_gyromouse_app/Application\" --include_path=\"C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Projects/ble/CC2640r2f-BLEMouse-master/ble5_hid_gyromouse_app/Startup\" --include_path=\"C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Projects/ble/CC2640r2f-BLEMouse-master/ble5_hid_gyromouse_app/PROFILES\" --include_path=\"C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Projects/ble/CC2640r2f-BLEMouse-master/ble5_hid_gyromouse_app/Include\" --include_path=\"/source/ti/ble5stack/controller/cc26xx_r2/inc\" --include_path=\"/source/ti/ble5stack/inc\" --include_path=\"/source/ti/ble5stack/common/cc26xx\" --include_path=\"/source/ti/ble5stack/icall/inc\" --include_path=\"/source/ti/ble5stack/target\" --include_path=\"/source/ti/ble5stack/hal/src/inc\" --include_path=\"/source/ti/ble5stack/hal/src/target/_common\" --include_path=\"/source/ti/ble5stack/hal/src/target/_common/cc26xx\" --include_path=\"/source/ti/ble5stack/heapmgr\" --include_path=\"/source/ti/ble5stack/icall/src/inc\" --include_path=\"/source/ti/ble5stack/osal/src/inc\" --include_path=\"/source/ti/ble5stack/services/src/saddr\" --include_path=\"/source/ti/ble5stack/services/src/sdata\" --include_path=\"/source/ti/devices/cc26x0r2\" --include_path=\"C:/ti/ccsv7/tools/compiler/ti-cgt-arm_16.9.6.LTS/include\" --define=NPI_USE_UART --define=BOARD_DISPLAY_USE_LCD=0 --define=BOARD_DISPLAY_USE_UART=1 --define=BOARD_DISPLAY_USE_UART_ANSI=1 --define=CC2640R2_LAUNCHXL --define=CC26XX --define=CC26XX_R2 --define=DEVICE_FAMILY=cc26x0r2 --define=Display_DISABLE_ALL --define=HEAPMGR_SIZE=0 --define=ICALL_EVENTS --define=ICALL_JT --define=ICALL_LITE --define=ICALL_MAX_NUM_ENTITIES=6 --define=ICALL_MAX_NUM_TASKS=6 --define=ICALL_STACK0_ADDR --define=POWER_SAVING --define=STACK_LIBRARY --define=USE_CORE_SDK --define=USE_ICALL --define=xdc_runtime_Assert_DISABLE_ALL --define=xdc_runtime_Log_DISABLE_ALL -g --c99 --gcc --diag_warning=225 --diag_wrap=off --display_error_number --gen_func_subsections=on --abi=eabi " "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

configPkg/linker.cmd: build-462217313 ../TOOLS/app_ble.cfg
configPkg/compiler.opt: build-462217313
configPkg/: build-462217313


