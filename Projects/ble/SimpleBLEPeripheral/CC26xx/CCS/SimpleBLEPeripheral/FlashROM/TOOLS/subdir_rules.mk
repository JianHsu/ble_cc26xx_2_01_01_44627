################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
build-1846563311:
	@$(MAKE) --no-print-directory -Onone -f TOOLS/subdir_rules.mk build-1846563311-inproc

build-1846563311-inproc: C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Projects/ble/SimpleBLEPeripheral/CC26xx/CCS/Config/appBLE.cfg
	@echo 'Building file: "$<"'
	@echo 'Invoking: XDCtools'
	"C:/ti/xdctools_3_31_01_33_core/xs" --xdcpath="C:/ti/tirtos_simplelink_2_13_00_06/packages;C:/ti/tirtos_simplelink_2_13_00_06/products/bios_6_42_00_08/packages;C:/ti/tirtos_simplelink_2_13_00_06/products/uia_2_00_02_39/packages;C:/ti/ccsv7/ccs_base;" xdc.tools.configuro -o configPkg -t ti.targets.arm.elf.M3 -p ti.platforms.simplelink:CC2640F128 -r release -c "C:/ti/ccsv7/tools/compiler/ti-cgt-arm_16.9.6.LTS" --compileOptions "-mv7M3 --code_state=16 --abi=eabi -me -O4 --opt_for_speed=0 --include_path=\"C:/ti/ccsv7/tools/compiler/ti-cgt-arm_16.9.6.LTS/include\" --include_path=\"C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Projects/ble/SimpleBLEPeripheral/CC26xx/Source/Application\" --include_path=\"C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Projects/ble/include\" --include_path=\"C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Projects/ble/ICall/Include\" --include_path=\"C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Projects/ble/Profiles/Roles/CC26xx\" --include_path=\"C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Projects/ble/Profiles/Roles\" --include_path=\"C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Projects/ble/Profiles/DevInfo\" --include_path=\"C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Projects/ble/Profiles/SimpleProfile/CC26xx\" --include_path=\"C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Projects/ble/Profiles/SimpleProfile\" --include_path=\"C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Projects/ble/common/cc26xx\" --include_path=\"C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Components/applib/heap\" --include_path=\"C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Components/ble/hci\" --include_path=\"C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Components/ble/controller/CC26xx/include\" --include_path=\"C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Components/ble/host\" --include_path=\"C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Components/hal/target/CC2650TIRTOS\" --include_path=\"C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Components/hal/target/_common/cc26xx\" --include_path=\"C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Components/hal/include\" --include_path=\"C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Components/osal/include\" --include_path=\"C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Components/services/sdata\" --include_path=\"C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Components/services/saddr\" --include_path=\"C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Components/icall/include\" --include_path=\"C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Components/ble/include\" --include_path=\"C:/ti/tirtos_simplelink_2_13_00_06/products/cc26xxware_2_21_01_15600\" --include_path=\"C:/ti/tirtos_simplelink_2_13_00_06/packages/ti/boards/SRF06EB/CC2650EM_7ID\" -g --gcc --define=USE_ICALL --define=POWER_SAVING --define=SBP_TASK_STACK_SIZE=700 --define=GAPROLE_TASK_STACK_SIZE=520 --define=HEAPMGR_SIZE=2672 --define=TI_DRIVERS_SPI_DMA_INCLUDED --define=TI_DRIVERS_LCD_INCLUDED --define=ICALL_MAX_NUM_TASKS=3 --define=ICALL_MAX_NUM_ENTITIES=6 --define=xdc_runtime_Assert_DISABLE_ALL --define=xdc_runtime_Log_DISABLE_ALL --define=MAX_NUM_BLE_CONNS=1 --define=CC26XXWARE --define=CC26XX --diag_warning=225 --diag_suppress=48 --diag_wrap=off --display_error_number --gen_func_subsections=on " "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

configPkg/linker.cmd: build-1846563311 C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Projects/ble/SimpleBLEPeripheral/CC26xx/CCS/Config/appBLE.cfg
configPkg/compiler.opt: build-1846563311
configPkg/: build-1846563311


