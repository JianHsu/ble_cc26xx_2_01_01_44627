################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
ICall/ICall.obj: C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Components/icall/ports/tirtos/ICall.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: "$<"'
	@echo 'Invoking: ARM Compiler'
	"C:/ti/ccsv7/tools/compiler/ti-cgt-arm_16.9.6.LTS/bin/armcl" --cmd_file="C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Projects/ble/SensorTag/CC26xx/CCS/SensorTag/../../IAR/Application/CC2650/../../../CCS/Config/ccsCompilerDefines.bcfg"  -mv7M3 --code_state=16 --abi=eabi -me --include_path="C:/ti/ccsv7/tools/compiler/ti-cgt-arm_16.9.6.LTS/include" --include_path="C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Projects/ble/SensorTag/CC26xx/Source/Application" --include_path="C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Projects/ble/include" --include_path="C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Projects/ble/ICall/Include" --include_path="C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Projects/ble/Profiles/Roles/CC26xx" --include_path="C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Projects/ble/Profiles/Roles" --include_path="C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Projects/ble/Profiles/Keys" --include_path="C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Projects/ble/Profiles/DevInfo" --include_path="C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Projects/ble/Profiles/SensorProfile/cc26xx" --include_path="C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Projects/ble/Profiles/OAD/cc26xxST" --include_path="C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Projects/ble/common/cc26xx" --include_path="C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Components/applib/heap" --include_path="C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Components/ble/hci" --include_path="C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Components/ble/controller/CC26xx/include" --include_path="C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Components/ble/host" --include_path="C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Components/hal/target/CC2650TIRTOS" --include_path="C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Components/hal/target/_common/cc26xx" --include_path="C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Components/hal/include" --include_path="C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Components/osal/include" --include_path="C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Components/services/sdata" --include_path="C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Components/services/saddr" --include_path="C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Components/icall/include" --include_path="C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Components/icall/ports/tirtos" --include_path="C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Components/ble/include" --include_path="C:/ti/tirtos_simplelink_2_13_00_06/products/cc26xxware_2_21_01_15600" --include_path="C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Projects/ble/SensorTag/CC26xx/Source/Application/Board_patch/interfaces" --include_path="C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Projects/ble/SensorTag/CC26xx/Source/Application/Board_patch/devices" --include_path="C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Projects/ble/SensorTag/CC26xx/Source/Application/Board_patch/CC26XXST_0120" --define=POWER_SAVING --define=GATT_TI_UUID_128_BIT --define=FEATURE_REGISTER_SERVICE --define=FEATURE_OAD --define=FACTORY_IMAGE --define=USE_ICALL --define=TI_DRIVERS_PIN_INCLUDED --define=TI_DRIVERS_I2C_INCLUDED --define=TI_DRIVERS_SPI_INCLUDED --define=GAPROLE_TASK_STACK_SIZE=550 --define=HEAPMGR_SIZE=2872 --define=ICALL_MAX_NUM_TASKS=8 --define=ICALL_MAX_NUM_ENTITIES=11 --define=xdc_runtime_Assert_DISABLE_ALL --define=xdc_runtime_Log_DISABLE_ALL --define=CC26XXWARE --define=CC26XX --diag_warning=225 --diag_suppress=48 --diag_wrap=off --display_error_number --gen_func_subsections=on --preproc_with_compile --preproc_dependency="ICall/ICall.d_raw" --obj_directory="ICall" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

ICall/ICallCC2650.obj: C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Components/icall/ports/tirtos/ICallCC2650.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: "$<"'
	@echo 'Invoking: ARM Compiler'
	"C:/ti/ccsv7/tools/compiler/ti-cgt-arm_16.9.6.LTS/bin/armcl" --cmd_file="C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Projects/ble/SensorTag/CC26xx/CCS/SensorTag/../../IAR/Application/CC2650/../../../CCS/Config/ccsCompilerDefines.bcfg"  -mv7M3 --code_state=16 --abi=eabi -me --include_path="C:/ti/ccsv7/tools/compiler/ti-cgt-arm_16.9.6.LTS/include" --include_path="C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Projects/ble/SensorTag/CC26xx/Source/Application" --include_path="C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Projects/ble/include" --include_path="C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Projects/ble/ICall/Include" --include_path="C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Projects/ble/Profiles/Roles/CC26xx" --include_path="C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Projects/ble/Profiles/Roles" --include_path="C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Projects/ble/Profiles/Keys" --include_path="C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Projects/ble/Profiles/DevInfo" --include_path="C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Projects/ble/Profiles/SensorProfile/cc26xx" --include_path="C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Projects/ble/Profiles/OAD/cc26xxST" --include_path="C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Projects/ble/common/cc26xx" --include_path="C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Components/applib/heap" --include_path="C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Components/ble/hci" --include_path="C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Components/ble/controller/CC26xx/include" --include_path="C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Components/ble/host" --include_path="C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Components/hal/target/CC2650TIRTOS" --include_path="C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Components/hal/target/_common/cc26xx" --include_path="C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Components/hal/include" --include_path="C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Components/osal/include" --include_path="C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Components/services/sdata" --include_path="C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Components/services/saddr" --include_path="C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Components/icall/include" --include_path="C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Components/icall/ports/tirtos" --include_path="C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Components/ble/include" --include_path="C:/ti/tirtos_simplelink_2_13_00_06/products/cc26xxware_2_21_01_15600" --include_path="C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Projects/ble/SensorTag/CC26xx/Source/Application/Board_patch/interfaces" --include_path="C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Projects/ble/SensorTag/CC26xx/Source/Application/Board_patch/devices" --include_path="C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Projects/ble/SensorTag/CC26xx/Source/Application/Board_patch/CC26XXST_0120" --define=POWER_SAVING --define=GATT_TI_UUID_128_BIT --define=FEATURE_REGISTER_SERVICE --define=FEATURE_OAD --define=FACTORY_IMAGE --define=USE_ICALL --define=TI_DRIVERS_PIN_INCLUDED --define=TI_DRIVERS_I2C_INCLUDED --define=TI_DRIVERS_SPI_INCLUDED --define=GAPROLE_TASK_STACK_SIZE=550 --define=HEAPMGR_SIZE=2872 --define=ICALL_MAX_NUM_TASKS=8 --define=ICALL_MAX_NUM_ENTITIES=11 --define=xdc_runtime_Assert_DISABLE_ALL --define=xdc_runtime_Log_DISABLE_ALL --define=CC26XXWARE --define=CC26XX --diag_warning=225 --diag_suppress=48 --diag_wrap=off --display_error_number --gen_func_subsections=on --preproc_with_compile --preproc_dependency="ICall/ICallCC2650.d_raw" --obj_directory="ICall" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '


