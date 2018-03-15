################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CFG_SRCS += \
C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Projects/ble/SimpleBLEPeripheral/CC26xx/CCS/Config/appBLE.cfg 


# Each subdirectory must supply rules for building sources it contributes
configPkg/linker.cmd: C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Projects/ble/SimpleBLEPeripheral/CC26xx/CCS/Config/appBLE.cfg
	@echo 'Building file: $<'
	@echo 'Invoking: XDCtools'
	"/xs" --xdcpath= xdc.tools.configuro -o configPkg -t ti.targets.arm.elf.M3 -p ti.platforms.simplelink:CC2640F128 -r release -c --compileOptions "$<"
	@echo 'Finished building: $<'
	@echo ' '


