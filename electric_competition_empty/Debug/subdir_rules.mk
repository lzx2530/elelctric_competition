################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
%.o: ../%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Arm Compiler - building file: "$<"'
	"C:/ti/ccs2100/ccs/tools/compiler/ti-cgt-armllvm_5.1.1.LTS/bin/tiarmclang.exe" -c @"device.opt"  -march=thumbv6m -mcpu=cortex-m0plus -mfloat-abi=soft -mlittle-endian -mthumb -O0 -I"D:/ccs/elelctric_competition/electric_competition_empty/bsp" -I"D:/ccs/elelctric_competition/electric_competition_empty/protocol" -I"D:/ccs/elelctric_competition/electric_competition_empty/drivers" -I"D:/ccs/elelctric_competition/electric_competition_empty/common" -I"D:/ccs/elelctric_competition/electric_competition_empty/app" -I"D:/ccs/elelctric_competition/electric_competition_empty/algo" -I"D:/ccs/elelctric_competition/electric_competition_empty" -I"D:/ccs/elelctric_competition/electric_competition_empty/Debug" -I"D:/ccs/mspm0_sdk/mspm0_sdk_2_10_00_04/source/third_party/CMSIS/Core/Include" -I"D:/ccs/mspm0_sdk/mspm0_sdk_2_10_00_04/source" -gdwarf-3 -Wall -MMD -MP -MF"$(basename $(<F)).d_raw" -MT"$(@)"  $(GEN_OPTS__FLAG) -o"$@" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

build-245789528: ../empty.syscfg
	@echo 'SysConfig - building file: "$<"'
	"C:/ti/ccs2100/ccs/utils/sysconfig_1.28.0/sysconfig_cli.bat" -s "D:/ccs/mspm0_sdk/mspm0_sdk_2_10_00_04/.metadata/product.json" --script "D:/ccs/elelctric_competition/electric_competition_empty/empty.syscfg" -o "." --compiler ticlang
	@echo 'Finished building: "$<"'
	@echo ' '

device_linker.cmd: build-245789528 ../empty.syscfg
device.opt: build-245789528
device.cmd.genlibs: build-245789528
ti_msp_dl_config.c: build-245789528
ti_msp_dl_config.h: build-245789528
Event.dot: build-245789528

%.o: ./%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Arm Compiler - building file: "$<"'
	"C:/ti/ccs2100/ccs/tools/compiler/ti-cgt-armllvm_5.1.1.LTS/bin/tiarmclang.exe" -c @"device.opt"  -march=thumbv6m -mcpu=cortex-m0plus -mfloat-abi=soft -mlittle-endian -mthumb -O0 -I"D:/ccs/elelctric_competition/electric_competition_empty/bsp" -I"D:/ccs/elelctric_competition/electric_competition_empty/protocol" -I"D:/ccs/elelctric_competition/electric_competition_empty/drivers" -I"D:/ccs/elelctric_competition/electric_competition_empty/common" -I"D:/ccs/elelctric_competition/electric_competition_empty/app" -I"D:/ccs/elelctric_competition/electric_competition_empty/algo" -I"D:/ccs/elelctric_competition/electric_competition_empty" -I"D:/ccs/elelctric_competition/electric_competition_empty/Debug" -I"D:/ccs/mspm0_sdk/mspm0_sdk_2_10_00_04/source/third_party/CMSIS/Core/Include" -I"D:/ccs/mspm0_sdk/mspm0_sdk_2_10_00_04/source" -gdwarf-3 -Wall -MMD -MP -MF"$(basename $(<F)).d_raw" -MT"$(@)"  $(GEN_OPTS__FLAG) -o"$@" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

startup_mspm0g350x_ticlang.o: D:/ccs/mspm0_sdk/mspm0_sdk_2_10_00_04/source/ti/devices/msp/m0p/startup_system_files/ticlang/startup_mspm0g350x_ticlang.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Arm Compiler - building file: "$<"'
	"C:/ti/ccs2100/ccs/tools/compiler/ti-cgt-armllvm_5.1.1.LTS/bin/tiarmclang.exe" -c @"device.opt"  -march=thumbv6m -mcpu=cortex-m0plus -mfloat-abi=soft -mlittle-endian -mthumb -O0 -I"D:/ccs/elelctric_competition/electric_competition_empty/bsp" -I"D:/ccs/elelctric_competition/electric_competition_empty/protocol" -I"D:/ccs/elelctric_competition/electric_competition_empty/drivers" -I"D:/ccs/elelctric_competition/electric_competition_empty/common" -I"D:/ccs/elelctric_competition/electric_competition_empty/app" -I"D:/ccs/elelctric_competition/electric_competition_empty/algo" -I"D:/ccs/elelctric_competition/electric_competition_empty" -I"D:/ccs/elelctric_competition/electric_competition_empty/Debug" -I"D:/ccs/mspm0_sdk/mspm0_sdk_2_10_00_04/source/third_party/CMSIS/Core/Include" -I"D:/ccs/mspm0_sdk/mspm0_sdk_2_10_00_04/source" -gdwarf-3 -Wall -MMD -MP -MF"$(basename $(<F)).d_raw" -MT"$(@)"  $(GEN_OPTS__FLAG) -o"$@" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '


