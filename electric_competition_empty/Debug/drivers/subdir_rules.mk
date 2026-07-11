################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
drivers/%.o: ../drivers/%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Arm Compiler - building file: "$<"'
	"C:/ti/ccs2100/ccs/tools/compiler/ti-cgt-armllvm_5.1.1.LTS/bin/tiarmclang.exe" -c @"device.opt"  -march=thumbv6m -mcpu=cortex-m0plus -mfloat-abi=soft -mlittle-endian -mthumb -O0 -I"D:/ccs/elelctric_competition/electric_competition_empty/bsp" -I"D:/ccs/elelctric_competition/electric_competition_empty/protocol" -I"D:/ccs/elelctric_competition/electric_competition_empty/drivers" -I"D:/ccs/elelctric_competition/electric_competition_empty/common" -I"D:/ccs/elelctric_competition/electric_competition_empty/app" -I"D:/ccs/elelctric_competition/electric_competition_empty/algo" -I"D:/ccs/elelctric_competition/electric_competition_empty" -I"D:/ccs/elelctric_competition/electric_competition_empty/Debug" -I"D:/ccs/mspm0_sdk/mspm0_sdk_2_10_00_04/source/third_party/CMSIS/Core/Include" -I"D:/ccs/mspm0_sdk/mspm0_sdk_2_10_00_04/source" -gdwarf-3 -Wall -MMD -MP -MF"drivers/$(basename $(<F)).d_raw" -MT"$(@)"  $(GEN_OPTS__FLAG) -o"$@" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '


