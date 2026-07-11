################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

CG_TOOL_ROOT := C:/ti/ccs2100/ccs/tools/compiler/ti-cgt-armllvm_5.1.1.LTS

GEN_OPTS__FLAG := @"./device.opt" 
GEN_CMDS__FLAG := -Wl,-l"./device_linker.cmd" 

ORDERED_OBJS += \
"./empty.o" \
"./ti_msp_dl_config.o" \
"./startup_mspm0g350x_ticlang.o" \
"./algo/algo_filter.o" \
"./algo/algo_fusion.o" \
"./algo/algo_pid.o" \
"./app/app_chassis.o" \
"./app/app_control_scheduler.o" \
"./app/app_imu.o" \
"./app/app_isr.o" \
"./app/app_turret.o" \
"./app/app_ui.o" \
"./bsp/bsp_gpio.o" \
"./bsp/bsp_i2c.o" \
"./bsp/bsp_pwm.o" \
"./bsp/bsp_uart.o" \
"./common/ringbuf.o" \
"./common/soft_timer.o" \
"./drivers/drv_encoder_ab.o" \
"./drivers/drv_line_sensor.o" \
"./drivers/drv_motor_dc.o" \
"./drivers/drv_mpu9250.o" \
"./drivers/drv_oled_ssd1306.o" \
"./drivers/drv_stepper.o" \
"./protocol/proto_k230.o" \
"./protocol/proto_vofa_firewater.o" \
$(GEN_CMDS__FLAG) \
-Wl,-ldevice.cmd.genlibs \
-Wl,-llibc.a \

-include ../makefile.init

RM := DEL /F
RMDIR := RMDIR /S/Q

# All of the sources participating in the build are defined here
-include sources.mk
-include subdir_vars.mk
-include algo/subdir_vars.mk
-include app/subdir_vars.mk
-include bsp/subdir_vars.mk
-include common/subdir_vars.mk
-include drivers/subdir_vars.mk
-include protocol/subdir_vars.mk
-include subdir_rules.mk
-include algo/subdir_rules.mk
-include app/subdir_rules.mk
-include bsp/subdir_rules.mk
-include common/subdir_rules.mk
-include drivers/subdir_rules.mk
-include protocol/subdir_rules.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C55_DEPS)),)
-include $(C55_DEPS)
endif
ifneq ($(strip $(C_UPPER_DEPS)),)
-include $(C_UPPER_DEPS)
endif
ifneq ($(strip $(S67_DEPS)),)
-include $(S67_DEPS)
endif
ifneq ($(strip $(S62_DEPS)),)
-include $(S62_DEPS)
endif
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(OPT_DEPS)),)
-include $(OPT_DEPS)
endif
ifneq ($(strip $(C??_DEPS)),)
-include $(C??_DEPS)
endif
ifneq ($(strip $(ASM_UPPER_DEPS)),)
-include $(ASM_UPPER_DEPS)
endif
ifneq ($(strip $(S??_DEPS)),)
-include $(S??_DEPS)
endif
ifneq ($(strip $(C64_DEPS)),)
-include $(C64_DEPS)
endif
ifneq ($(strip $(CXX_DEPS)),)
-include $(CXX_DEPS)
endif
ifneq ($(strip $(S64_DEPS)),)
-include $(S64_DEPS)
endif
ifneq ($(strip $(CLA_DEPS)),)
-include $(CLA_DEPS)
endif
ifneq ($(strip $(S55_DEPS)),)
-include $(S55_DEPS)
endif
ifneq ($(strip $(SV7A_DEPS)),)
-include $(SV7A_DEPS)
endif
ifneq ($(strip $(C62_DEPS)),)
-include $(C62_DEPS)
endif
ifneq ($(strip $(C67_DEPS)),)
-include $(C67_DEPS)
endif
ifneq ($(strip $(K_DEPS)),)
-include $(K_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(CC_DEPS)),)
-include $(CC_DEPS)
endif
ifneq ($(strip $(C++_DEPS)),)
-include $(C++_DEPS)
endif
ifneq ($(strip $(C43_DEPS)),)
-include $(C43_DEPS)
endif
ifneq ($(strip $(S43_DEPS)),)
-include $(S43_DEPS)
endif
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(CPP_DEPS)),)
-include $(CPP_DEPS)
endif
ifneq ($(strip $(SA_DEPS)),)
-include $(SA_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXE_OUTPUTS += \
electric_competition_empty.out 

EXE_OUTPUTS__QUOTED += \
"electric_competition_empty.out" 


# All Target
all: $(OBJS) $(GEN_CMDS)
	@$(MAKE) --no-print-directory -Onone "electric_competition_empty.out"

# Tool invocations
electric_competition_empty.out: $(OBJS) $(GEN_CMDS)
	@echo 'Arm Linker - building target: "$@"'
	"C:/ti/ccs2100/ccs/tools/compiler/ti-cgt-armllvm_5.1.1.LTS/bin/tiarmclang.exe" @"device.opt"  -march=thumbv6m -mcpu=cortex-m0plus -mfloat-abi=soft -mlittle-endian -mthumb -O0 -gdwarf-3 -Wall -Wl,-m"electric_competition_empty.map" -Wl,-i"D:/ccs/mspm0_sdk/mspm0_sdk_2_10_00_04/source" -Wl,-i"D:/ccs/elelctric_competition/electric_competition_empty" -Wl,-i"D:/ccs/elelctric_competition/electric_competition_empty/Debug/syscfg" -Wl,-i"C:/ti/ccs2100/ccs/tools/compiler/ti-cgt-armllvm_5.1.1.LTS/lib" -Wl,--diag_wrap=off -Wl,--display_error_number -Wl,--warn_sections -Wl,--xml_link_info="electric_competition_empty_linkInfo.xml" -Wl,--rom_model -o "electric_competition_empty.out" $(ORDERED_OBJS)
	@echo 'Finished building target: "$@"'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(GEN_MISC_FILES__QUOTED)$(GEN_FILES__QUOTED)$(EXE_OUTPUTS__QUOTED)
	-$(RM) "empty.o" "ti_msp_dl_config.o" "startup_mspm0g350x_ticlang.o" "algo\algo_filter.o" "algo\algo_fusion.o" "algo\algo_pid.o" "app\app_chassis.o" "app\app_control_scheduler.o" "app\app_imu.o" "app\app_isr.o" "app\app_turret.o" "app\app_ui.o" "bsp\bsp_gpio.o" "bsp\bsp_i2c.o" "bsp\bsp_pwm.o" "bsp\bsp_uart.o" "common\ringbuf.o" "common\soft_timer.o" "drivers\drv_encoder_ab.o" "drivers\drv_line_sensor.o" "drivers\drv_motor_dc.o" "drivers\drv_mpu9250.o" "drivers\drv_oled_ssd1306.o" "drivers\drv_stepper.o" "protocol\proto_k230.o" "protocol\proto_vofa_firewater.o" 
	-$(RM) "empty.d" "ti_msp_dl_config.d" "startup_mspm0g350x_ticlang.d" "algo\algo_filter.d" "algo\algo_fusion.d" "algo\algo_pid.d" "app\app_chassis.d" "app\app_control_scheduler.d" "app\app_imu.d" "app\app_isr.d" "app\app_turret.d" "app\app_ui.d" "bsp\bsp_gpio.d" "bsp\bsp_i2c.d" "bsp\bsp_pwm.d" "bsp\bsp_uart.d" "common\ringbuf.d" "common\soft_timer.d" "drivers\drv_encoder_ab.d" "drivers\drv_line_sensor.d" "drivers\drv_motor_dc.d" "drivers\drv_mpu9250.d" "drivers\drv_oled_ssd1306.d" "drivers\drv_stepper.d" "protocol\proto_k230.d" "protocol\proto_vofa_firewater.d" 
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets

