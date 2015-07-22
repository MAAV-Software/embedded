################################################################################
# Automatically-generated file. Do not edit!
################################################################################

CG_TOOL_ROOT := /home/sheldon/ti/ccsv6/tools/compiler/ti-cgt-arm_5.2.4

ORDERED_OBJS += \
"./cmdline.obj" \
"./main.obj" \
"./tm4c123gh6pm_startup_ccs.obj" \
"./uartstdio.obj" \
"./src/Dof.obj" \
"./src/FlightModeHw.obj" \
"./src/I2CHw.obj" \
"./src/Imu.obj" \
"./src/ImuHw.obj" \
"./src/LED.obj" \
"./src/Lidar.obj" \
"./src/Loop.obj" \
"./src/LowPass.obj" \
"./src/PPM.obj" \
"./src/Pid.obj" \
"./src/ProgramState.obj" \
"./src/Px4.obj" \
"./src/SdCard.obj" \
"./src/Vehicle.obj" \
"./src/rc.obj" \
"./src/servoIn.obj" \
"./src/switch.obj" \
"./src/time_util.obj" \
"./src/utility.obj" \
"./src/runnables/ControllerRunnable.obj" \
"./src/runnables/DataLinkRunnables.obj" \
"./src/runnables/DjiRunnable.obj" \
"./src/runnables/FlightModeRunnable.obj" \
"./src/runnables/I2CRunnable.obj" \
"./src/runnables/ImuRunnable.obj" \
"./src/runnables/OptSwitchRunnables.obj" \
"./src/runnables/Px4Runnable.obj" \
"./src/messaging/data_link.obj" \
"./src/messaging/djiout_feedback_t.obj" \
"./src/messaging/dof_feedback_t.obj" \
"./src/messaging/feedback_t.obj" \
"./src/messaging/lcmlite.obj" \
"./src/messaging/position_t.obj" \
"./src/messaging/ringbuf.obj" \
"./src/messaging/str_log_t.obj" \
"./src/messaging/target_t.obj" \
"./src/messaging/tuning_t.obj" \
"./sdlib/ff.obj" \
"./sdlib/mmc-dk-tm4c123g.obj" \
"/home/sheldon/ti/SW-TM4C-2.1.0.12573/driverlib/ccs/Debug/driverlib.lib" \
"/home/sheldon/maav-controls/libs/dsplib.lib" \
"/home/sheldon/ti/SW-TM4C-2.1.0.12573/sensorlib/ccs/Debug/sensorlib.lib" \
"../tm4c123gh6pm.cmd" \
$(GEN_CMDS__FLAG) \
-l"libc.a" \

-include ../makefile.init

RM := rm -rf
RMDIR := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include subdir_vars.mk
-include src/subdir_vars.mk
-include src/runnables/subdir_vars.mk
-include src/messaging/subdir_vars.mk
-include sdlib/subdir_vars.mk
-include subdir_rules.mk
-include src/subdir_rules.mk
-include src/runnables/subdir_rules.mk
-include src/messaging/subdir_rules.mk
-include sdlib/subdir_rules.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(S62_DEPS)),)
-include $(S62_DEPS)
endif
ifneq ($(strip $(C64_DEPS)),)
-include $(C64_DEPS)
endif
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(CC_DEPS)),)
-include $(CC_DEPS)
endif
ifneq ($(strip $(S55_DEPS)),)
-include $(S55_DEPS)
endif
ifneq ($(strip $(C67_DEPS)),)
-include $(C67_DEPS)
endif
ifneq ($(strip $(CLA_DEPS)),)
-include $(CLA_DEPS)
endif
ifneq ($(strip $(C??_DEPS)),)
-include $(C??_DEPS)
endif
ifneq ($(strip $(CPP_DEPS)),)
-include $(CPP_DEPS)
endif
ifneq ($(strip $(S??_DEPS)),)
-include $(S??_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(C62_DEPS)),)
-include $(C62_DEPS)
endif
ifneq ($(strip $(CXX_DEPS)),)
-include $(CXX_DEPS)
endif
ifneq ($(strip $(C++_DEPS)),)
-include $(C++_DEPS)
endif
ifneq ($(strip $(ASM_UPPER_DEPS)),)
-include $(ASM_UPPER_DEPS)
endif
ifneq ($(strip $(K_DEPS)),)
-include $(K_DEPS)
endif
ifneq ($(strip $(C43_DEPS)),)
-include $(C43_DEPS)
endif
ifneq ($(strip $(INO_DEPS)),)
-include $(INO_DEPS)
endif
ifneq ($(strip $(S67_DEPS)),)
-include $(S67_DEPS)
endif
ifneq ($(strip $(SA_DEPS)),)
-include $(SA_DEPS)
endif
ifneq ($(strip $(S43_DEPS)),)
-include $(S43_DEPS)
endif
ifneq ($(strip $(OPT_DEPS)),)
-include $(OPT_DEPS)
endif
ifneq ($(strip $(PDE_DEPS)),)
-include $(PDE_DEPS)
endif
ifneq ($(strip $(S64_DEPS)),)
-include $(S64_DEPS)
endif
ifneq ($(strip $(C_UPPER_DEPS)),)
-include $(C_UPPER_DEPS)
endif
ifneq ($(strip $(C55_DEPS)),)
-include $(C55_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXE_OUTPUTS += \
maav.out \

EXE_OUTPUTS__QUOTED += \
"maav.out" \

BIN_OUTPUTS += \
maav.hex \

BIN_OUTPUTS__QUOTED += \
"maav.hex" \


# All Target
all: maav.out

# Tool invocations
maav.out: $(OBJS) $(CMD_SRCS) $(LIB_SRCS) $(GEN_CMDS)
	@echo 'Building target: $@'
	@echo 'Invoking: ARM Linker'
	"/home/sheldon/ti/ccsv6/tools/compiler/ti-cgt-arm_5.2.4/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 --abi=eabi -me -Ooff --opt_for_speed=1 -g --c99 --c++03 --gcc --define=_LINKAGE --define=PART_TM4C1230C3PM --define=__GNUC__ --define=__FPU_PRESENT=1 --define=_CODE_ACCESS="" --define=_DATA_ACCESS="" --define=ARM_MATH_CM4 --define=TARGET_IS_TM4C123_RB1 --define=ccs="ccs" --define=PART_TM4C123GH6PM --display_error_number --diag_warning=225 --diag_wrap=off -z -m"maav.map" --heap_size=1536 --stack_size=16384 -i"/home/sheldon/ti/ccsv6/tools/compiler/ti-cgt-arm_5.2.4/lib" -i"/home/sheldon/ti/ccsv6/tools/compiler/ti-cgt-arm_5.2.4/include" -i"/home/sheldon/maav-controls/include" --reread_libs --warn_sections --display_error_number --diag_wrap=off --xml_link_info="maav_linkInfo.xml" --rom_model -o "maav.out" $(ORDERED_OBJS)
	@echo 'Finished building target: $@'
	@echo ' '

maav.hex: $(EXE_OUTPUTS)
	@echo 'Invoking: ARM Hex Utility'
	"/home/sheldon/ti/ccsv6/tools/compiler/ti-cgt-arm_5.2.4/bin/armhex"  -o "maav.hex" $(EXE_OUTPUTS__QUOTED)
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(EXE_OUTPUTS__QUOTED)$(BIN_OUTPUTS__QUOTED)
	-$(RM) "main.pp" "src/Dof.pp" "src/FlightModeHw.pp" "src/I2CHw.pp" "src/Imu.pp" "src/ImuHw.pp" "src/Lidar.pp" "src/Loop.pp" "src/LowPass.pp" "src/Pid.pp" "src/ProgramState.pp" "src/Px4.pp" "src/SdCard.pp" "src/Vehicle.pp" "src/rc.pp" "src/servoIn.pp" "src/runnables/ControllerRunnable.pp" "src/runnables/DataLinkRunnables.pp" "src/runnables/DjiRunnable.pp" "src/runnables/FlightModeRunnable.pp" "src/runnables/I2CRunnable.pp" "src/runnables/ImuRunnable.pp" "src/runnables/OptSwitchRunnables.pp" "src/runnables/Px4Runnable.pp" 
	-$(RM) "cmdline.pp" "tm4c123gh6pm_startup_ccs.pp" "uartstdio.pp" "src/LED.pp" "src/PPM.pp" "src/switch.pp" "src/time_util.pp" "src/utility.pp" "src/messaging/data_link.pp" "src/messaging/djiout_feedback_t.pp" "src/messaging/dof_feedback_t.pp" "src/messaging/feedback_t.pp" "src/messaging/lcmlite.pp" "src/messaging/position_t.pp" "src/messaging/ringbuf.pp" "src/messaging/str_log_t.pp" "src/messaging/target_t.pp" "src/messaging/tuning_t.pp" "sdlib/ff.pp" "sdlib/mmc-dk-tm4c123g.pp" 
	-$(RM) "cmdline.obj" "main.obj" "tm4c123gh6pm_startup_ccs.obj" "uartstdio.obj" "src/Dof.obj" "src/FlightModeHw.obj" "src/I2CHw.obj" "src/Imu.obj" "src/ImuHw.obj" "src/LED.obj" "src/Lidar.obj" "src/Loop.obj" "src/LowPass.obj" "src/PPM.obj" "src/Pid.obj" "src/ProgramState.obj" "src/Px4.obj" "src/SdCard.obj" "src/Vehicle.obj" "src/rc.obj" "src/servoIn.obj" "src/switch.obj" "src/time_util.obj" "src/utility.obj" "src/runnables/ControllerRunnable.obj" "src/runnables/DataLinkRunnables.obj" "src/runnables/DjiRunnable.obj" "src/runnables/FlightModeRunnable.obj" "src/runnables/I2CRunnable.obj" "src/runnables/ImuRunnable.obj" "src/runnables/OptSwitchRunnables.obj" "src/runnables/Px4Runnable.obj" "src/messaging/data_link.obj" "src/messaging/djiout_feedback_t.obj" "src/messaging/dof_feedback_t.obj" "src/messaging/feedback_t.obj" "src/messaging/lcmlite.obj" "src/messaging/position_t.obj" "src/messaging/ringbuf.obj" "src/messaging/str_log_t.obj" "src/messaging/target_t.obj" "src/messaging/tuning_t.obj" "sdlib/ff.obj" 
	-$(RM) "sdlib/mmc-dk-tm4c123g.obj" 
	-@echo 'Finished clean'
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets

