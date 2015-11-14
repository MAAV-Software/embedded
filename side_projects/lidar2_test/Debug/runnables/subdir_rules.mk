################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Each subdirectory must supply rules for building sources it contributes
runnables/I2CRunnable.obj: ../runnables/I2CRunnable.cpp $(GEN_OPTS) $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Compiler'
	"c:/ti/ccsv6/tools/compiler/ti-cgt-arm_5.2.4/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 --abi=eabi -me --include_path="C:/Users/Sasawat/3workspace/ctrl/side_projects/lidar2_test" --include_path="c:/ti/ccsv6/tools/compiler/ti-cgt-arm_5.2.4/include" --include_path="C:/ti/TivaWare_C_Series-2.1.0.12573" --include_path="C:/Users/Sasawat/3workspace/ctrl/include" --advice:power=all -g --embedded_cpp --gcc --define=ccs="ccs" --define=PART_TM4C123GH6PM --display_error_number --diag_warning=225 --diag_wrap=off --preproc_with_compile --preproc_dependency="runnables/I2CRunnable.pp" --obj_directory="runnables" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '


