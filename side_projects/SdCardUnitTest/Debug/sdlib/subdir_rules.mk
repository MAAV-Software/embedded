################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Each subdirectory must supply rules for building sources it contributes
sdlib/ff.obj: /home/sheldon/ctrl-sajanptl/sdlib/ff.c $(GEN_OPTS) $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Compiler'
	"/home/sheldon/ti/ccsv6/tools/compiler/ti-cgt-arm_5.2.4/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 --abi=eabi -me --include_path="/home/sheldon/ctrl-sajanptl/sdlib" --include_path="/home/sheldon/ctrl-sajanptl/src" --include_path="/home/sheldon/ti/ccsv6/tools/compiler/ti-cgt-arm_5.2.4/include" --include_path="/home/sheldon/ti/SW-TM4C-2.1.0.12573/utils" --include_path="/home/sheldon/ti/SW-TM4C-2.1.0.12573" -g --gcc --define=ccs="ccs" --define=PART_TM4C1230C3PM --define=TARGET_IS_TM4C123_RB1 --define=PART_TM4C123GH6PM --display_error_number --diag_warning=225 --diag_wrap=off --preproc_with_compile --preproc_dependency="sdlib/ff.pp" --obj_directory="sdlib" $(GEN_OPTS__FLAG) "$(shell echo $<)"
	@echo 'Finished building: $<'
	@echo ' '

sdlib/mmc-dk-tm4c123g.obj: /home/sheldon/ctrl-sajanptl/sdlib/mmc-dk-tm4c123g.c $(GEN_OPTS) $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Compiler'
	"/home/sheldon/ti/ccsv6/tools/compiler/ti-cgt-arm_5.2.4/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 --abi=eabi -me --include_path="/home/sheldon/ctrl-sajanptl/sdlib" --include_path="/home/sheldon/ctrl-sajanptl/src" --include_path="/home/sheldon/ti/ccsv6/tools/compiler/ti-cgt-arm_5.2.4/include" --include_path="/home/sheldon/ti/SW-TM4C-2.1.0.12573/utils" --include_path="/home/sheldon/ti/SW-TM4C-2.1.0.12573" -g --gcc --define=ccs="ccs" --define=PART_TM4C1230C3PM --define=TARGET_IS_TM4C123_RB1 --define=PART_TM4C123GH6PM --display_error_number --diag_warning=225 --diag_wrap=off --preproc_with_compile --preproc_dependency="sdlib/mmc-dk-tm4c123g.pp" --obj_directory="sdlib" $(GEN_OPTS__FLAG) "$(shell echo $<)"
	@echo 'Finished building: $<'
	@echo ' '


