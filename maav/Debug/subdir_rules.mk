################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Each subdirectory must supply rules for building sources it contributes
main.obj: ../main.cpp $(GEN_OPTS) $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Compiler'
	"/home/sajan/ti/ccsv6/tools/compiler/ti-cgt-arm_5.2.2/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 --abi=eabi -me -Ooff --opt_for_speed=1 --include_path="/home/sajan/ti/ccsv6/tools/compiler/ti-cgt-arm_5.2.2/include" --include_path="/home/sajan/maav-controls/include" --include_path="/home/sajan/ti/SW-TM4C-2.1.0.12573/utils" --include_path="/home/sajan/ti/SW-TM4C-2.1.0.12573" -g --gcc --define=PART_TM4C1230C3PM --define=ccs="ccs" --define=PART_TM4C123GH6PM --display_error_number --diag_warning=225 --diag_wrap=off --preproc_with_compile --preproc_dependency="main.pp" $(GEN_OPTS__FLAG) "$(shell echo $<)"
	@echo 'Finished building: $<'
	@echo ' '

tm4c123gh6pm_startup_ccs.obj: ../tm4c123gh6pm_startup_ccs.c $(GEN_OPTS) $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Compiler'
	"/home/sajan/ti/ccsv6/tools/compiler/ti-cgt-arm_5.2.2/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 --abi=eabi -me -Ooff --opt_for_speed=1 --include_path="/home/sajan/ti/ccsv6/tools/compiler/ti-cgt-arm_5.2.2/include" --include_path="/home/sajan/maav-controls/include" --include_path="/home/sajan/ti/SW-TM4C-2.1.0.12573/utils" --include_path="/home/sajan/ti/SW-TM4C-2.1.0.12573" -g --gcc --define=PART_TM4C1230C3PM --define=ccs="ccs" --define=PART_TM4C123GH6PM --display_error_number --diag_warning=225 --diag_wrap=off --preproc_with_compile --preproc_dependency="tm4c123gh6pm_startup_ccs.pp" $(GEN_OPTS__FLAG) "$(shell echo $<)"
	@echo 'Finished building: $<'
	@echo ' '

uartstdio.obj: /home/sajan/ti/SW-TM4C-2.1.0.12573/utils/uartstdio.c $(GEN_OPTS) $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Compiler'
	"/home/sajan/ti/ccsv6/tools/compiler/ti-cgt-arm_5.2.2/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 --abi=eabi -me -Ooff --opt_for_speed=1 --include_path="/home/sajan/ti/ccsv6/tools/compiler/ti-cgt-arm_5.2.2/include" --include_path="/home/sajan/maav-controls/include" --include_path="/home/sajan/ti/SW-TM4C-2.1.0.12573/utils" --include_path="/home/sajan/ti/SW-TM4C-2.1.0.12573" -g --gcc --define=PART_TM4C1230C3PM --define=ccs="ccs" --define=PART_TM4C123GH6PM --display_error_number --diag_warning=225 --diag_wrap=off --preproc_with_compile --preproc_dependency="uartstdio.pp" $(GEN_OPTS__FLAG) "$(shell echo $<)"
	@echo 'Finished building: $<'
	@echo ' '

