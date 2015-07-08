################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Each subdirectory must supply rules for building sources it contributes
tests/data_link_test.obj: ../tests/data_link_test.c $(GEN_OPTS) $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Compiler'
	"/home/sajan/ti/ccsv6/tools/compiler/ti-cgt-arm_5.2.2/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 --abi=eabi -me --include_path="/home/sajan/ti/ccsv6/tools/compiler/ti-cgt-arm_5.2.2/include" --include_path="/home/sajan/maav-controls/maav/include" --include_path="/home/sheldon/ti/SW-TM4C-2.1.0.12573/utils" --include_path="/home/sheldon/ti/SW-TM4C-2.1.0.12573" -g --define=PART_TM4C1230C3PM --display_error_number --diag_warning=225 --diag_wrap=off --preproc_with_compile --preproc_dependency="tests/data_link_test.pp" --obj_directory="tests" $(GEN_OPTS__FLAG) "$(shell echo $<)"
	@echo 'Finished building: $<'
	@echo ' '

tests/gimbal_test.obj: ../tests/gimbal_test.c $(GEN_OPTS) $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Compiler'
	"/home/sajan/ti/ccsv6/tools/compiler/ti-cgt-arm_5.2.2/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 --abi=eabi -me --include_path="/home/sajan/ti/ccsv6/tools/compiler/ti-cgt-arm_5.2.2/include" --include_path="/home/sajan/maav-controls/maav/include" --include_path="/home/sheldon/ti/SW-TM4C-2.1.0.12573/utils" --include_path="/home/sheldon/ti/SW-TM4C-2.1.0.12573" -g --define=PART_TM4C1230C3PM --display_error_number --diag_warning=225 --diag_wrap=off --preproc_with_compile --preproc_dependency="tests/gimbal_test.pp" --obj_directory="tests" $(GEN_OPTS__FLAG) "$(shell echo $<)"
	@echo 'Finished building: $<'
	@echo ' '

tests/qc_test.obj: ../tests/qc_test.c $(GEN_OPTS) $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Compiler'
	"/home/sajan/ti/ccsv6/tools/compiler/ti-cgt-arm_5.2.2/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 --abi=eabi -me --include_path="/home/sajan/ti/ccsv6/tools/compiler/ti-cgt-arm_5.2.2/include" --include_path="/home/sajan/maav-controls/maav/include" --include_path="/home/sheldon/ti/SW-TM4C-2.1.0.12573/utils" --include_path="/home/sheldon/ti/SW-TM4C-2.1.0.12573" -g --define=PART_TM4C1230C3PM --display_error_number --diag_warning=225 --diag_wrap=off --preproc_with_compile --preproc_dependency="tests/qc_test.pp" --obj_directory="tests" $(GEN_OPTS__FLAG) "$(shell echo $<)"
	@echo 'Finished building: $<'
	@echo ' '

tests/rc_test.obj: ../tests/rc_test.c $(GEN_OPTS) $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Compiler'
	"/home/sajan/ti/ccsv6/tools/compiler/ti-cgt-arm_5.2.2/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 --abi=eabi -me --include_path="/home/sajan/ti/ccsv6/tools/compiler/ti-cgt-arm_5.2.2/include" --include_path="/home/sajan/maav-controls/maav/include" --include_path="/home/sheldon/ti/SW-TM4C-2.1.0.12573/utils" --include_path="/home/sheldon/ti/SW-TM4C-2.1.0.12573" -g --define=PART_TM4C1230C3PM --display_error_number --diag_warning=225 --diag_wrap=off --preproc_with_compile --preproc_dependency="tests/rc_test.pp" --obj_directory="tests" $(GEN_OPTS__FLAG) "$(shell echo $<)"
	@echo 'Finished building: $<'
	@echo ' '

tests/runnable_test.obj: ../tests/runnable_test.cpp $(GEN_OPTS) $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Compiler'
	"/home/sajan/ti/ccsv6/tools/compiler/ti-cgt-arm_5.2.2/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 --abi=eabi -me --include_path="/home/sajan/ti/ccsv6/tools/compiler/ti-cgt-arm_5.2.2/include" --include_path="/home/sajan/maav-controls/maav/include" --include_path="/home/sheldon/ti/SW-TM4C-2.1.0.12573/utils" --include_path="/home/sheldon/ti/SW-TM4C-2.1.0.12573" -g --define=PART_TM4C1230C3PM --display_error_number --diag_warning=225 --diag_wrap=off --preproc_with_compile --preproc_dependency="tests/runnable_test.pp" --obj_directory="tests" $(GEN_OPTS__FLAG) "$(shell echo $<)"
	@echo 'Finished building: $<'
	@echo ' '


