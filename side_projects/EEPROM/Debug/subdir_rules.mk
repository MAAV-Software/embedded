################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Each subdirectory must supply rules for building sources it contributes
main.obj: ../main.c $(GEN_OPTS) $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Compiler'
	"C:/ti_v6_1/ccsv6/tools/compiler/ti-cgt-arm_5.2.4/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 --abi=eabi -me --include_path="C:/ti_v6_1/ccsv6/tools/compiler/ti-cgt-arm_5.2.4/include" --include_path="C:/ti/SW-TM4C-2.1.0.12573" --gcc --define=ccs="ccs" --define=PART_TM4C123GH6PM --preproc_with_compile --preproc_dependency="main.pp" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

tm4c123gh6pm_startup_ccs.obj: ../tm4c123gh6pm_startup_ccs.c $(GEN_OPTS) $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Compiler'
	"C:/ti_v6_1/ccsv6/tools/compiler/ti-cgt-arm_5.2.4/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 --abi=eabi -me --include_path="C:/ti_v6_1/ccsv6/tools/compiler/ti-cgt-arm_5.2.4/include" --include_path="C:/ti/SW-TM4C-2.1.0.12573" --gcc --define=ccs="ccs" --define=PART_TM4C123GH6PM --preproc_with_compile --preproc_dependency="tm4c123gh6pm_startup_ccs.pp" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

