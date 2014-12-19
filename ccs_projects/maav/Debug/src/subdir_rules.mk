################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Each subdirectory must supply rules for building sources it contributes
src/PPM.obj: ../src/PPM.c $(GEN_OPTS) $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Compiler'
	"c:/ti/ccsv6/tools/compiler/arm_5.1.5/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 --abi=eabi -me --include_path="c:/ti/ccsv6/tools/compiler/arm_5.1.5/include" --include_path="C:/Users/MAAV/maav-controls/ccs_projects/maav/include" --include_path="C:/ti/TivaWare_C_Series-2.1.0.12573/utils" --include_path="C:/ti/TivaWare_C_Series-2.1.0.12573" -g --define=PART_TM4C1230C3PM --display_error_number --diag_warning=225 --diag_wrap=off --preproc_with_compile --preproc_dependency="src/PPM.pp" --obj_directory="src" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

src/dof.obj: ../src/dof.c $(GEN_OPTS) $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Compiler'
	"c:/ti/ccsv6/tools/compiler/arm_5.1.5/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 --abi=eabi -me --include_path="c:/ti/ccsv6/tools/compiler/arm_5.1.5/include" --include_path="C:/Users/MAAV/maav-controls/ccs_projects/maav/include" --include_path="C:/ti/TivaWare_C_Series-2.1.0.12573/utils" --include_path="C:/ti/TivaWare_C_Series-2.1.0.12573" -g --define=PART_TM4C1230C3PM --display_error_number --diag_warning=225 --diag_wrap=off --preproc_with_compile --preproc_dependency="src/dof.pp" --obj_directory="src" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

src/px4_i2c.obj: ../src/px4_i2c.c $(GEN_OPTS) $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Compiler'
	"c:/ti/ccsv6/tools/compiler/arm_5.1.5/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 --abi=eabi -me --include_path="c:/ti/ccsv6/tools/compiler/arm_5.1.5/include" --include_path="C:/Users/MAAV/maav-controls/ccs_projects/maav/include" --include_path="C:/ti/TivaWare_C_Series-2.1.0.12573/utils" --include_path="C:/ti/TivaWare_C_Series-2.1.0.12573" -g --define=PART_TM4C1230C3PM --display_error_number --diag_warning=225 --diag_wrap=off --preproc_with_compile --preproc_dependency="src/px4_i2c.pp" --obj_directory="src" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

src/px4_kalman.obj: ../src/px4_kalman.c $(GEN_OPTS) $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Compiler'
	"c:/ti/ccsv6/tools/compiler/arm_5.1.5/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 --abi=eabi -me --include_path="c:/ti/ccsv6/tools/compiler/arm_5.1.5/include" --include_path="C:/Users/MAAV/maav-controls/ccs_projects/maav/include" --include_path="C:/ti/TivaWare_C_Series-2.1.0.12573/utils" --include_path="C:/ti/TivaWare_C_Series-2.1.0.12573" -g --define=PART_TM4C1230C3PM --display_error_number --diag_warning=225 --diag_wrap=off --preproc_with_compile --preproc_dependency="src/px4_kalman.pp" --obj_directory="src" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

src/quad_ctrl.obj: ../src/quad_ctrl.c $(GEN_OPTS) $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Compiler'
	"c:/ti/ccsv6/tools/compiler/arm_5.1.5/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 --abi=eabi -me --include_path="c:/ti/ccsv6/tools/compiler/arm_5.1.5/include" --include_path="C:/Users/MAAV/maav-controls/ccs_projects/maav/include" --include_path="C:/ti/TivaWare_C_Series-2.1.0.12573/utils" --include_path="C:/ti/TivaWare_C_Series-2.1.0.12573" -g --define=PART_TM4C1230C3PM --display_error_number --diag_warning=225 --diag_wrap=off --preproc_with_compile --preproc_dependency="src/quad_ctrl.pp" --obj_directory="src" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

src/servoIn.obj: ../src/servoIn.c $(GEN_OPTS) $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Compiler'
	"c:/ti/ccsv6/tools/compiler/arm_5.1.5/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 --abi=eabi -me --include_path="c:/ti/ccsv6/tools/compiler/arm_5.1.5/include" --include_path="C:/Users/MAAV/maav-controls/ccs_projects/maav/include" --include_path="C:/ti/TivaWare_C_Series-2.1.0.12573/utils" --include_path="C:/ti/TivaWare_C_Series-2.1.0.12573" -g --define=PART_TM4C1230C3PM --display_error_number --diag_warning=225 --diag_wrap=off --preproc_with_compile --preproc_dependency="src/servoIn.pp" --obj_directory="src" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

src/time_util.obj: ../src/time_util.c $(GEN_OPTS) $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Compiler'
	"c:/ti/ccsv6/tools/compiler/arm_5.1.5/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 --abi=eabi -me --include_path="c:/ti/ccsv6/tools/compiler/arm_5.1.5/include" --include_path="C:/Users/MAAV/maav-controls/ccs_projects/maav/include" --include_path="C:/ti/TivaWare_C_Series-2.1.0.12573/utils" --include_path="C:/ti/TivaWare_C_Series-2.1.0.12573" -g --define=PART_TM4C1230C3PM --display_error_number --diag_warning=225 --diag_wrap=off --preproc_with_compile --preproc_dependency="src/time_util.pp" --obj_directory="src" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

src/utility.obj: ../src/utility.c $(GEN_OPTS) $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Compiler'
	"c:/ti/ccsv6/tools/compiler/arm_5.1.5/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 --abi=eabi -me --include_path="c:/ti/ccsv6/tools/compiler/arm_5.1.5/include" --include_path="C:/Users/MAAV/maav-controls/ccs_projects/maav/include" --include_path="C:/ti/TivaWare_C_Series-2.1.0.12573/utils" --include_path="C:/ti/TivaWare_C_Series-2.1.0.12573" -g --define=PART_TM4C1230C3PM --display_error_number --diag_warning=225 --diag_wrap=off --preproc_with_compile --preproc_dependency="src/utility.pp" --obj_directory="src" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '


