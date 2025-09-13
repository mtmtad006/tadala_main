###########################################################################
## Makefile generated for component 'MicroMouse_Deploy'. 
## 
## Makefile     : MicroMouse_Deploy.mk
## Generated on : Sat Sep 13 19:10:00 2025
## Final product: $(RELATIVE_PATH_TO_ANCHOR)/MicroMouse_Deploy.elf
## Product type : executable
## 
###########################################################################

###########################################################################
## MACROS
###########################################################################

# Macro Descriptions:
# PRODUCT_NAME            Name of the system to build
# MAKEFILE                Name of this makefile
# COMPILER_COMMAND_FILE   Compiler command listing model reference header paths
# CMD_FILE                Command file

PRODUCT_NAME              = MicroMouse_Deploy
MAKEFILE                  = MicroMouse_Deploy.mk
MATLAB_ROOT               = C:/Program Files/MATLAB/R2025a
MATLAB_BIN                = C:/Program Files/MATLAB/R2025a/bin
MATLAB_ARCH_BIN           = $(MATLAB_BIN)/win64
START_DIR                 = C:/Users/Student/Documents/MATLAB/tadala_main/3099/tadala_micromouse
SOLVER                    = 
SOLVER_OBJ                = 
CLASSIC_INTERFACE         = 0
TGT_FCN_LIB               = ISO_C
MODEL_HAS_DYNAMICALLY_LOADED_SFCNS = 0
RELATIVE_PATH_TO_ANCHOR   = ../..
COMPILER_COMMAND_FILE     = MicroMouse_Deploy_comp.rsp
CMD_FILE                  = MicroMouse_Deploy.rsp
C_STANDARD_OPTS           = 
CPP_STANDARD_OPTS         = 
EMPTY                     = 
SPACE                     = $(EMPTY) $(EMPTY)
SPACE_TO_QUESTION         = $(subst $(SPACE),?,$1)
ESCAPE_SPACES             = $(subst $(SPACE),\ ,$1)
SUBSTITUTE_ESCAPED_SPACES = $(subst \ ,__<SPACE>__,$1)
ADD_QUOTES                = $(foreach aPath,$1,"$(aPath)")
REVERT_SPACES             = $(subst __<SPACE>__,$(SPACE),$1)
CONVERT_ESCAPED_SPACES_TO_QUOTES = $(call REVERT_SPACES,$(call ADD_QUOTES,$(call SUBSTITUTE_ESCAPED_SPACES,$1)))

###########################################################################
## TOOLCHAIN SPECIFICATIONS
###########################################################################

# Toolchain Name:          GNU Tools for ARM Embedded Processors
# Supported Version(s):    
# ToolchainInfo Version:   2025a
# Specification Revision:  1.0
# 
#-------------------------------------------
# Macros assumed to be defined elsewhere
#-------------------------------------------

# TARGET_LOAD_CMD_ARGS
# TARGET_LOAD_CMD
# MW_GNU_ARM_TOOLS_PATH
# FDATASECTIONS_FLG

#-----------
# MACROS
#-----------

LIBGCC                    = ${shell $(MW_GNU_ARM_TOOLS_PATH)/arm-none-eabi-gcc ${CFLAGS} -print-libgcc-file-name}
LIBC                      = ${shell $(MW_GNU_ARM_TOOLS_PATH)/arm-none-eabi-gcc ${CFLAGS} -print-file-name=libc.a}
LIBM                      = ${shell $(MW_GNU_ARM_TOOLS_PATH)/arm-none-eabi-gcc ${CFLAGS} -print-file-name=libm.a}
PRODUCT_NAME_WITHOUT_EXTN = $(basename $(PRODUCT))
PRODUCT_BIN               = $(PRODUCT_NAME_WITHOUT_EXTN).bin
PRODUCT_HEX               = $(PRODUCT_NAME_WITHOUT_EXTN).hex
CPFLAGS                   = -O binary
SHELL                     = %SystemRoot%/system32/cmd.exe

TOOLCHAIN_SRCS = 
TOOLCHAIN_INCS = 
TOOLCHAIN_LIBS = -lm

#------------------------
# BUILD TOOL COMMANDS
#------------------------

# Assembler: GNU ARM Assembler
AS_PATH = $(MW_GNU_ARM_TOOLS_PATH)
AS = "$(AS_PATH)/arm-none-eabi-gcc"

# C Compiler: GNU ARM C Compiler
CC_PATH = $(MW_GNU_ARM_TOOLS_PATH)
CC = "$(CC_PATH)/arm-none-eabi-gcc"

# Linker: GNU ARM Linker
LD_PATH = $(MW_GNU_ARM_TOOLS_PATH)
LD = "$(LD_PATH)/arm-none-eabi-g++"

# C++ Compiler: GNU ARM C++ Compiler
CPP_PATH = $(MW_GNU_ARM_TOOLS_PATH)
CPP = "$(CPP_PATH)/arm-none-eabi-g++"

# C++ Linker: GNU ARM C++ Linker
CPP_LD_PATH = $(MW_GNU_ARM_TOOLS_PATH)
CPP_LD = "$(CPP_LD_PATH)/arm-none-eabi-g++"

# Archiver: GNU ARM Archiver
AR_PATH = $(MW_GNU_ARM_TOOLS_PATH)
AR = "$(AR_PATH)/arm-none-eabi-ar"

# MEX Tool: MEX Tool
MEX_PATH = $(MATLAB_ARCH_BIN)
MEX = "$(MEX_PATH)/mex"

# Binary Converter: Binary Converter
OBJCOPYPATH = $(MW_GNU_ARM_TOOLS_PATH)
OBJCOPY = "$(OBJCOPYPATH)/arm-none-eabi-objcopy"

# Hex Converter: Hex Converter
OBJCOPYPATH = $(MW_GNU_ARM_TOOLS_PATH)
OBJCOPY = "$(OBJCOPYPATH)/arm-none-eabi-objcopy"

# Executable Size: Executable Size
EXESIZEPATH = $(MW_GNU_ARM_TOOLS_PATH)
EXESIZE = "$(EXESIZEPATH)/arm-none-eabi-size"

# Download: Download
DOWNLOAD =

# Execute: Execute
EXECUTE = $(PRODUCT)

# Builder: GMAKE Utility
MAKE_PATH = %MATLAB%\bin\win64
MAKE = "$(MAKE_PATH)/gmake"


#-------------------------
# Directives/Utilities
#-------------------------

ASDEBUG             = -g
AS_OUTPUT_FLAG      = -o
CDEBUG              = -g
C_OUTPUT_FLAG       = -o
LDDEBUG             = -g
OUTPUT_FLAG         = -o
CPPDEBUG            = -g
CPP_OUTPUT_FLAG     = -o
CPPLDDEBUG          = -g
OUTPUT_FLAG         = -o
ARDEBUG             =
STATICLIB_OUTPUT_FLAG =
MEX_DEBUG           = -g
RM                  = @del /f/q
ECHO                = @echo
MV                  = @move
RUN                 =

#----------------------------------------
# "Faster Builds" Build Configuration
#----------------------------------------

ARFLAGS              = ruvs
ASFLAGS              = -MMD -MP -MF"$(@:%.s.o=%.s.dep)" -MT"$@"  \
                       -Wall \
                       -x assembler-with-cpp \
                       $(ASFLAGS_ADDITIONAL) \
                       $(DEFINES) \
                       $(INCLUDES) \
                       -c
OBJCOPYFLAGS_BIN     = -O binary $(PRODUCT) $(PRODUCT_BIN)
CFLAGS               = $(FDATASECTIONS_FLG) \
                       -Wall \
                       -c \
                       -MMD -MP -MF"$(@:%.c.o=%.c.dep)" -MT"$@"  \
                       -O0
CPPFLAGS             = -std=gnu++14 \
                       -fno-rtti \
                       -fno-exceptions \
                       $(FDATASECTIONS_FLG) \
                       -Wall \
                       -c \
                       -MMD -MP -MF"$(@:%.cpp.o=%.cpp.dep)" -MT"$@"  \
                       -O0
CPP_LDFLAGS          = -Wl,--gc-sections \
                       -Wl,-Map="$(PRODUCT_NAME).map"
CPP_SHAREDLIB_LDFLAGS  =
DOWNLOAD_FLAGS       =
EXESIZE_FLAGS        = $(PRODUCT)
EXECUTE_FLAGS        =
OBJCOPYFLAGS_HEX     = -O ihex $(PRODUCT) $(PRODUCT_HEX)
LDFLAGS              = -Wl,--gc-sections \
                       -Wl,-Map="$(PRODUCT_NAME).map"
MEX_CPPFLAGS         =
MEX_CPPLDFLAGS       =
MEX_CFLAGS           =
MEX_LDFLAGS          =
MAKE_FLAGS           = -f $(MAKEFILE)
SHAREDLIB_LDFLAGS    =



###########################################################################
## OUTPUT INFO
###########################################################################

PRODUCT = $(RELATIVE_PATH_TO_ANCHOR)/MicroMouse_Deploy.elf
PRODUCT_TYPE = "executable"
BUILD_TYPE = "Top-Level Standalone Executable"

###########################################################################
## INCLUDE PATHS
###########################################################################

INCLUDES_BUILDINFO = 

INCLUDES = $(INCLUDES_BUILDINFO)

###########################################################################
## DEFINES
###########################################################################

DEFINES_ = -DMW_STANDALONE_EXECUTION_PROFILER_ON -D__MW_TARGET_USE_HARDWARE_RESOURCES_H__ -DUSE_HAL_DRIVER -DSTM32L476xx -DMW_TIMEBASESOURCE=TIM6 -DMW_GPIO_BIT_SHIFT=0 -DMW_CubeMXVersion_6.13.0
DEFINES_BUILD_ARGS = -DCLASSIC_INTERFACE=0 -DALLOCATIONFCN=0 -DTERMFCN=1 -DONESTEPFCN=1 -DMAT_FILE=0 -DMULTI_INSTANCE_CODE=0 -DINTEGER_CODE=0 -DMT=0
DEFINES_CUSTOM = -Ddefine -DCOMPILED_BY_SIMULINK -Ddefine -DIMU_ICM42605 -Ddefine -DIMU_DYNAMIC_FSR -Ddefine -DTOF_DYNAMIC_FSR -Ddefine -DSCREEN_FONT_DYNAMIC -Ddefine -DI2C_SAFE_RESTART
DEFINES_OPTS = -DTID01EQ=1
DEFINES_SKIPFORSIL = -DXCP_CUSTOM_PLATFORM -D__FPU_PRESENT=1U -D__FPU_USED=1U -DSTACK_SIZE=4096 -DRT
DEFINES_STANDARD = -DMODEL=MicroMouse_Deploy -DNUMST=2 -DNCSTATES=3 -DHAVESTDIO -DMODEL_HAS_DYNAMICALLY_LOADED_SFCNS=0

DEFINES = $(DEFINES_) $(DEFINES_BUILD_ARGS) $(DEFINES_CUSTOM) $(DEFINES_OPTS) $(DEFINES_SKIPFORSIL) $(DEFINES_STANDARD)

###########################################################################
## SOURCE FILES
###########################################################################

SRCS = $(START_DIR)/MicroMouse_Deploy_ert_rtw/instrumented/MicroMouse_Deploy.c $(START_DIR)/MicroMouse_Deploy_ert_rtw/instrumented/MicroMouse_Deploy_data.c $(START_DIR)/MicroMouse_Deploy_ert_rtw/instrumented/rt_nonfinite.c C:/Users/Student/Documents/MATLAB/tadala_main/3099/tadala_micromouseTemplate/MicroMouseProgramming_Code/Core/Src/IMU.c C:/Users/Student/Documents/MATLAB/tadala_main/3099/tadala_micromouseTemplate/MicroMouseProgramming_Code/Core/Src/Motors.c C:/Users/Student/Documents/MATLAB/tadala_main/3099/tadala_micromouseTemplate/MicroMouseProgramming_Code/Core/Src/SSD1306.c C:/Users/Student/Documents/MATLAB/tadala_main/3099/tadala_micromouseTemplate/MicroMouseProgramming_Code/Core/Src/VL53L0X.c C:/Users/Student/Documents/MATLAB/tadala_main/3099/tadala_micromouseTemplate/MicroMouseProgramming_Code/Core/Src/ADCs.c C:/Users/Student/Documents/MATLAB/tadala_main/3099/tadala_micromouseTemplate/MicroMouseProgramming_Code/Core/Src/Buttons.c C:/Users/Student/Documents/MATLAB/tadala_main/3099/tadala_micromouseTemplate/MicroMouseProgramming_Code/Core/Src/LEDs.c C:/Users/Student/Documents/MATLAB/tadala_main/3099/tadala_micromouseTemplate/MicroMouseProgramming_Code/Core/Src/INA219.c C:/Users/Student/Documents/MATLAB/tadala_main/3099/tadala_micromouseTemplate/MicroMouseProgramming_Code/Core/Src/preformatted_flash.c C:/ProgramData/MATLAB/SupportPackages/R2025a/toolbox/shared/supportpackages/stm32/src/overrideHALDelay.c C:/ProgramData/MATLAB/SupportPackages/R2025a/toolbox/shared/supportpackages/stm32/src/platform_timer.c "$(MATLAB_ROOT)/toolbox/target/shared/armcortexmbase/scheduler/src/SysTickScheduler.c" "$(MATLAB_ROOT)/toolbox/target/shared/armcortexmbase/scheduler/src/m3m4m4f_multitasking.c" C:/Users/Student/Documents/MATLAB/tadala_main/3099/tadala_micromouseTemplate/MicroMouseProgramming_Code/Core/Src/main.c C:/Users/Student/Documents/MATLAB/tadala_main/3099/tadala_micromouseTemplate/MicroMouseProgramming_Code/Core/Src/stm32l4xx_it.c C:/Users/Student/Documents/MATLAB/tadala_main/3099/tadala_micromouseTemplate/MicroMouseProgramming_Code/Core/Src/stm32l4xx_hal_msp.c C:/Users/Student/Documents/MATLAB/tadala_main/3099/tadala_micromouseTemplate/MicroMouseProgramming_Code/Core/Src/stm32l4xx_hal_timebase_tim.c C:/Users/Student/Documents/MATLAB/tadala_main/3099/tadala_micromouseTemplate/MicroMouseProgramming_Code/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_tim.c C:/Users/Student/Documents/MATLAB/tadala_main/3099/tadala_micromouseTemplate/MicroMouseProgramming_Code/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_tim_ex.c C:/Users/Student/Documents/MATLAB/tadala_main/3099/tadala_micromouseTemplate/MicroMouseProgramming_Code/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_adc.c C:/Users/Student/Documents/MATLAB/tadala_main/3099/tadala_micromouseTemplate/MicroMouseProgramming_Code/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_adc_ex.c C:/Users/Student/Documents/MATLAB/tadala_main/3099/tadala_micromouseTemplate/MicroMouseProgramming_Code/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal.c C:/Users/Student/Documents/MATLAB/tadala_main/3099/tadala_micromouseTemplate/MicroMouseProgramming_Code/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_rcc.c C:/Users/Student/Documents/MATLAB/tadala_main/3099/tadala_micromouseTemplate/MicroMouseProgramming_Code/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_rcc_ex.c C:/Users/Student/Documents/MATLAB/tadala_main/3099/tadala_micromouseTemplate/MicroMouseProgramming_Code/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_flash.c C:/Users/Student/Documents/MATLAB/tadala_main/3099/tadala_micromouseTemplate/MicroMouseProgramming_Code/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_flash_ex.c C:/Users/Student/Documents/MATLAB/tadala_main/3099/tadala_micromouseTemplate/MicroMouseProgramming_Code/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_flash_ramfunc.c C:/Users/Student/Documents/MATLAB/tadala_main/3099/tadala_micromouseTemplate/MicroMouseProgramming_Code/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_gpio.c C:/Users/Student/Documents/MATLAB/tadala_main/3099/tadala_micromouseTemplate/MicroMouseProgramming_Code/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_i2c.c C:/Users/Student/Documents/MATLAB/tadala_main/3099/tadala_micromouseTemplate/MicroMouseProgramming_Code/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_i2c_ex.c C:/Users/Student/Documents/MATLAB/tadala_main/3099/tadala_micromouseTemplate/MicroMouseProgramming_Code/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_dma.c C:/Users/Student/Documents/MATLAB/tadala_main/3099/tadala_micromouseTemplate/MicroMouseProgramming_Code/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_dma_ex.c C:/Users/Student/Documents/MATLAB/tadala_main/3099/tadala_micromouseTemplate/MicroMouseProgramming_Code/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_pwr.c C:/Users/Student/Documents/MATLAB/tadala_main/3099/tadala_micromouseTemplate/MicroMouseProgramming_Code/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_pwr_ex.c C:/Users/Student/Documents/MATLAB/tadala_main/3099/tadala_micromouseTemplate/MicroMouseProgramming_Code/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_cortex.c C:/Users/Student/Documents/MATLAB/tadala_main/3099/tadala_micromouseTemplate/MicroMouseProgramming_Code/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_exti.c C:/Users/Student/Documents/MATLAB/tadala_main/3099/tadala_micromouseTemplate/MicroMouseProgramming_Code/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_uart.c C:/Users/Student/Documents/MATLAB/tadala_main/3099/tadala_micromouseTemplate/MicroMouseProgramming_Code/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_uart_ex.c C:/Users/Student/Documents/MATLAB/tadala_main/3099/tadala_micromouseTemplate/MicroMouseProgramming_Code/Core/Src/system_stm32l4xx.c C:/Users/Student/Documents/MATLAB/tadala_main/3099/tadala_micromouseTemplate/MicroMouseProgramming_Code/STM32CubeIDE/Application/User/Core/syscalls.c C:/Users/Student/Documents/MATLAB/tadala_main/3099/tadala_micromouseTemplate/MicroMouseProgramming_Code/STM32CubeIDE/Application/User/Core/sysmem.c C:/Users/Student/Documents/MATLAB/tadala_main/3099/tadala_micromouseTemplate/MicroMouseProgramming_Code/STM32CubeIDE/Application/User/Startup/startup_stm32l476vetx.s code_profiling_utility_functions.c

MAIN_SRC = $(START_DIR)/MicroMouse_Deploy_ert_rtw/instrumented/ert_main.c

ALL_SRCS = $(SRCS) $(MAIN_SRC)

###########################################################################
## OBJECTS
###########################################################################

OBJS = MicroMouse_Deploy.c.o MicroMouse_Deploy_data.c.o rt_nonfinite.c.o IMU.c.o Motors.c.o SSD1306.c.o VL53L0X.c.o ADCs.c.o Buttons.c.o LEDs.c.o INA219.c.o preformatted_flash.c.o overrideHALDelay.c.o platform_timer.c.o SysTickScheduler.c.o m3m4m4f_multitasking.c.o main.c.o stm32l4xx_it.c.o stm32l4xx_hal_msp.c.o stm32l4xx_hal_timebase_tim.c.o stm32l4xx_hal_tim.c.o stm32l4xx_hal_tim_ex.c.o stm32l4xx_hal_adc.c.o stm32l4xx_hal_adc_ex.c.o stm32l4xx_hal.c.o stm32l4xx_hal_rcc.c.o stm32l4xx_hal_rcc_ex.c.o stm32l4xx_hal_flash.c.o stm32l4xx_hal_flash_ex.c.o stm32l4xx_hal_flash_ramfunc.c.o stm32l4xx_hal_gpio.c.o stm32l4xx_hal_i2c.c.o stm32l4xx_hal_i2c_ex.c.o stm32l4xx_hal_dma.c.o stm32l4xx_hal_dma_ex.c.o stm32l4xx_hal_pwr.c.o stm32l4xx_hal_pwr_ex.c.o stm32l4xx_hal_cortex.c.o stm32l4xx_hal_exti.c.o stm32l4xx_hal_uart.c.o stm32l4xx_hal_uart_ex.c.o system_stm32l4xx.c.o syscalls.c.o sysmem.c.o startup_stm32l476vetx.s.o code_profiling_utility_functions.c.o

MAIN_OBJ = ert_main.c.o

ALL_OBJS = $(OBJS) $(MAIN_OBJ)

###########################################################################
## PREBUILT OBJECT FILES
###########################################################################

PREBUILT_OBJS = 

###########################################################################
## LIBRARIES
###########################################################################

LIBS = C:/ProgramData/MATLAB/SupportPackages/R2025a/3P.instrset/cmsis_dsp.instrset/Lib/GCC/arm_cortexm4ldfsp_math/libCMSISDSP.a

###########################################################################
## SYSTEM LIBRARIES
###########################################################################

SYSTEM_LIBS = 

###########################################################################
## ADDITIONAL TOOLCHAIN FLAGS
###########################################################################

#---------------
# C Compiler
#---------------

CFLAGS_SKIPFORSIL = -mcpu=cortex-m4 -mthumb -mlittle-endian -mthumb-interwork -mfpu=fpv4-sp-d16 -mfloat-abi=hard -ffp-contract=fast
CFLAGS_BASIC = $(DEFINES) $(INCLUDES) @$(COMPILER_COMMAND_FILE)

CFLAGS += $(CFLAGS_SKIPFORSIL) $(CFLAGS_BASIC)

#-----------------
# C++ Compiler
#-----------------

CPPFLAGS_SKIPFORSIL = -mcpu=cortex-m4 -mthumb -mlittle-endian -mthumb-interwork -mfpu=fpv4-sp-d16 -mfloat-abi=hard -ffp-contract=fast
CPPFLAGS_BASIC = $(DEFINES) $(INCLUDES) @$(COMPILER_COMMAND_FILE)

CPPFLAGS += $(CPPFLAGS_SKIPFORSIL) $(CPPFLAGS_BASIC)

#---------------
# C++ Linker
#---------------

CPP_LDFLAGS_SKIPFORSIL = -mcpu=cortex-m4 -mthumb -mlittle-endian -mthumb-interwork -mfpu=fpv4-sp-d16 -mfloat-abi=hard --entry Reset_Handler --specs=nosys.specs  -T "C:\Users\Student\Documents\MATLAB\tadala_main\3099\tadala_micromouseTemplate\MicroMouseProgramming_Code\STM32CubeIDE\STM32L476VETX_FLASH.ld"

CPP_LDFLAGS += $(CPP_LDFLAGS_SKIPFORSIL)

#------------------------------
# C++ Shared Library Linker
#------------------------------

CPP_SHAREDLIB_LDFLAGS_SKIPFORSIL = -mcpu=cortex-m4 -mthumb -mlittle-endian -mthumb-interwork -mfpu=fpv4-sp-d16 -mfloat-abi=hard --entry Reset_Handler --specs=nosys.specs  -T "C:\Users\Student\Documents\MATLAB\tadala_main\3099\tadala_micromouseTemplate\MicroMouseProgramming_Code\STM32CubeIDE\STM32L476VETX_FLASH.ld"

CPP_SHAREDLIB_LDFLAGS += $(CPP_SHAREDLIB_LDFLAGS_SKIPFORSIL)

#-----------
# Linker
#-----------

LDFLAGS_SKIPFORSIL = -mcpu=cortex-m4 -mthumb -mlittle-endian -mthumb-interwork -mfpu=fpv4-sp-d16 -mfloat-abi=hard --entry Reset_Handler --specs=nosys.specs  -T "C:\Users\Student\Documents\MATLAB\tadala_main\3099\tadala_micromouseTemplate\MicroMouseProgramming_Code\STM32CubeIDE\STM32L476VETX_FLASH.ld"

LDFLAGS += $(LDFLAGS_SKIPFORSIL)

#---------------------
# MEX C++ Compiler
#---------------------

MEX_CPP_Compiler_BASIC =  @$(COMPILER_COMMAND_FILE)

MEX_CPPFLAGS += $(MEX_CPP_Compiler_BASIC)

#-----------------
# MEX Compiler
#-----------------

MEX_Compiler_BASIC =  @$(COMPILER_COMMAND_FILE)

MEX_CFLAGS += $(MEX_Compiler_BASIC)

#--------------------------
# Shared Library Linker
#--------------------------

SHAREDLIB_LDFLAGS_SKIPFORSIL = -mcpu=cortex-m4 -mthumb -mlittle-endian -mthumb-interwork -mfpu=fpv4-sp-d16 -mfloat-abi=hard --entry Reset_Handler --specs=nosys.specs  -T "C:\Users\Student\Documents\MATLAB\tadala_main\3099\tadala_micromouseTemplate\MicroMouseProgramming_Code\STM32CubeIDE\STM32L476VETX_FLASH.ld"

SHAREDLIB_LDFLAGS += $(SHAREDLIB_LDFLAGS_SKIPFORSIL)

###########################################################################
## INLINED COMMANDS
###########################################################################


ALL_DEPS:=$(patsubst %.o,%.dep,$(ALL_OBJS))
all:

ifndef DISABLE_GCC_FUNCTION_DATA_SECTIONS
FDATASECTIONS_FLG := -ffunction-sections -fdata-sections
endif



-include codertarget_assembly_flags.mk
-include ../codertarget_assembly_flags.mk
-include ../../codertarget_assembly_flags.mk
-include mw_gnu_arm_tools_path.mk
-include ../mw_gnu_arm_tools_path.mk
-include ../../mw_gnu_arm_tools_path.mk
-include $(ALL_DEPS)


###########################################################################
## PHONY TARGETS
###########################################################################

.PHONY : all build buildobj clean info prebuild postbuild download execute


all : build postbuild
	@echo "### Successfully generated all binary outputs."


build : prebuild $(PRODUCT)


buildobj : prebuild $(OBJS) $(PREBUILT_OBJS) $(LIBS)
	@echo "### Successfully generated all binary outputs."


prebuild : 


postbuild : $(PRODUCT)
	@echo "### Invoking postbuild tool "Binary Converter" ..."
	$(OBJCOPY) $(OBJCOPYFLAGS_BIN)
	@echo "### Done invoking postbuild tool."
	@echo "### Invoking postbuild tool "Hex Converter" ..."
	$(OBJCOPY) $(OBJCOPYFLAGS_HEX)
	@echo "### Done invoking postbuild tool."
	@echo "### Invoking postbuild tool "Executable Size" ..."
	$(EXESIZE) $(EXESIZE_FLAGS)
	@echo "### Done invoking postbuild tool."


download : postbuild


execute : download
	@echo "### Invoking postbuild tool "Execute" ..."
	$(EXECUTE) $(EXECUTE_FLAGS)
	@echo "### Done invoking postbuild tool."


###########################################################################
## FINAL TARGET
###########################################################################

#-------------------------------------------
# Create a standalone executable            
#-------------------------------------------

$(PRODUCT) : $(OBJS) $(PREBUILT_OBJS) $(LIBS) $(MAIN_OBJ)
	@echo "### Creating standalone executable "$(PRODUCT)" ..."
	$(LD) $(LDFLAGS) -o $(PRODUCT) @$(CMD_FILE) $(call CONVERT_ESCAPED_SPACES_TO_QUOTES,$(LIBS)) $(SYSTEM_LIBS) $(TOOLCHAIN_LIBS)
	@echo "### Created: $(PRODUCT)"


###########################################################################
## INTERMEDIATE TARGETS
###########################################################################

#---------------------
# SOURCE-TO-OBJECT
#---------------------

%.c.o : %.c
	$(CC) $(CFLAGS) -o "$@" "$<"


%.s.o : %.s
	$(AS) $(ASFLAGS) -o "$@" "$<"


%.s.o : %.S
	$(AS) $(ASFLAGS) -o "$@" "$<"


%.cpp.o : %.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.cpp.o : %.cc
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.cpp.o : %.C
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.cpp.o : %.cxx
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.c.o : $(RELATIVE_PATH_TO_ANCHOR)/%.c
	$(CC) $(CFLAGS) -o "$@" "$<"


%.s.o : $(RELATIVE_PATH_TO_ANCHOR)/%.s
	$(AS) $(ASFLAGS) -o "$@" "$<"


%.s.o : $(RELATIVE_PATH_TO_ANCHOR)/%.S
	$(AS) $(ASFLAGS) -o "$@" "$<"


%.cpp.o : $(RELATIVE_PATH_TO_ANCHOR)/%.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.cpp.o : $(RELATIVE_PATH_TO_ANCHOR)/%.cc
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.cpp.o : $(RELATIVE_PATH_TO_ANCHOR)/%.C
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.cpp.o : $(RELATIVE_PATH_TO_ANCHOR)/%.cxx
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.c.o : $(START_DIR)/%.c
	$(CC) $(CFLAGS) -o "$@" "$<"


%.s.o : $(START_DIR)/%.s
	$(AS) $(ASFLAGS) -o "$@" "$<"


%.s.o : $(START_DIR)/%.S
	$(AS) $(ASFLAGS) -o "$@" "$<"


%.cpp.o : $(START_DIR)/%.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.cpp.o : $(START_DIR)/%.cc
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.cpp.o : $(START_DIR)/%.C
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.cpp.o : $(START_DIR)/%.cxx
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.c.o : C:/Users/Student/Documents/MATLAB/tadala_main/3099/tadala_micromouseTemplate/MicroMouseProgramming_Code/Core/Src/%.c
	$(CC) $(CFLAGS) -o "$@" "$<"


%.s.o : C:/Users/Student/Documents/MATLAB/tadala_main/3099/tadala_micromouseTemplate/MicroMouseProgramming_Code/Core/Src/%.s
	$(AS) $(ASFLAGS) -o "$@" "$<"


%.s.o : C:/Users/Student/Documents/MATLAB/tadala_main/3099/tadala_micromouseTemplate/MicroMouseProgramming_Code/Core/Src/%.S
	$(AS) $(ASFLAGS) -o "$@" "$<"


%.cpp.o : C:/Users/Student/Documents/MATLAB/tadala_main/3099/tadala_micromouseTemplate/MicroMouseProgramming_Code/Core/Src/%.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.cpp.o : C:/Users/Student/Documents/MATLAB/tadala_main/3099/tadala_micromouseTemplate/MicroMouseProgramming_Code/Core/Src/%.cc
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.cpp.o : C:/Users/Student/Documents/MATLAB/tadala_main/3099/tadala_micromouseTemplate/MicroMouseProgramming_Code/Core/Src/%.C
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.cpp.o : C:/Users/Student/Documents/MATLAB/tadala_main/3099/tadala_micromouseTemplate/MicroMouseProgramming_Code/Core/Src/%.cxx
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.c.o : $(call SPACE_TO_QUESTION,$(MATLAB_ROOT))/rtw/c/src/%.c
	$(CC) $(CFLAGS) -o "$@" "$<"


%.s.o : $(call SPACE_TO_QUESTION,$(MATLAB_ROOT))/rtw/c/src/%.s
	$(AS) $(ASFLAGS) -o "$@" "$<"


%.s.o : $(call SPACE_TO_QUESTION,$(MATLAB_ROOT))/rtw/c/src/%.S
	$(AS) $(ASFLAGS) -o "$@" "$<"


%.cpp.o : $(call SPACE_TO_QUESTION,$(MATLAB_ROOT))/rtw/c/src/%.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.cpp.o : $(call SPACE_TO_QUESTION,$(MATLAB_ROOT))/rtw/c/src/%.cc
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.cpp.o : $(call SPACE_TO_QUESTION,$(MATLAB_ROOT))/rtw/c/src/%.C
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.cpp.o : $(call SPACE_TO_QUESTION,$(MATLAB_ROOT))/rtw/c/src/%.cxx
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.c.o : $(call SPACE_TO_QUESTION,$(MATLAB_ROOT))/simulink/src/%.c
	$(CC) $(CFLAGS) -o "$@" "$<"


%.s.o : $(call SPACE_TO_QUESTION,$(MATLAB_ROOT))/simulink/src/%.s
	$(AS) $(ASFLAGS) -o "$@" "$<"


%.s.o : $(call SPACE_TO_QUESTION,$(MATLAB_ROOT))/simulink/src/%.S
	$(AS) $(ASFLAGS) -o "$@" "$<"


%.cpp.o : $(call SPACE_TO_QUESTION,$(MATLAB_ROOT))/simulink/src/%.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.cpp.o : $(call SPACE_TO_QUESTION,$(MATLAB_ROOT))/simulink/src/%.cc
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.cpp.o : $(call SPACE_TO_QUESTION,$(MATLAB_ROOT))/simulink/src/%.C
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.cpp.o : $(call SPACE_TO_QUESTION,$(MATLAB_ROOT))/simulink/src/%.cxx
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.c.o : $(call SPACE_TO_QUESTION,$(MATLAB_ROOT))/toolbox/simulink/blocks/src/%.c
	$(CC) $(CFLAGS) -o "$@" "$<"


%.s.o : $(call SPACE_TO_QUESTION,$(MATLAB_ROOT))/toolbox/simulink/blocks/src/%.s
	$(AS) $(ASFLAGS) -o "$@" "$<"


%.s.o : $(call SPACE_TO_QUESTION,$(MATLAB_ROOT))/toolbox/simulink/blocks/src/%.S
	$(AS) $(ASFLAGS) -o "$@" "$<"


%.cpp.o : $(call SPACE_TO_QUESTION,$(MATLAB_ROOT))/toolbox/simulink/blocks/src/%.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.cpp.o : $(call SPACE_TO_QUESTION,$(MATLAB_ROOT))/toolbox/simulink/blocks/src/%.cc
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.cpp.o : $(call SPACE_TO_QUESTION,$(MATLAB_ROOT))/toolbox/simulink/blocks/src/%.C
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.cpp.o : $(call SPACE_TO_QUESTION,$(MATLAB_ROOT))/toolbox/simulink/blocks/src/%.cxx
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.c.o : C:/ProgramData/MATLAB/SupportPackages/R2025a/toolbox/shared/supportpackages/stm32/src/%.c
	$(CC) $(CFLAGS) -o "$@" "$<"


%.s.o : C:/ProgramData/MATLAB/SupportPackages/R2025a/toolbox/shared/supportpackages/stm32/src/%.s
	$(AS) $(ASFLAGS) -o "$@" "$<"


%.s.o : C:/ProgramData/MATLAB/SupportPackages/R2025a/toolbox/shared/supportpackages/stm32/src/%.S
	$(AS) $(ASFLAGS) -o "$@" "$<"


%.cpp.o : C:/ProgramData/MATLAB/SupportPackages/R2025a/toolbox/shared/supportpackages/stm32/src/%.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.cpp.o : C:/ProgramData/MATLAB/SupportPackages/R2025a/toolbox/shared/supportpackages/stm32/src/%.cc
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.cpp.o : C:/ProgramData/MATLAB/SupportPackages/R2025a/toolbox/shared/supportpackages/stm32/src/%.C
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.cpp.o : C:/ProgramData/MATLAB/SupportPackages/R2025a/toolbox/shared/supportpackages/stm32/src/%.cxx
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.c.o : $(START_DIR)/MicroMouse_Deploy_ert_rtw/instrumented/%.c
	$(CC) $(CFLAGS) -o "$@" "$<"


%.s.o : $(START_DIR)/MicroMouse_Deploy_ert_rtw/instrumented/%.s
	$(AS) $(ASFLAGS) -o "$@" "$<"


%.s.o : $(START_DIR)/MicroMouse_Deploy_ert_rtw/instrumented/%.S
	$(AS) $(ASFLAGS) -o "$@" "$<"


%.cpp.o : $(START_DIR)/MicroMouse_Deploy_ert_rtw/instrumented/%.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.cpp.o : $(START_DIR)/MicroMouse_Deploy_ert_rtw/instrumented/%.cc
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.cpp.o : $(START_DIR)/MicroMouse_Deploy_ert_rtw/instrumented/%.C
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.cpp.o : $(START_DIR)/MicroMouse_Deploy_ert_rtw/instrumented/%.cxx
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


MicroMouse_Deploy.c.o : $(START_DIR)/MicroMouse_Deploy_ert_rtw/instrumented/MicroMouse_Deploy.c
	$(CC) $(CFLAGS) -o "$@" "$<"


MicroMouse_Deploy_data.c.o : $(START_DIR)/MicroMouse_Deploy_ert_rtw/instrumented/MicroMouse_Deploy_data.c
	$(CC) $(CFLAGS) -o "$@" "$<"


ert_main.c.o : $(START_DIR)/MicroMouse_Deploy_ert_rtw/instrumented/ert_main.c
	$(CC) $(CFLAGS) -o "$@" "$<"


rt_nonfinite.c.o : $(START_DIR)/MicroMouse_Deploy_ert_rtw/instrumented/rt_nonfinite.c
	$(CC) $(CFLAGS) -o "$@" "$<"


IMU.c.o : C:/Users/Student/Documents/MATLAB/tadala_main/3099/tadala_micromouseTemplate/MicroMouseProgramming_Code/Core/Src/IMU.c
	$(CC) $(CFLAGS) -o "$@" "$<"


Motors.c.o : C:/Users/Student/Documents/MATLAB/tadala_main/3099/tadala_micromouseTemplate/MicroMouseProgramming_Code/Core/Src/Motors.c
	$(CC) $(CFLAGS) -o "$@" "$<"


SSD1306.c.o : C:/Users/Student/Documents/MATLAB/tadala_main/3099/tadala_micromouseTemplate/MicroMouseProgramming_Code/Core/Src/SSD1306.c
	$(CC) $(CFLAGS) -o "$@" "$<"


VL53L0X.c.o : C:/Users/Student/Documents/MATLAB/tadala_main/3099/tadala_micromouseTemplate/MicroMouseProgramming_Code/Core/Src/VL53L0X.c
	$(CC) $(CFLAGS) -o "$@" "$<"


ADCs.c.o : C:/Users/Student/Documents/MATLAB/tadala_main/3099/tadala_micromouseTemplate/MicroMouseProgramming_Code/Core/Src/ADCs.c
	$(CC) $(CFLAGS) -o "$@" "$<"


Buttons.c.o : C:/Users/Student/Documents/MATLAB/tadala_main/3099/tadala_micromouseTemplate/MicroMouseProgramming_Code/Core/Src/Buttons.c
	$(CC) $(CFLAGS) -o "$@" "$<"


LEDs.c.o : C:/Users/Student/Documents/MATLAB/tadala_main/3099/tadala_micromouseTemplate/MicroMouseProgramming_Code/Core/Src/LEDs.c
	$(CC) $(CFLAGS) -o "$@" "$<"


INA219.c.o : C:/Users/Student/Documents/MATLAB/tadala_main/3099/tadala_micromouseTemplate/MicroMouseProgramming_Code/Core/Src/INA219.c
	$(CC) $(CFLAGS) -o "$@" "$<"


preformatted_flash.c.o : C:/Users/Student/Documents/MATLAB/tadala_main/3099/tadala_micromouseTemplate/MicroMouseProgramming_Code/Core/Src/preformatted_flash.c
	$(CC) $(CFLAGS) -o "$@" "$<"


overrideHALDelay.c.o : C:/ProgramData/MATLAB/SupportPackages/R2025a/toolbox/shared/supportpackages/stm32/src/overrideHALDelay.c
	$(CC) $(CFLAGS) -o "$@" "$<"


platform_timer.c.o : C:/ProgramData/MATLAB/SupportPackages/R2025a/toolbox/shared/supportpackages/stm32/src/platform_timer.c
	$(CC) $(CFLAGS) -o "$@" "$<"


SysTickScheduler.c.o : $(call ESCAPE_SPACES,$(MATLAB_ROOT))/toolbox/target/shared/armcortexmbase/scheduler/src/SysTickScheduler.c
	$(CC) $(CFLAGS) -o "$@" "$<"


m3m4m4f_multitasking.c.o : $(call ESCAPE_SPACES,$(MATLAB_ROOT))/toolbox/target/shared/armcortexmbase/scheduler/src/m3m4m4f_multitasking.c
	$(CC) $(CFLAGS) -o "$@" "$<"


main.c.o : C:/Users/Student/Documents/MATLAB/tadala_main/3099/tadala_micromouseTemplate/MicroMouseProgramming_Code/Core/Src/main.c
	$(CC) $(CFLAGS) -o "$@" "$<"


stm32l4xx_it.c.o : C:/Users/Student/Documents/MATLAB/tadala_main/3099/tadala_micromouseTemplate/MicroMouseProgramming_Code/Core/Src/stm32l4xx_it.c
	$(CC) $(CFLAGS) -o "$@" "$<"


stm32l4xx_hal_msp.c.o : C:/Users/Student/Documents/MATLAB/tadala_main/3099/tadala_micromouseTemplate/MicroMouseProgramming_Code/Core/Src/stm32l4xx_hal_msp.c
	$(CC) $(CFLAGS) -o "$@" "$<"


stm32l4xx_hal_timebase_tim.c.o : C:/Users/Student/Documents/MATLAB/tadala_main/3099/tadala_micromouseTemplate/MicroMouseProgramming_Code/Core/Src/stm32l4xx_hal_timebase_tim.c
	$(CC) $(CFLAGS) -o "$@" "$<"


stm32l4xx_hal_tim.c.o : C:/Users/Student/Documents/MATLAB/tadala_main/3099/tadala_micromouseTemplate/MicroMouseProgramming_Code/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_tim.c
	$(CC) $(CFLAGS) -o "$@" "$<"


stm32l4xx_hal_tim_ex.c.o : C:/Users/Student/Documents/MATLAB/tadala_main/3099/tadala_micromouseTemplate/MicroMouseProgramming_Code/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_tim_ex.c
	$(CC) $(CFLAGS) -o "$@" "$<"


stm32l4xx_hal_adc.c.o : C:/Users/Student/Documents/MATLAB/tadala_main/3099/tadala_micromouseTemplate/MicroMouseProgramming_Code/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_adc.c
	$(CC) $(CFLAGS) -o "$@" "$<"


stm32l4xx_hal_adc_ex.c.o : C:/Users/Student/Documents/MATLAB/tadala_main/3099/tadala_micromouseTemplate/MicroMouseProgramming_Code/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_adc_ex.c
	$(CC) $(CFLAGS) -o "$@" "$<"


stm32l4xx_hal.c.o : C:/Users/Student/Documents/MATLAB/tadala_main/3099/tadala_micromouseTemplate/MicroMouseProgramming_Code/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal.c
	$(CC) $(CFLAGS) -o "$@" "$<"


stm32l4xx_hal_rcc.c.o : C:/Users/Student/Documents/MATLAB/tadala_main/3099/tadala_micromouseTemplate/MicroMouseProgramming_Code/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_rcc.c
	$(CC) $(CFLAGS) -o "$@" "$<"


stm32l4xx_hal_rcc_ex.c.o : C:/Users/Student/Documents/MATLAB/tadala_main/3099/tadala_micromouseTemplate/MicroMouseProgramming_Code/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_rcc_ex.c
	$(CC) $(CFLAGS) -o "$@" "$<"


stm32l4xx_hal_flash.c.o : C:/Users/Student/Documents/MATLAB/tadala_main/3099/tadala_micromouseTemplate/MicroMouseProgramming_Code/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_flash.c
	$(CC) $(CFLAGS) -o "$@" "$<"


stm32l4xx_hal_flash_ex.c.o : C:/Users/Student/Documents/MATLAB/tadala_main/3099/tadala_micromouseTemplate/MicroMouseProgramming_Code/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_flash_ex.c
	$(CC) $(CFLAGS) -o "$@" "$<"


stm32l4xx_hal_flash_ramfunc.c.o : C:/Users/Student/Documents/MATLAB/tadala_main/3099/tadala_micromouseTemplate/MicroMouseProgramming_Code/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_flash_ramfunc.c
	$(CC) $(CFLAGS) -o "$@" "$<"


stm32l4xx_hal_gpio.c.o : C:/Users/Student/Documents/MATLAB/tadala_main/3099/tadala_micromouseTemplate/MicroMouseProgramming_Code/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_gpio.c
	$(CC) $(CFLAGS) -o "$@" "$<"


stm32l4xx_hal_i2c.c.o : C:/Users/Student/Documents/MATLAB/tadala_main/3099/tadala_micromouseTemplate/MicroMouseProgramming_Code/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_i2c.c
	$(CC) $(CFLAGS) -o "$@" "$<"


stm32l4xx_hal_i2c_ex.c.o : C:/Users/Student/Documents/MATLAB/tadala_main/3099/tadala_micromouseTemplate/MicroMouseProgramming_Code/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_i2c_ex.c
	$(CC) $(CFLAGS) -o "$@" "$<"


stm32l4xx_hal_dma.c.o : C:/Users/Student/Documents/MATLAB/tadala_main/3099/tadala_micromouseTemplate/MicroMouseProgramming_Code/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_dma.c
	$(CC) $(CFLAGS) -o "$@" "$<"


stm32l4xx_hal_dma_ex.c.o : C:/Users/Student/Documents/MATLAB/tadala_main/3099/tadala_micromouseTemplate/MicroMouseProgramming_Code/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_dma_ex.c
	$(CC) $(CFLAGS) -o "$@" "$<"


stm32l4xx_hal_pwr.c.o : C:/Users/Student/Documents/MATLAB/tadala_main/3099/tadala_micromouseTemplate/MicroMouseProgramming_Code/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_pwr.c
	$(CC) $(CFLAGS) -o "$@" "$<"


stm32l4xx_hal_pwr_ex.c.o : C:/Users/Student/Documents/MATLAB/tadala_main/3099/tadala_micromouseTemplate/MicroMouseProgramming_Code/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_pwr_ex.c
	$(CC) $(CFLAGS) -o "$@" "$<"


stm32l4xx_hal_cortex.c.o : C:/Users/Student/Documents/MATLAB/tadala_main/3099/tadala_micromouseTemplate/MicroMouseProgramming_Code/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_cortex.c
	$(CC) $(CFLAGS) -o "$@" "$<"


stm32l4xx_hal_exti.c.o : C:/Users/Student/Documents/MATLAB/tadala_main/3099/tadala_micromouseTemplate/MicroMouseProgramming_Code/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_exti.c
	$(CC) $(CFLAGS) -o "$@" "$<"


stm32l4xx_hal_uart.c.o : C:/Users/Student/Documents/MATLAB/tadala_main/3099/tadala_micromouseTemplate/MicroMouseProgramming_Code/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_uart.c
	$(CC) $(CFLAGS) -o "$@" "$<"


stm32l4xx_hal_uart_ex.c.o : C:/Users/Student/Documents/MATLAB/tadala_main/3099/tadala_micromouseTemplate/MicroMouseProgramming_Code/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_uart_ex.c
	$(CC) $(CFLAGS) -o "$@" "$<"


system_stm32l4xx.c.o : C:/Users/Student/Documents/MATLAB/tadala_main/3099/tadala_micromouseTemplate/MicroMouseProgramming_Code/Core/Src/system_stm32l4xx.c
	$(CC) $(CFLAGS) -o "$@" "$<"


syscalls.c.o : C:/Users/Student/Documents/MATLAB/tadala_main/3099/tadala_micromouseTemplate/MicroMouseProgramming_Code/STM32CubeIDE/Application/User/Core/syscalls.c
	$(CC) $(CFLAGS) -o "$@" "$<"


sysmem.c.o : C:/Users/Student/Documents/MATLAB/tadala_main/3099/tadala_micromouseTemplate/MicroMouseProgramming_Code/STM32CubeIDE/Application/User/Core/sysmem.c
	$(CC) $(CFLAGS) -o "$@" "$<"


startup_stm32l476vetx.s.o : C:/Users/Student/Documents/MATLAB/tadala_main/3099/tadala_micromouseTemplate/MicroMouseProgramming_Code/STM32CubeIDE/Application/User/Startup/startup_stm32l476vetx.s
	$(AS) $(ASFLAGS) -o "$@" "$<"


###########################################################################
## DEPENDENCIES
###########################################################################

$(ALL_OBJS) : rtw_proj.tmw $(COMPILER_COMMAND_FILE) $(MAKEFILE)


###########################################################################
## MISCELLANEOUS TARGETS
###########################################################################

info : 
	@echo "### PRODUCT = $(PRODUCT)"
	@echo "### PRODUCT_TYPE = $(PRODUCT_TYPE)"
	@echo "### BUILD_TYPE = $(BUILD_TYPE)"
	@echo "### INCLUDES = $(INCLUDES)"
	@echo "### DEFINES = $(DEFINES)"
	@echo "### ALL_SRCS = $(ALL_SRCS)"
	@echo "### ALL_OBJS = $(ALL_OBJS)"
	@echo "### LIBS = $(LIBS)"
	@echo "### MODELREF_LIBS = $(MODELREF_LIBS)"
	@echo "### SYSTEM_LIBS = $(SYSTEM_LIBS)"
	@echo "### TOOLCHAIN_LIBS = $(TOOLCHAIN_LIBS)"
	@echo "### ASFLAGS = $(ASFLAGS)"
	@echo "### CFLAGS = $(CFLAGS)"
	@echo "### LDFLAGS = $(LDFLAGS)"
	@echo "### SHAREDLIB_LDFLAGS = $(SHAREDLIB_LDFLAGS)"
	@echo "### CPPFLAGS = $(CPPFLAGS)"
	@echo "### CPP_LDFLAGS = $(CPP_LDFLAGS)"
	@echo "### CPP_SHAREDLIB_LDFLAGS = $(CPP_SHAREDLIB_LDFLAGS)"
	@echo "### ARFLAGS = $(ARFLAGS)"
	@echo "### MEX_CFLAGS = $(MEX_CFLAGS)"
	@echo "### MEX_CPPFLAGS = $(MEX_CPPFLAGS)"
	@echo "### MEX_LDFLAGS = $(MEX_LDFLAGS)"
	@echo "### MEX_CPPLDFLAGS = $(MEX_CPPLDFLAGS)"
	@echo "### OBJCOPYFLAGS_BIN = $(OBJCOPYFLAGS_BIN)"
	@echo "### OBJCOPYFLAGS_HEX = $(OBJCOPYFLAGS_HEX)"
	@echo "### EXESIZE_FLAGS = $(EXESIZE_FLAGS)"
	@echo "### DOWNLOAD_FLAGS = $(DOWNLOAD_FLAGS)"
	@echo "### EXECUTE_FLAGS = $(EXECUTE_FLAGS)"
	@echo "### MAKE_FLAGS = $(MAKE_FLAGS)"


clean : 
	$(ECHO) "### Deleting all derived files ..."
	$(RM) $(subst /,\,$(PRODUCT))
	$(RM) $(subst /,\,$(ALL_OBJS))
	$(RM) *.c.dep
	$(RM) *.cpp.dep
	$(RM) *.s.dep
	$(ECHO) "### Deleted all derived files."


