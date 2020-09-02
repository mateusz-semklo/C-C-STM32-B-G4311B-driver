################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Include/subdir.mk
-include Drivers/STM32G4xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
B-G4311B-driver.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
B-G4311B-driver.list \

OBJCOPY_BIN += \
B-G4311B-driver.bin \


# All Target
all: B-G4311B-driver.elf secondary-outputs

# Tool invocations
B-G4311B-driver.elf: $(OBJS) $(USER_OBJS) C:\Users\Mateusz\STM32CubeIDE\workspace_1.3.0\B-G4311B-driver\STM32G431CBUX_FLASH.ld
	arm-none-eabi-gcc -o "B-G4311B-driver.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"C:\Users\Mateusz\STM32CubeIDE\workspace_1.3.0\B-G4311B-driver\STM32G431CBUX_FLASH.ld" --specs=nosys.specs -Wl,-Map="B-G4311B-driver.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -u _printf_float -u _scanf_float -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

B-G4311B-driver.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "B-G4311B-driver.list"
	@echo 'Finished building: $@'
	@echo ' '

B-G4311B-driver.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "B-G4311B-driver.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_BIN)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
