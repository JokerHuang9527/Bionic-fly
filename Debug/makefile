################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32F1xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

OPTIONAL_TOOL_DEPS := \
$(wildcard ../makefile.defs) \
$(wildcard ../makefile.init) \
$(wildcard ../makefile.targets) \


BUILD_ARTIFACT_NAME := stm32f103rb_1104_bldc_as5047p
BUILD_ARTIFACT_EXTENSION := elf
BUILD_ARTIFACT_PREFIX :=
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME)$(if $(BUILD_ARTIFACT_EXTENSION),.$(BUILD_ARTIFACT_EXTENSION),)

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
stm32f103rb_1104_bldc_as5047p.elf \

MAP_FILES += \
stm32f103rb_1104_bldc_as5047p.map \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
stm32f103rb_1104_bldc_as5047p.list \

OBJCOPY_HEX += \
stm32f103rb_1104_bldc_as5047p.hex \

OBJCOPY_BIN += \
stm32f103rb_1104_bldc_as5047p.bin \


# All Target
all: main-build

# Main-build Target
main-build: stm32f103rb_1104_bldc_as5047p.elf secondary-outputs

# Tool invocations
stm32f103rb_1104_bldc_as5047p.elf stm32f103rb_1104_bldc_as5047p.map: $(OBJS) $(USER_OBJS) D:\stm32f103rb_1104_bldc_as5047p(2024.03.06)\STM32F103RBTX_FLASH.ld makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-gcc -o "stm32f103rb_1104_bldc_as5047p.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m3 -T"D:\stm32f103rb_1104_bldc_as5047p(2024.03.06)\STM32F103RBTX_FLASH.ld" --specs=nosys.specs -Wl,-Map="stm32f103rb_1104_bldc_as5047p.map" -Wl,--gc-sections -static --specs=nano.specs -mfloat-abi=soft -mthumb -u _printf_float -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

stm32f103rb_1104_bldc_as5047p.list: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "stm32f103rb_1104_bldc_as5047p.list"
	@echo 'Finished building: $@'
	@echo ' '

stm32f103rb_1104_bldc_as5047p.hex: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objcopy  -O ihex $(EXECUTABLES) "stm32f103rb_1104_bldc_as5047p.hex"
	@echo 'Finished building: $@'
	@echo ' '

stm32f103rb_1104_bldc_as5047p.bin: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "stm32f103rb_1104_bldc_as5047p.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) default.size.stdout stm32f103rb_1104_bldc_as5047p.bin stm32f103rb_1104_bldc_as5047p.elf stm32f103rb_1104_bldc_as5047p.hex stm32f103rb_1104_bldc_as5047p.list stm32f103rb_1104_bldc_as5047p.map
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_HEX) $(OBJCOPY_BIN)

fail-specified-linker-script-missing:
	@echo 'Error: Cannot find the specified linker script. Check the linker settings in the build configuration.'
	@exit 2

warn-no-linker-script-specified:
	@echo 'Warning: No linker script specified. Check the linker settings in the build configuration.'

.PHONY: all clean dependents main-build fail-specified-linker-script-missing warn-no-linker-script-specified

-include ../makefile.targets
