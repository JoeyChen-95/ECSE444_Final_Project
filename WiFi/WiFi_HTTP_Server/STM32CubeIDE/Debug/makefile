################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32L4xx_HAL_Driver/subdir.mk
-include Drivers/BSP/Components/HTS221/subdir.mk
-include Drivers/BSP/subdir.mk
-include Application/WiFi/subdir.mk
-include Application/User/subdir.mk
-include Application/Startup/subdir.mk
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


BUILD_ARTIFACT_NAME := WiFi_HTTP_Server
BUILD_ARTIFACT_EXTENSION := elf
BUILD_ARTIFACT_PREFIX :=
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME)$(if $(BUILD_ARTIFACT_EXTENSION),.$(BUILD_ARTIFACT_EXTENSION),)

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
WiFi_HTTP_Server.elf \

MAP_FILES += \
WiFi_HTTP_Server.map \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
WiFi_HTTP_Server.list \

OBJCOPY_HEX += \
WiFi_HTTP_Server.hex \


# All Target
all: main-build

# Main-build Target
main-build: WiFi_HTTP_Server.elf secondary-outputs

# Tool invocations
WiFi_HTTP_Server.elf WiFi_HTTP_Server.map: $(OBJS) $(USER_OBJS) C:\Users\Mr.Chimpanzee\STM32Cube\Repository\STM32Cube_FW_L4_V1.17.2\Projects\B-L4S5I-IOT01A\Applications\WiFi\WiFi_HTTP_Server\STM32CubeIDE\STM32L4S5VITX_FLASH.ld makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-gcc -o "WiFi_HTTP_Server.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"C:\Users\Mr.Chimpanzee\STM32Cube\Repository\STM32Cube_FW_L4_V1.17.2\Projects\B-L4S5I-IOT01A\Applications\WiFi\WiFi_HTTP_Server\STM32CubeIDE\STM32L4S5VITX_FLASH.ld" --specs=nosys.specs -Wl,-Map="WiFi_HTTP_Server.map" -Wl,--gc-sections -static -L"C:\Users\Mr.Chimpanzee\STM32Cube\Repository\STM32Cube_FW_L4_V1.17.2\Projects\B-L4S5I-IOT01A\Applications\WiFi\WiFi_HTTP_Server\lib" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -u _printf_float -u _scanf_float -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

WiFi_HTTP_Server.list: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "WiFi_HTTP_Server.list"
	@echo 'Finished building: $@'
	@echo ' '

WiFi_HTTP_Server.hex: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objcopy  -O ihex $(EXECUTABLES) "WiFi_HTTP_Server.hex"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) WiFi_HTTP_Server.elf WiFi_HTTP_Server.hex WiFi_HTTP_Server.list WiFi_HTTP_Server.map default.size.stdout
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_HEX)

fail-specified-linker-script-missing:
	@echo 'Error: Cannot find the specified linker script. Check the linker settings in the build configuration.'
	@exit 2

warn-no-linker-script-specified:
	@echo 'Warning: No linker script specified. Check the linker settings in the build configuration.'

.PHONY: all clean dependents main-build fail-specified-linker-script-missing warn-no-linker-script-specified

-include ../makefile.targets
