################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32F1xx_HAL_Driver/Src/subdir.mk
-include Core/fatfs/subdir.mk
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
HMI_Core.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
HMI_Core.list \

OBJCOPY_HEX += \
HMI_Core.hex \

OBJCOPY_SREC += \
HMI_Core.srec \


# All Target
all: HMI_Core.elf secondary-outputs

# Tool invocations
HMI_Core.elf: $(OBJS) $(USER_OBJS) C:\Users\engin\Documents\GitHub\Kies_HMI\sw\HMI_Core\STM32F103C8TX_FLASH.ld
	arm-none-eabi-gcc -o "HMI_Core.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m3 -T"C:\Users\engin\Documents\GitHub\Kies_HMI\sw\HMI_Core\STM32F103C8TX_FLASH.ld" --specs=nosys.specs -Wl,-Map="HMI_Core.map" -Wl,--gc-sections -static --specs=nano.specs -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

HMI_Core.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "HMI_Core.list"
	@echo 'Finished building: $@'
	@echo ' '

HMI_Core.hex: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O ihex $(EXECUTABLES) "HMI_Core.hex"
	@echo 'Finished building: $@'
	@echo ' '

HMI_Core.srec: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O srec $(EXECUTABLES) "HMI_Core.srec"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_HEX) $(OBJCOPY_SREC)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
