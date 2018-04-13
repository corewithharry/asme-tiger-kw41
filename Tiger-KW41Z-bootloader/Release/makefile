################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include utilities/subdir.mk
-include source/drivers/uart/subdir.mk
-include source/drivers/timer/subdir.mk
-include source/drivers/spi/subdir.mk
-include source/drivers/flash/subdir.mk
-include source/drivers/eeprom/subdir.mk
-include source/subdir.mk
-include drivers/subdir.mk
-include CMSIS/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all: Tiger-KW41Z-bootloader.axf

# Tool invocations
Tiger-KW41Z-bootloader.axf: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: MCU Linker'
	arm-none-eabi-gcc -Xlinker -Map="Tiger-KW41Z-bootloader.map" -Xlinker --gc-sections -Xlinker -print-memory-usage -mcpu=cortex-m0plus -mthumb -T Bootloader.ld -L ../source -o "Tiger-KW41Z-bootloader.axf" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '
	$(MAKE) --no-print-directory post-build

# Other Targets
clean:
	-$(RM) $(EXECUTABLES)$(OBJS)$(C_DEPS) Tiger-KW41Z-bootloader.axf
	-@echo ' '

post-build:
	-@echo 'Performing post-build steps'
	-arm-none-eabi-size "Tiger-KW41Z-bootloader.axf" ; arm-none-eabi-objcopy -v -O binary "Tiger-KW41Z-bootloader.axf" "Tiger-KW41Z-bootloader.bin" ; checksum -p MKW41Z512xxx4 -d "Tiger-KW41Z-bootloader.bin"
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY: post-build

-include ../makefile.targets
