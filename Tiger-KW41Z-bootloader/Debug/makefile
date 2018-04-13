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
all: FRDM-KW41Z-bootloader-vX.Y.Z.axf

# Tool invocations
FRDM-KW41Z-bootloader-vX.Y.Z.axf: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: MCU Linker'
	arm-none-eabi-gcc -Xlinker -Map="FRDM-KW41Z-bootloader-vX.Y.Z.map" -Xlinker --gc-sections -Xlinker -print-memory-usage -mcpu=cortex-m0plus -mthumb -T Bootloader.ld -L ../source -o "FRDM-KW41Z-bootloader-vX.Y.Z.axf" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '
	$(MAKE) --no-print-directory post-build

# Other Targets
clean:
	-$(RM) $(EXECUTABLES)$(OBJS)$(C_DEPS) FRDM-KW41Z-bootloader-vX.Y.Z.axf
	-@echo ' '

post-build:
	-@echo 'Performing post-build steps'
	-arm-none-eabi-size "FRDM-KW41Z-bootloader-vX.Y.Z.axf" ; arm-none-eabi-objcopy -v -O binary "FRDM-KW41Z-bootloader-vX.Y.Z.axf" "FRDM-KW41Z-bootloader-vX.Y.Z.bin" ; checksum -p MKW41Z512xxx4 -d "FRDM-KW41Z-bootloader-vX.Y.Z.bin"
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY: post-build

-include ../makefile.targets
