################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../drivers/fsl_clock.c \
../drivers/fsl_common.c \
../drivers/fsl_flash.c \
../drivers/fsl_lpuart.c \
../drivers/fsl_smc.c 

OBJS += \
./drivers/fsl_clock.o \
./drivers/fsl_common.o \
./drivers/fsl_flash.o \
./drivers/fsl_lpuart.o \
./drivers/fsl_smc.o 

C_DEPS += \
./drivers/fsl_clock.d \
./drivers/fsl_common.d \
./drivers/fsl_flash.d \
./drivers/fsl_lpuart.d \
./drivers/fsl_smc.d 


# Each subdirectory must supply rules for building sources it contributes
drivers/%.o: ../drivers/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU C Compiler'
	arm-none-eabi-gcc -std=gnu99 -DCR_INTEGER_PRINTF -DDEBUG -DgUseOTAPBootloader_d=1 -DFRDM_KW41Z -DFREEDOM -DSDK_DEBUGCONSOLE=0 -DSDK_DEBUGCONSOLE_UART -D__MCUXPRESSO -D__USE_CMSIS -DCPU_MKW41Z512VHT4 -DCPU_MKW41Z512VHT4_cm0plus -D__REDLIB__ -I"C:\rcs\NXP_Tiger\sw_frdm\FRDM-KW41Z-bootloader-MCU\source" -I"C:\rcs\NXP_Tiger\sw_frdm\FRDM-KW41Z-bootloader-MCU" -I"C:\rcs\NXP_Tiger\sw_frdm\FRDM-KW41Z-bootloader-MCU\framework\common" -I"C:\rcs\NXP_Tiger\sw_frdm\FRDM-KW41Z-bootloader-MCU\drivers" -I"C:\rcs\NXP_Tiger\sw_frdm\FRDM-KW41Z-bootloader-MCU\CMSIS" -I"C:\rcs\NXP_Tiger\sw_frdm\FRDM-KW41Z-bootloader-MCU\utilities" -I"C:\rcs\NXP_Tiger\sw_frdm\FRDM-KW41Z-bootloader-MCU\source\config" -I"C:\rcs\NXP_Tiger\sw_frdm\FRDM-KW41Z-bootloader-MCU\source\drivers\eeprom" -I"C:\rcs\NXP_Tiger\sw_frdm\FRDM-KW41Z-bootloader-MCU\source\drivers\flash" -I"C:\rcs\NXP_Tiger\sw_frdm\FRDM-KW41Z-bootloader-MCU\source\drivers\spi" -I"C:\rcs\NXP_Tiger\sw_frdm\FRDM-KW41Z-bootloader-MCU\source\drivers\timer" -I"C:\rcs\NXP_Tiger\sw_frdm\FRDM-KW41Z-bootloader-MCU\source\drivers\uart" -O0 -fno-common -g -Wall -c  -ffunction-sections  -fdata-sections  -ffreestanding  -fno-builtin -mcpu=cortex-m0plus -mthumb -D__REDLIB__ -specs=redlib.specs -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


