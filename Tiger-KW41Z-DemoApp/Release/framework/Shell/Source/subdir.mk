################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../framework/Shell/Source/shell.c \
../framework/Shell/Source/shell_autocomplete.c \
../framework/Shell/Source/shell_cmdhelp.c \
../framework/Shell/Source/shell_cmdhist.c 

OBJS += \
./framework/Shell/Source/shell.o \
./framework/Shell/Source/shell_autocomplete.o \
./framework/Shell/Source/shell_cmdhelp.o \
./framework/Shell/Source/shell_cmdhist.o 

C_DEPS += \
./framework/Shell/Source/shell.d \
./framework/Shell/Source/shell_autocomplete.d \
./framework/Shell/Source/shell_cmdhelp.d \
./framework/Shell/Source/shell_cmdhist.d 


# Each subdirectory must supply rules for building sources it contributes
framework/Shell/Source/%.o: ../framework/Shell/Source/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU C Compiler'
	arm-none-eabi-gcc -std=gnu99 -DCR_INTEGER_PRINTF -DFSL_RTOS_FREE_RTOS -DFRDM_KW41Z -DFREEDOM -DSDK_DEBUGCONSOLE=0 -DSDK_DEBUGCONSOLE_UART -D__MCUXPRESSO -D__USE_CMSIS -DNDEBUG -DSDK_OS_FREE_RTOS -DCPU_MKW41Z512VHT4 -DCPU_MKW41Z512VHT4_cm0plus -D__REDLIB__ -DVT_KW41Z_DEMO=1 -DACCELEROMETER_ENABLE=1 -DgOtaCurrentFileVersionNo_c=0x20010000 -DgOtaCurrentFileVersion_c=0x00,0x00,0x00,0x02 -DI2C_FSL=1 -DHAVE_STDBOOL_H -DHAVE_STDINT_H -DgDCDC_Enabled_d -I"C:\NXP_Tiger\Tiger-KW41Z-DemoApp\source" -I"C:\NXP_Tiger\Tiger-KW41Z-DemoApp" -I"C:\NXP_Tiger\Tiger-KW41Z-DemoApp\framework\FunctionLib" -I"C:\NXP_Tiger\Tiger-KW41Z-DemoApp\drivers" -I"C:\NXP_Tiger\Tiger-KW41Z-DemoApp\nwk_ip\core\interface" -I"C:\NXP_Tiger\Tiger-KW41Z-DemoApp\nwk_ip\core\interface\modules" -I"C:\NXP_Tiger\Tiger-KW41Z-DemoApp\nwk_ip\core\interface\thread" -I"C:\NXP_Tiger\Tiger-KW41Z-DemoApp\nwk_ip\base\interface" -I"C:\NXP_Tiger\Tiger-KW41Z-DemoApp\framework\FSCI\Interface" -I"C:\NXP_Tiger\Tiger-KW41Z-DemoApp\framework\FSCI\Source" -I"C:\NXP_Tiger\Tiger-KW41Z-DemoApp\framework\OSAbstraction\Interface" -I"C:\NXP_Tiger\Tiger-KW41Z-DemoApp\ieee_802.15.4\phy\source\MKW41Z" -I"C:\NXP_Tiger\Tiger-KW41Z-DemoApp\framework\Flash\External\Interface" -I"C:\NXP_Tiger\Tiger-KW41Z-DemoApp\nwk_ip\app\config" -I"C:\NXP_Tiger\Tiger-KW41Z-DemoApp\framework\TimersManager\Interface" -I"C:\NXP_Tiger\Tiger-KW41Z-DemoApp\framework\TimersManager\Source" -I"C:\NXP_Tiger\Tiger-KW41Z-DemoApp\framework\LowPower\Interface\MKW41Z" -I"C:\NXP_Tiger\Tiger-KW41Z-DemoApp\framework\LowPower\Source\MKW41Z" -I"C:\NXP_Tiger\Tiger-KW41Z-DemoApp\nwk_ip\app\common" -I"C:\NXP_Tiger\Tiger-KW41Z-DemoApp\framework\common" -I"C:\NXP_Tiger\Tiger-KW41Z-DemoApp\ieee_802.15.4\phy\interface" -I"C:\NXP_Tiger\Tiger-KW41Z-DemoApp\freertos" -I"C:\NXP_Tiger\Tiger-KW41Z-DemoApp\framework\SecLib" -I"C:\NXP_Tiger\Tiger-KW41Z-DemoApp\framework\SerialManager\Source\SPI_Adapter" -I"C:\NXP_Tiger\Tiger-KW41Z-DemoApp\framework\SerialManager\Interface" -I"C:\NXP_Tiger\Tiger-KW41Z-DemoApp\framework\ModuleInfo" -I"C:\NXP_Tiger\Tiger-KW41Z-DemoApp\framework\Panic\Interface" -I"C:\NXP_Tiger\Tiger-KW41Z-DemoApp\framework\Shell\Interface" -I"C:\NXP_Tiger\Tiger-KW41Z-DemoApp\board" -I"C:\NXP_Tiger\Tiger-KW41Z-DemoApp\framework\RNG\Interface" -I"C:\NXP_Tiger\Tiger-KW41Z-DemoApp\CMSIS" -I"C:\NXP_Tiger\Tiger-KW41Z-DemoApp\framework\Keyboard\Interface" -I"C:\NXP_Tiger\Tiger-KW41Z-DemoApp\framework\SerialManager\Source\I2C_Adapter" -I"C:\NXP_Tiger\Tiger-KW41Z-DemoApp\framework\LED\Interface" -I"C:\NXP_Tiger\Tiger-KW41Z-DemoApp\framework\MemManager\Interface" -I"C:\NXP_Tiger\Tiger-KW41Z-DemoApp\framework\GPIO" -I"C:\NXP_Tiger\Tiger-KW41Z-DemoApp\framework\DCDC\Interface" -I"C:\NXP_Tiger\Tiger-KW41Z-DemoApp\framework\NVM\Interface" -I"C:\NXP_Tiger\Tiger-KW41Z-DemoApp\framework\NVM\Source" -I"C:\NXP_Tiger\Tiger-KW41Z-DemoApp\framework\Messaging\Interface" -I"C:\NXP_Tiger\Tiger-KW41Z-DemoApp\framework\SerialManager\Source\UART_Adapter" -I"C:\NXP_Tiger\Tiger-KW41Z-DemoApp\framework\OtaSupport\Interface" -I"C:\NXP_Tiger\Tiger-KW41Z-DemoApp\framework\Lists" -I"C:\NXP_Tiger\Tiger-KW41Z-DemoApp\utilities" -I"C:\NXP_Tiger\Tiger-KW41Z-DemoApp\framework\MWSCoexistence\Interface" -I"C:\NXP_Tiger\Tiger-KW41Z-DemoApp\ieee_802.15.4\mac\interface" -I"C:\NXP_Tiger\Tiger-KW41Z-DemoApp\framework\XCVR\MKW41Z4" -I"C:\NXP_Tiger\Tiger-KW41Z-DemoApp\ieee_802.15.4\mac\source\App" -I"C:\NXP_Tiger\Tiger-KW41Z-DemoApp\framework\Flash\Internal" -Os -fno-common -g -Wall -Wno-missing-braces  -c  -ffunction-sections  -fdata-sections  -ffreestanding  -fno-builtin -imacros "C:/NXP_Tiger/Tiger-KW41Z-DemoApp/source/config.h" -mcpu=cortex-m0plus -mthumb -D__REDLIB__ -specs=redlib.specs -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


