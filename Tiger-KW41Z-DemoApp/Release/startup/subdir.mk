################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_UPPER_SRCS += \
../startup/startup_MKW41Z4.S 

OBJS += \
./startup/startup_MKW41Z4.o 


# Each subdirectory must supply rules for building sources it contributes
startup/%.o: ../startup/%.S
	@echo 'Building file: $<'
	@echo 'Invoking: MCU Assembler'
	arm-none-eabi-gcc -c -x assembler-with-cpp -D__REDLIB__ -I"C:\NXP_Tiger\Tiger-KW41Z-DemoApp\source" -I"C:\NXP_Tiger\Tiger-KW41Z-DemoApp" -I"C:\NXP_Tiger\Tiger-KW41Z-DemoApp\framework\FunctionLib" -I"C:\NXP_Tiger\Tiger-KW41Z-DemoApp\drivers" -I"C:\NXP_Tiger\Tiger-KW41Z-DemoApp\nwk_ip\core\interface" -I"C:\NXP_Tiger\Tiger-KW41Z-DemoApp\nwk_ip\core\interface\modules" -I"C:\NXP_Tiger\Tiger-KW41Z-DemoApp\nwk_ip\core\interface\thread" -I"C:\NXP_Tiger\Tiger-KW41Z-DemoApp\nwk_ip\base\interface" -I"C:\NXP_Tiger\Tiger-KW41Z-DemoApp\framework\FSCI\Interface" -I"C:\NXP_Tiger\Tiger-KW41Z-DemoApp\framework\FSCI\Source" -I"C:\NXP_Tiger\Tiger-KW41Z-DemoApp\framework\OSAbstraction\Interface" -I"C:\NXP_Tiger\Tiger-KW41Z-DemoApp\ieee_802.15.4\phy\source\MKW41Z" -I"C:\NXP_Tiger\Tiger-KW41Z-DemoApp\framework\Flash\External\Interface" -I"C:\NXP_Tiger\Tiger-KW41Z-DemoApp\nwk_ip\app\config" -I"C:\NXP_Tiger\Tiger-KW41Z-DemoApp\framework\TimersManager\Interface" -I"C:\NXP_Tiger\Tiger-KW41Z-DemoApp\framework\TimersManager\Source" -I"C:\NXP_Tiger\Tiger-KW41Z-DemoApp\framework\LowPower\Interface\MKW41Z" -I"C:\NXP_Tiger\Tiger-KW41Z-DemoApp\framework\LowPower\Source\MKW41Z" -I"C:\NXP_Tiger\Tiger-KW41Z-DemoApp\nwk_ip\app\common" -I"C:\NXP_Tiger\Tiger-KW41Z-DemoApp\framework\common" -I"C:\NXP_Tiger\Tiger-KW41Z-DemoApp\ieee_802.15.4\phy\interface" -I"C:\NXP_Tiger\Tiger-KW41Z-DemoApp\freertos" -I"C:\NXP_Tiger\Tiger-KW41Z-DemoApp\framework\SecLib" -I"C:\NXP_Tiger\Tiger-KW41Z-DemoApp\framework\SerialManager\Source\SPI_Adapter" -I"C:\NXP_Tiger\Tiger-KW41Z-DemoApp\framework\SerialManager\Interface" -I"C:\NXP_Tiger\Tiger-KW41Z-DemoApp\framework\ModuleInfo" -I"C:\NXP_Tiger\Tiger-KW41Z-DemoApp\framework\Panic\Interface" -I"C:\NXP_Tiger\Tiger-KW41Z-DemoApp\framework\Shell\Interface" -I"C:\NXP_Tiger\Tiger-KW41Z-DemoApp\board" -I"C:\NXP_Tiger\Tiger-KW41Z-DemoApp\framework\RNG\Interface" -I"C:\NXP_Tiger\Tiger-KW41Z-DemoApp\CMSIS" -I"C:\NXP_Tiger\Tiger-KW41Z-DemoApp\framework\Keyboard\Interface" -I"C:\NXP_Tiger\Tiger-KW41Z-DemoApp\framework\SerialManager\Source\I2C_Adapter" -I"C:\NXP_Tiger\Tiger-KW41Z-DemoApp\framework\LED\Interface" -I"C:\NXP_Tiger\Tiger-KW41Z-DemoApp\framework\MemManager\Interface" -I"C:\NXP_Tiger\Tiger-KW41Z-DemoApp\framework\GPIO" -I"C:\NXP_Tiger\Tiger-KW41Z-DemoApp\framework\DCDC\Interface" -I"C:\NXP_Tiger\Tiger-KW41Z-DemoApp\framework\NVM\Interface" -I"C:\NXP_Tiger\Tiger-KW41Z-DemoApp\framework\NVM\Source" -I"C:\NXP_Tiger\Tiger-KW41Z-DemoApp\framework\Messaging\Interface" -I"C:\NXP_Tiger\Tiger-KW41Z-DemoApp\framework\SerialManager\Source\UART_Adapter" -I"C:\NXP_Tiger\Tiger-KW41Z-DemoApp\framework\OtaSupport\Interface" -I"C:\NXP_Tiger\Tiger-KW41Z-DemoApp\framework\Lists" -I"C:\NXP_Tiger\Tiger-KW41Z-DemoApp\utilities" -I"C:\NXP_Tiger\Tiger-KW41Z-DemoApp\framework\MWSCoexistence\Interface" -I"C:\NXP_Tiger\Tiger-KW41Z-DemoApp\ieee_802.15.4\mac\interface" -I"C:\NXP_Tiger\Tiger-KW41Z-DemoApp\framework\XCVR\MKW41Z4" -I"C:\NXP_Tiger\Tiger-KW41Z-DemoApp\ieee_802.15.4\mac\source\App" -I"C:\NXP_Tiger\Tiger-KW41Z-DemoApp\framework\Flash\Internal" -mcpu=cortex-m0plus -mthumb -D__REDLIB__ -specs=redlib.specs -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


