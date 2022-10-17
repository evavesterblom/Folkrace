################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../VL53L1X/VL53L1X.c 

OBJS += \
./VL53L1X/VL53L1X.o 

C_DEPS += \
./VL53L1X/VL53L1X.d 


# Each subdirectory must supply rules for building sources it contributes
VL53L1X/%.o VL53L1X/%.su: ../VL53L1X/%.c VL53L1X/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F030x8 -c -I../Core/Inc -I../Drivers/STM32F0xx_HAL_Driver/Inc -I../Drivers/STM32F0xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F0xx/Include -I../Drivers/CMSIS/Include -I"/Users/macbookpro/Desktop/ROBOTIKLUBI/viper/VL53L1X" -I"/Users/macbookpro/Desktop/ROBOTIKLUBI/viper/VL53L1X/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-VL53L1X

clean-VL53L1X:
	-$(RM) ./VL53L1X/VL53L1X.d ./VL53L1X/VL53L1X.o ./VL53L1X/VL53L1X.su

.PHONY: clean-VL53L1X

