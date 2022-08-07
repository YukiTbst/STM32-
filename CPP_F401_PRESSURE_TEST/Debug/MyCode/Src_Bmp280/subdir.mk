################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../MyCode/Src_Bmp280/BMP280.c 

C_DEPS += \
./MyCode/Src_Bmp280/BMP280.d 

OBJS += \
./MyCode/Src_Bmp280/BMP280.o 


# Each subdirectory must supply rules for building sources it contributes
MyCode/Src_Bmp280/%.o: ../MyCode/Src_Bmp280/%.c MyCode/Src_Bmp280/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F401xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/vortex/Desktop/stm32_cube_proj/CPP_F401_PRESSURE_TEST/MyCode/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-MyCode-2f-Src_Bmp280

clean-MyCode-2f-Src_Bmp280:
	-$(RM) ./MyCode/Src_Bmp280/BMP280.d ./MyCode/Src_Bmp280/BMP280.o

.PHONY: clean-MyCode-2f-Src_Bmp280

