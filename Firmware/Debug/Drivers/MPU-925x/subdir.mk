################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/MPU-925x/mpu925x_spi_hal.c 

OBJS += \
./Drivers/MPU-925x/mpu925x_spi_hal.o 

C_DEPS += \
./Drivers/MPU-925x/mpu925x_spi_hal.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/MPU-925x/%.o Drivers/MPU-925x/%.su: ../Drivers/MPU-925x/%.c Drivers/MPU-925x/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32L476xx -DUSE_HAL_DRIVER -DUSE_FULL_LL_DRIVER -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/SH2 -I../Drivers/BNO08x -I../Drivers/AKxx-x -I../Drivers/MPU-925x -I../Drivers/LL_Drivers -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-MPU-2d-925x

clean-Drivers-2f-MPU-2d-925x:
	-$(RM) ./Drivers/MPU-925x/mpu925x_spi_hal.d ./Drivers/MPU-925x/mpu925x_spi_hal.o ./Drivers/MPU-925x/mpu925x_spi_hal.su

.PHONY: clean-Drivers-2f-MPU-2d-925x

